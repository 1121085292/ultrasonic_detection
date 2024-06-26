#include "ultrasonic_perception_component.h"

#include "cyber/time/clock.h"

using Clock = apollo::cyber::Clock;

bool UltrasonicComponent::Init() {
  // 导入超声波雷达检测配置文件
  UltrasonicConfig ultra_config;
  if (!apollo::cyber::common::GetProtoFromFile(config_file_path_,
                                               &ultra_config)) {
    AERROR << "Read config failed: " << config_file_path_;
    return false;
  }
  ADEBUG << "Ultrasonic Config:\n" << ultra_config.DebugString();
  // 探头安装坐标
  ultrasonic_detection::proto::UltrasonicCoordinate ultra_coordinate;
  // 左侧2个探头在整车坐标系下的位置
  coordinate_map_[1] =
      Point3D(ultra_coordinate.fsl_x(), ultra_coordinate.fsl_y(),
              ultra_coordinate.fsl_angle() * M_PI / 180);
  coordinate_map_[7] =
      Point3D(ultra_coordinate.rsr_x(), ultra_coordinate.rsr_y(),
              ultra_coordinate.rsr_angle() * M_PI / 180);
  // 前4个探头在整车坐标系下的位置
  coordinate_map_[2] =
      Point3D(ultra_coordinate.fol_x(), ultra_coordinate.fol_y(),
              ultra_coordinate.fol_angle() * M_PI / 180);
  coordinate_map_[3] =
      Point3D(ultra_coordinate.fcl_x(), ultra_coordinate.fcl_y(),
              ultra_coordinate.fcl_angle() * M_PI / 180);
  coordinate_map_[4] =
      Point3D(ultra_coordinate.fcr_x(), ultra_coordinate.fcr_y(),
              ultra_coordinate.fcr_angle() * M_PI / 180);
  coordinate_map_[5] =
      Point3D(ultra_coordinate.for_x(), ultra_coordinate.for_y(),
              ultra_coordinate.for_angle() * M_PI / 180);
  // 后4个探头在整车坐标系下的位置
  coordinate_map_[11] =
      Point3D(ultra_coordinate.rol_x(), ultra_coordinate.rol_y(),
              ultra_coordinate.rol_angle() * M_PI / 180);
  coordinate_map_[10] =
      Point3D(ultra_coordinate.rcl_x(), ultra_coordinate.rcl_y(),
              ultra_coordinate.rcl_angle() * M_PI / 180);
  coordinate_map_[9] =
      Point3D(ultra_coordinate.rcr_x(), ultra_coordinate.rcr_y(),
              ultra_coordinate.rcr_angle() * M_PI / 180);
  coordinate_map_[8] =
      Point3D(ultra_coordinate.ror_x(), ultra_coordinate.ror_y(),
              ultra_coordinate.ror_angle() * M_PI / 180);
  // 右侧2个探头在整车坐标系下的位置
  coordinate_map_[6] =
      Point3D(ultra_coordinate.fsr_x(), ultra_coordinate.fsr_y(),
              ultra_coordinate.fsr_angle() * M_PI / 180);
  coordinate_map_[12] =
      Point3D(ultra_coordinate.rsl_x(), ultra_coordinate.rsl_y(),
              ultra_coordinate.rsl_angle() * M_PI / 180);

  // 使用三角测距
  use_triangle_measured_ = ultra_config.use_triangle_measured();
  // Line Fit
  line_fit_params_.min_fit_num = ultra_config.line_fit_config().min_fit_num();
  line_fit_params_.min_cluster_size =
      ultra_config.line_fit_config().min_cluster_size();
  line_fit_params_.cluster_dist_threshold =
      ultra_config.line_fit_config().cluster_dist_threshold();
  line_fit_params_.merge_angle_threshold =
      ultra_config.line_fit_config().merge_angle_threshold();
  line_fit_params_.merge_dist_threshold =
      ultra_config.line_fit_config().merge_dist_threshold();
  // Parking Spot
  parking_spot_params_.grow_line_dist =
      ultra_config.parking_spot_config().grow_line_dist();
  parking_spot_params_.grow_line_angle =
      ultra_config.parking_spot_config().grow_line_angle();
  parking_spot_params_.min_line_length =
      ultra_config.parking_spot_config().min_line_length();
  parking_spot_params_.offset_threshold =
      ultra_config.parking_spot_config().offset_threshold();
  parking_spot_params_.angle_threshold =
      ultra_config.parking_spot_config().angle_threshold();
  // Curb
  curb_params_.angle_threshold = ultra_config.curb_config().angle_threshold();
  curb_params_.length_threshold = ultra_config.curb_config().length_threshold();

  // 空间车位识别
  parking_spot_detect_ptr_ = std::make_shared<ParkingSpotDetection>();
  // init ultrasonic writer channel
  ultrasonic_writer_ =
      node_->CreateWriter<UltrasonicList>("perception/ultrasonic/");
  target_spot_writer_ = node_->CreateWriter<TargetParkingSpotInfo>(
      "perception/ultrasonic/parkingSpot/");
  hmi_reader_ = node_->CreateReader<HMI>("hmi/");

  // 超声波雷达can解析器
  uss_interface_ptr_ = std::make_shared<USSInterface>();
  point_writer_ = node_->CreateWriter<uss::common_msgs::Point>("point");
  output_ = std::ofstream("ultrasonic_detection/position.txt");
  return true;
}

bool UltrasonicComponent::Proc(
    const std::shared_ptr<CanDataList> &can_data_list,
    const std::shared_ptr<InsLocation> &location) {
  // 解析CAN数据包
  std::string string;
  std::vector<std::string> strings;
  if (!can_data_list->SerializeToString(&string)) {
    AERROR << "SerializeToString failed";
    return false;
  }
  strings.emplace_back(string);
  // 获取echo data
  auto echo_list = uss_interface_ptr_->update(strings);

  // 提取每个探头的测距信息
  std::vector<int> distances(coordinate_map_.size() + 1, 0);
  // 根据探头配置选择测距方式，实现探头定位
  std::map<int, Point2D> pos_map;  //  车身坐标系下的障碍物位置
  std::map<int, Point2D> global_pos_map;  //  全局坐标系下障碍物位置
  auto pose = std::make_shared<Pose>(location->pose());
  ParkingSpot fsr_parking_spots;

  for (const auto &echo : echo_list.echo()) {
    int sensor_id = echo.sensor_id();
    echo_map_[sensor_id] = echo;
    auto distance = echo.distance();
    distances[sensor_id] = echo.distance();

    // for (int sensor_id = 1; sensor_id <= 12; ++sensor_id) {
    Point2D position;
    Point2D global_pos;
    // 直接测距定位
    // 根据探头ID和测距信息创建探头对象，包含了侧探头
    if (sensor_id != 6) continue;
    auto ultra_ptr =
        std::make_shared<Ultrasonic>(coordinate_map_[sensor_id], distance);
    // 直接测距定位
    ultra_ptr->DirectMeasuredPositionCalculate();
    position = ultra_ptr->GetPosition();
    pos_map[sensor_id] = position;
    status_map_[sensor_id] = ultra_ptr->GetStatus();
    // 转换到全局坐标系
    ultra_ptr->GlobalDirectPositionCalculate(pose);
    global_pos = ultra_ptr->GetGlobalPosition();
    global_pos_map[sensor_id] = global_pos;
    if (sensor_id == 6) {
      output_ << global_pos.x << " " << global_pos.y << "\n";
      auto point_msg = std::make_shared<uss::common_msgs::Point>();
      point_msg->set_x(global_pos.x);
      point_msg->set_y(global_pos.y);
      point_writer_->Write(point_msg);
      parking_spot_detect_ptr_->ParkingSpotSearch(
          global_pos, pose, line_fit_params_, parking_spot_params_,
          curb_params_, fsr_parking_spots);
      if (fsr_parking_spots.has_spot_type()) {
        spots_[spot_id_] = fsr_parking_spots;
        spot_id_++;
      }
    }
    // 三角测距定位目标,更新直接测距计算的位置
    if (use_triangle_measured_) {
      std::shared_ptr<UltrasonicDetection> ultra_detec_ptr;
      // sensor_id:2-5,8-11
      if (sensor_id == 2 || sensor_id == 3 || sensor_id == 4) {
        auto ul = Ultrasonic(coordinate_map_[sensor_id], distances[sensor_id]);
        auto ur =
            Ultrasonic(coordinate_map_[sensor_id + 1], distances[sensor_id]);
        ultra_detec_ptr = std::make_shared<UltrasonicDetection>(ul, ur);

      } else if (sensor_id == 8 || sensor_id == 9 || sensor_id == 10) {
        auto ul =
            Ultrasonic(coordinate_map_[sensor_id + 1], distances[sensor_id]);
        auto ur = Ultrasonic(coordinate_map_[sensor_id], distances[sensor_id]);
        ultra_detec_ptr = std::make_shared<UltrasonicDetection>(ul, ur);
      }
      // 三角定位
      ultra_detec_ptr->TriangleMeasuredPositionCalculate();
      position = ultra_detec_ptr->GetPosition();
      pos_map[sensor_id] = position;
      triangle_flag_[sensor_id] = true;
      status_map_[sensor_id] = ultra_detec_ptr->GetStatus();
      // 转换到全局坐标系
      ultra_detec_ptr->GlobalTrianglePositionCalculate(pose);
      global_pos = ultra_ptr->GetGlobalPosition();
      global_pos_map[sensor_id] = global_pos;
      if (sensor_id == 5) {
        pos_map[sensor_id] = pos_map[sensor_id - 1];
        triangle_flag_[sensor_id] = true;
        status_map_[sensor_id] = status_map_[sensor_id - 1];
        global_pos_map[sensor_id] = global_pos_map[sensor_id - 1];
      }
      if (sensor_id == 8) {
        pos_map[sensor_id] = pos_map[sensor_id + 1];
        triangle_flag_[sensor_id] = true;
        status_map_[sensor_id] = status_map_[sensor_id + 1];
        global_pos_map[sensor_id] = global_pos_map[sensor_id + 1];
      }
    }
  }
  // 空间车位识别
  /**
   * 使用FSL和FSR进行空间车位识别
   * 存储10帧障碍物位置后，拟合边界线段，对满足要求的线段对进行生长再判断
   */
  // 车位
  // ParkingSpot fsl_parking_spots;
  // ParkingSpot fsr_parking_spots;
  // // TODO：Async？
  // for (const auto &pair : global_pos_map) {
  //   if (pair.first == 1) {
  //     parking_spot_detect_ptr_->ParkingSpotSearch(
  //         pair.second, pose, line_fit_params_, parking_spot_params_,
  //         curb_params_, fsl_parking_spots);
  //     if (fsl_parking_spots.has_spot_type()) {
  //       spots_[spot_id_] = fsl_parking_spots;
  //       spot_id_++;
  //     }
  //   }
  //   if (pair.first == 6) {
  //     parking_spot_detect_ptr_->ParkingSpotSearch(
  //         pair.second, pose, line_fit_params_, parking_spot_params_,
  //         curb_params_, fsr_parking_spots);
  //     if (fsr_parking_spots.has_spot_type()) {
  //       spots_[spot_id_] = fsr_parking_spots;
  //       spot_id_++;
  //     }
  //   }
  // }
  // 未找到空间车位
  if (spots_.empty()) {
    AINFO << "No parking Spots";
    return false;
  }

  // 组织数据
  auto ultrasonic_list = std::make_shared<UltrasonicList>();
  // 测距定位信息
  FillUltraObject(pos_map, global_pos_map, ultrasonic_list);
  auto now = Clock::NowInSeconds();
  ultrasonic_list->set_timestamp(now);
  // TODO:信号是否有延迟
  if (now - echo_list.timestamp() < 1) {
    ultrasonic_list->set_error_code(uss::common_msgs::ErrorCode::OK);
  } else {
    ultrasonic_list->set_error_code(
        uss::common_msgs::ErrorCode::PERCEPTION_ERROR);
  }
  // 空间车位信息
  FillParkingSpots(spots_, ultrasonic_list);
  // 发布空间车位
  AINFO << ultrasonic_list->parking_spots_info().DebugString();
  ultrasonic_writer_->Write(ultrasonic_list);
  // 目标车位：从GUI界面获取用户选择的车位信息和车头朝向
  hmi_reader_->Observe();
  auto hmi_msg = hmi_reader_->GetLatestObserved();
  if (hmi_msg == nullptr) {
    AINFO << "No message in 'hmi/' channel";
  } else {
    ADEBUG << "HMI: " << hmi_msg->DebugString();
  }
  auto spot = spots_[hmi_msg->spot_id()];
  bool parking_inwards = hmi_msg->parking_inwards();
  // 发布目标车位
  if (spot.has_spot_type()) {
    auto target_spot = std::make_shared<TargetParkingSpotInfo>();
    PublishTargetParkingSpot(spot, parking_inwards, hmi_msg->spot_id(),
                             target_spot);
    target_spot_writer_->Write(target_spot);
  }
  return true;
}

void UltrasonicComponent::FillUltraObject(
    const std::map<int, Point2D> &points_map,
    const std::map<int, Point2D> &global_pos_map,
    std::shared_ptr<UltrasonicList> &out_msg) {
  auto tmp_map1 = points_map;
  auto tmp_map2 = global_pos_map;
  for (size_t sensor_id = 1; sensor_id <= points_map.size(); ++sensor_id) {
    // 添加障碍物
    auto ul_obj = out_msg->add_ul_objs();
    // 探头位置
    auto it = kUltrasonicOrientation2NameMap.find(
        static_cast<UltrasonicOrientation>(sensor_id));
    ul_obj->set_orientation(it->second);
    // 障碍物点
    ul_obj->mutable_position()->set_x(tmp_map1[sensor_id].x);
    ul_obj->mutable_position()->set_y(tmp_map1[sensor_id].y);
    // 全局坐标系下障碍物点
    ul_obj->mutable_position_global()->set_x(tmp_map2[sensor_id].x);
    ul_obj->mutable_position_global()->set_y(tmp_map2[sensor_id].y);
    // 最近距离
    ul_obj->set_distance(echo_map_[sensor_id].distance());
    // 波宽
    ul_obj->set_width(echo_map_[sensor_id].width());
    // 置信度
    ul_obj->set_confidence(echo_map_[sensor_id].confidence());
    // 检测状态
    ul_obj->set_status(status_map_[sensor_id]);
    // 是否三角测距定位
    ul_obj->set_use_triangle(triangle_flag_[sensor_id]);
  }
}

void UltrasonicComponent::FillParkingSpots(
    const std::map<size_t, ParkingSpot> &spots,
    std::shared_ptr<UltrasonicList> &out_msg) {
  auto parking_spots_info = out_msg->mutable_parking_spots_info();
  for (const auto &pair : spots) {
    auto spot = parking_spots_info->add_parking_spots();
    spot->CopyFrom(pair.second);
  }
  // 车位数量
  parking_spots_info->set_spots_cnt(spot_id_);
}

void UltrasonicComponent::PublishTargetParkingSpot(
    const ParkingSpot &spot, bool parking_inwards, size_t spot_id,
    std::shared_ptr<TargetParkingSpotInfo> &out_msg) {
  auto target_spot = out_msg->mutable_target_parking_spot();
  target_spot->CopyFrom(spot);

  // 车头朝向
  out_msg->mutable_hmi()->set_parking_inwards(parking_inwards);
  // 车位编号
  out_msg->mutable_hmi()->set_spot_id(spot_id);
}
