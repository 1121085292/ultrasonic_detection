#include "ultrasonic_perception_component.h"
#include "cyber/time/clock.h"

using Clock = apollo::cyber::Clock;

bool UltrasonicComponent::Init()
{
  // 导入探头安装坐标文件
  UltrasonicCoordinate ultra_coordinate;
  if(!apollo::cyber::common::GetProtoFromFile(config_file_path_, &ultra_coordinate)){
    AERROR << "Read config failed: " << config_file_path_;
    return false;
  }
  AINFO << "Ultrasonic Coordinate:\n" << ultra_coordinate.DebugString();
  // 左侧2个探头在整车坐标系下的位置
  coordinate_map_[0] = Point(ultra_coordinate.fsl_x(), ultra_coordinate.fsl_y(), ultra_coordinate.fsl_angle() * M_PI / 180);
  coordinate_map_[11] = Point(ultra_coordinate.rsr_x(), ultra_coordinate.rsr_y(), ultra_coordinate.rsr_angle() * M_PI / 180);
  // 前4个探头在整车坐标系下的位置
  coordinate_map_[1] = Point(ultra_coordinate.fol_x(), ultra_coordinate.fol_y(), ultra_coordinate.fol_angle() * M_PI / 180);
  coordinate_map_[2] = Point(ultra_coordinate.fcl_x(), ultra_coordinate.fcl_y(), ultra_coordinate.fcl_angle() * M_PI / 180);
  coordinate_map_[3] = Point(ultra_coordinate.fcr_x(), ultra_coordinate.fcr_y(), ultra_coordinate.fcr_angle() * M_PI / 180);
  coordinate_map_[4] = Point(ultra_coordinate.for_x(), ultra_coordinate.for_y(), ultra_coordinate.for_angle() * M_PI / 180);
  // 后4个探头在整车坐标系下的位置
  coordinate_map_[7] = Point(ultra_coordinate.rol_x(), ultra_coordinate.rol_y(), ultra_coordinate.rol_angle() * M_PI / 180);
  coordinate_map_[8] = Point(ultra_coordinate.rcl_x(), ultra_coordinate.rcl_y(), ultra_coordinate.rcl_angle() * M_PI / 180);
  coordinate_map_[9] = Point(ultra_coordinate.rcr_x(), ultra_coordinate.rcr_y(), ultra_coordinate.rcr_angle() * M_PI / 180);
  coordinate_map_[10] = Point(ultra_coordinate.ror_x(), ultra_coordinate.ror_y(), ultra_coordinate.ror_angle() * M_PI / 180);
  // 右侧2个探头在整车坐标系下的位置
  coordinate_map_[5] = Point(ultra_coordinate.fsr_x(), ultra_coordinate.fsr_y(), ultra_coordinate.fsr_angle() * M_PI / 180);
  coordinate_map_[6] = Point(ultra_coordinate.rsl_x(), ultra_coordinate.rsl_y(), ultra_coordinate.rsl_angle() * M_PI / 180);

  // 使用三角测距
  use_triangle_measured_ = ultra_coordinate.use_triangle_measured();
  // 空间车位识别
  parking_spot_detect_ptr_ = std::make_shared<ParkingSpotDetection>();
  // init ultrasonic writer channel
  ultrasonic_writer_ = node_->CreateWriter<UltrasonicList>("perception/ultrasonic/obstalce");
  parking_spot_writer_ = node_->CreateWriter<TargetParkingSpotInfo>("perception/ultrasonic/parking_spot");
  return true;
}

bool UltrasonicComponent::Proc(
    const std::shared_ptr<EchoList>& echo_list,
    const std::shared_ptr<InsLocation>& location){
  // 提取每个探头的测距信息
  std::map<int, std::vector<int>> dis_map;
  for(const auto& echo : echo_list->echo_list()){
    int sensor_id = echo.sensor_id();
    echo_map_[sensor_id] = echo;
    // 遍历添加echo中的距离到容器
    std::vector<int> distances;
    // 如果某个探头故障，默认测距为最大量程
    if(echo.error_code() != common_msgs::error_code::ErrorCode::OK){
      AERROR << "No." << sensor_id << " Ultrasonic Error";
      distances.emplace_back(6000);
    } else {
      for(const auto& distance : echo.distances()){
        distances.emplace_back(distance);
      }
    }
    dis_map[sensor_id] = distances;
  }

  // 根据探头配置选择测距方式，实现探头定位
  std::map<int, Point2D> pos_map;           //  车身坐标系下的障碍物位置
  std::map<int, Point2D> global_pos_map;    //  全局坐标系下障碍物位置
  auto pose = std::make_shared<Pose>(location->pose());
  for(int sensor_id = 0; sensor_id < 12; ++sensor_id){
    Point2D position;
    Point2D global_pos;
    // 直接测距定位
    // 根据探头ID和测距信息创建探头对象，包含了侧探头
    auto ultra_ptr = std::make_shared<Ultrasonic>(coordinate_map_[sensor_id], dis_map[sensor_id][0]);
    // 直接测距定位
    ultra_ptr->DirectMeasuredPositionCalculate();
    position = ultra_ptr->GetPosition();
    pos_map[sensor_id] = position;
    status_map_[sensor_id] = ultra_ptr->GetStatus();
    // 转换到全局坐标系
    ultra_ptr->GlobalDirectPositionCalculate(pose);
    global_pos = ultra_ptr->GetGlobalPosition();
    global_pos_map[sensor_id] = global_pos;
    // 三角测距定位目标,更新直接测距计算的位置
    if(use_triangle_measured_){
      // sensor_id:1-4,7-10
      if(sensor_id == 1 || sensor_id == 2 || sensor_id == 3 ||
          sensor_id == 7 || sensor_id == 8 || sensor_id == 9){
        auto ul = Ultrasonic(coordinate_map_[sensor_id], dis_map[sensor_id][0]);
        auto ur = Ultrasonic(coordinate_map_[sensor_id + 1], dis_map[sensor_id][0]);
        auto ultra_detec_ptr = std::make_shared<UltrasonicDetection>(ul, ur);
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
      } else if(sensor_id == 4 || sensor_id == 10){
        pos_map[sensor_id] = pos_map[sensor_id - 1];
        triangle_flag_[sensor_id] = true;
        status_map_[sensor_id] = status_map_[sensor_id - 1];
        global_pos_map[sensor_id] = global_pos_map[sensor_id - 1];
      }
    }
  }
  // 空间车位识别
  /*
  * 使用FSL和FSR进行空间车位识别
  * 存储20帧障碍物位置，数据滤波后进行线段拟合，通过横向和纵向距离进行阈值判断，识别潜在车位
  */
  // 车位
  std::vector<Point2D> parking_vertices;
  for(const auto& pair : global_pos_map){
    if(pair.first == 0){
      parking_spot_detect_ptr_->ParkingSpotSearch(pair.second, pose, parking_vertices);
    }
    if(pair.first == 5){
      parking_spot_detect_ptr_->ParkingSpotSearch(pair.second, pose, parking_vertices);
    }
  }

  // 组织数据
  // 测距定位信息
  auto ultrasonic_list = std::make_shared<UltrasonicList>();
  FillUltraObject(pos_map, global_pos_map, ultrasonic_list);
  auto now = Clock::NowInSeconds();
  ultrasonic_list->set_timestamp(now);
  // TODO:信号是否有延迟
  if(now - echo_list->timestamp() < 1){
    ultrasonic_list->set_error_code(common_msgs::error_code::ErrorCode::OK);
  } else {
    ultrasonic_list->set_error_code(common_msgs::error_code::ErrorCode::PERCEPTION_ERROR);
  }
  // 空间车位信息
  auto parking_spot = std::make_shared<TargetParkingSpotInfo>();
  FillParkingSpot(parking_vertices, parking_spot);
  // 发布
  ultrasonic_writer_->Write(ultrasonic_list);
  parking_spot_writer_->Write(parking_spot);
  return true;
}

void UltrasonicComponent::FillUltraObject(
    const std::map<int, Point> &points_map,
    const std::map<int, Point> &global_pos_map,
    std::shared_ptr<UltrasonicList>& out_msg){
  
  auto tmp_map1 = points_map;
  auto tmp_map2 = global_pos_map;
  for(size_t sensor_id = 0; sensor_id < points_map.size(); ++sensor_id){
    // 添加障碍物
    auto ul_obj = out_msg->add_ul_obj();
    // 探头位置
    auto it = kUltrasonicOrientation2NameMap.find(static_cast<UltrasonicOrientation>(sensor_id));
    ul_obj->set_orientation(it->second);
    // 障碍物点
    ul_obj->mutable_position()->set_x(tmp_map1[sensor_id].x);
    ul_obj->mutable_position()->set_y(tmp_map1[sensor_id].y);
    // 全局坐标系下障碍物点
    ul_obj->mutable_position_global()->set_x(tmp_map2[sensor_id].x);
    ul_obj->mutable_position_global()->set_y(tmp_map2[sensor_id].y);
    // 最近距离
    ul_obj->set_distance(echo_map_[sensor_id].distances(0));
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

void UltrasonicComponent::FillParkingSpot(
    const std::vector<Point2D>& parking_vertices,
    std::shared_ptr<TargetParkingSpotInfo> &out_msg)
{
  std::vector<Point2D> points = parking_vertices;
  auto target_parking_vertices = out_msg->mutable_target_parking_vertices();
  // 0号点
  target_parking_vertices.set_spot_right_top_x(points[0].x);
  target_parking_vertices.set_spot_right_top_y(points[0].y);
  // 1号点
  target_parking_vertices.set_spot_left_top_x(points[1].x);
  target_parking_vertices.set_spot_left_top_y(points[1].y);
  // 2号点
  target_parking_vertices.set_spot_left_down_x(points[2].x);
  target_parking_vertices.set_spot_left_down_y(points[2].y);
  // 3号点
  target_parking_vertices.set_spot_right_down_x(points[3].x);
  target_parking_vertices.set_spot_right_down_y(points[3].y);
}
