#pragma once
#include <atomic>
#include <future>

#include "car/byd/uss_interface.h"
#include "opendbc/can/dbc_out/PDC_USS.h"
#include "ultrasonic_detection/base/ultrasonic_orientation.h"
#include "ultrasonic_detection/ultrasonic/parking_spot_detection.h"
#include "ultrasonic_detection/ultrasonic/ultrasonic_detection.h"

#include "common_msgs/car/can_data.pb.h"
#include "ultrasonic_detection/common_msgs/hmi.pb.h"
#include "ultrasonic_detection/common_msgs/parking_spots.pb.h"
#include "ultrasonic_detection/common_msgs/ultrasonic.pb.h"
#include "ultrasonic_detection/proto/ultrasonic_conf.pb.h"

#include "cyber/cyber.h"

dbc_init(PDC_USS);

using apollo::cyber::Component;
using common_msgs::can_data::CanDataList;
using ultrasonic_detection::proto::UltrasonicConfig;
using uss::common_msgs::HMI;
using uss::common_msgs::TargetParkingSpotInfo;
using uss::common_msgs::UltrasonicList;

class UltrasonicComponent : public Component<CanDataList, InsLocation> {
 public:
  bool Init() override;

  bool Proc(const std::shared_ptr<CanDataList>& can_data_list,
            const std::shared_ptr<InsLocation>& location) override;

 private:
  void FillUltraObject(const std::map<int, Point2D>& points_map,
                       const std::map<int, Point2D>& global_pos_map,
                       std::shared_ptr<UltrasonicList>& out_msg);
  void FillParkingSpots(const std::map<size_t, ParkingSpot>& spots,
                        std::shared_ptr<UltrasonicList>& out_msg);

  void PublishTargetParkingSpot(
      const ParkingSpot& spot, bool parking_inwards, size_t spot_id,
      std::shared_ptr<TargetParkingSpotInfo>& out_msg);
  std::map<int, Echo> echo_map_;  // 探头id ：echo

  bool use_triangle_measured_ = false;
  std::map<int, bool> triangle_flag_;

  std::map<int, Status> status_map_;

  // 发布空间车位
  std::shared_ptr<apollo::cyber::Writer<UltrasonicList>> ultrasonic_writer_ =
      nullptr;
  // 选择的目标车位和车头朝向
  std::shared_ptr<apollo::cyber::Reader<HMI>> hmi_reader_ = nullptr;
  // 发布目标车位
  std::shared_ptr<apollo::cyber::Writer<TargetParkingSpotInfo>>
      target_spot_writer_ = nullptr;
  // 探头相对车身坐标系安装位置
  std::map<int, Point3D> coordinate_map_;
  // 线段拟合参数
  LineFitParams line_fit_params_;
  // 空间车位识别参数
  ParkingSpotParams parking_spot_params_;
  // 路牙识别参数
  CurbParams curb_params_;
  // 空间车位识别
  std::shared_ptr<ParkingSpotDetection> parking_spot_detect_ptr_;
  // 空间车位
  std::map<size_t, ParkingSpot> spots_;
  // 车位id
  size_t spot_id_ = 0;
  // 解析超声波雷达can数据
  std::shared_ptr<USSInterface> uss_interface_ptr_ = nullptr;
};
CYBER_REGISTER_COMPONENT(UltrasonicComponent);