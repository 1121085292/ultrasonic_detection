#pragma once
#include <future>
#include <atomic>

#include "cyber/cyber.h"
#include "ultrasonic_detection/common_msgs/echo_list.pb.h"
#include "ultrasonic_detection/common_msgs/ultrasonic.pb.h"
#include "ultrasonic_detection/common_msgs/parking_perception.pb.h"
#include "ultrasonic_detection/common_msgs/hmi.pb.h"
#include "ultrasonic_detection/proto/ultrasonic_conf.pb.h"
#include "ultrasonic_detection/ultrasonic/ultrasonic_detection.h"
#include "ultrasonic_detection/ultrasonic/parking_spot_detection.h"
#include "ultrasonic_detection/base/ultrasonic_orientation.h"
#include "ultrasonic_detection/common/min_filter.h"

#define Minfilter 0

using apollo::cyber::Component;
using common_msgs::echo_list::Echo;
using common_msgs::echo_list::EchoList;
using common_msgs::ultrasonic::UltrasonicList;
using common_msgs::hmi::HMI;
using common_msgs::parking::proto::TargetParkingSpotInfo;
using ultrasonic_detection::proto::UltrasonicConfig;

class UltrasonicComponent : public Component<EchoList, InsLocation>{
  public:
    bool Init() override;
    
    bool Proc(const std::shared_ptr<EchoList>& echo_list,
              const std::shared_ptr<InsLocation>& location) override;

  private:
    void FillUltraObject(const std::map<int, Point2D> &points_map,
                        const std::map<int, Point2D> &global_pos_map,
                        std::shared_ptr<UltrasonicList>& out_msg);
    void FillParkingSpots(const std::map<size_t, ParkingSpot>& spots,
                          std::shared_ptr<UltrasonicList>& out_msg);

    void PublishTargetParkingSpot(const ParkingSpot& spot,
                                  bool parking_inwards, size_t spot_id,
                                  std::shared_ptr<TargetParkingSpotInfo>& out_msg);
    std::map<int, Echo> echo_map_;         // 探头id ：echo

    bool use_triangle_measured_ = false;
    std::map<int, bool> triangle_flag_;

    std::map<int, Status> status_map_;

    // 发布空间车位
    std::shared_ptr<apollo::cyber::Writer<UltrasonicList>> ultrasonic_writer_ = nullptr;
    // 选择的目标车位和车头朝向
    std::shared_ptr<apollo::cyber::Reader<HMI>> hmi_reader_ = nullptr;
    // 发布目标车位
    std::shared_ptr<apollo::cyber::Writer<TargetParkingSpotInfo>> target_spot_writer_ = nullptr;
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
};
CYBER_REGISTER_COMPONENT(UltrasonicComponent);