#pragma once

#include "cyber/cyber.h"
#include "ultrasonic_detection/common_msgs/echo_list.pb.h"
#include "ultrasonic_detection/common_msgs/ultrasonic.pb.h"
#include "ultrasonic_detection/ultrasonic/ultrasonic_detection.h"
#include "ultrasonic_detection/base/ultrasonic_orientation.h"

using common_msgs::echo_list::Echo;
using common_msgs::echo_list::EchoList;
using common_msgs::ultrasonic::UltrasonicList;

using apollo::cyber::Component;

class UltrasonicComponent : public Component<EchoList, InsLocation>{
  public:
    bool Init() override;
    
    bool Proc(const std::shared_ptr<EchoList>& echo_list,
              const std::shared_ptr<InsLocation>& location) override;

  private:
    void FillUltraObject(const std::map<int, Point> &points_map,
                        const std::map<int, Point> &global_pos_map,
                        std::shared_ptr<UltrasonicList>& out_msg);

    std::map<int, Echo> echo_map_;         // 探头id ：echo

    bool use_triangle_measured_ = false;
    std::map<int, bool> triangle_flag_;

    std::map<int, Status> status_map_;
    std::shared_ptr<apollo::cyber::Writer<UltrasonicList>> ultrasonic_writer_ = nullptr;

    // 探头相对车身坐标系安装位置
    std::map<int, Point> coordinate_map_;
};
CYBER_REGISTER_COMPONENT(UltrasonicComponent);