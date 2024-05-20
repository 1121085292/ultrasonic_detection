#pragma once

#include "cyber/cyber.h"
#include "ultrasonic_detection/common_msgs/ultrasonic.pb.h"
#include "ultrasonic_detection/common_msgs/InsLocation.pb.h"
#include "ultrasonic_detection/ui/visualizer.h"

const int car_width = 1880;
const int car_length = 4800;

using apollo::cyber::Component;
using apollo::cyber::Writer;
using common_msgs::parking::proto::ParkingSpotsInfo;
using common_msgs::parking::proto::ParkingSpotType;
using common_msgs::ultrasonic::UltrasonicList;
using common_msgs::InsLocation::InsLocation;
using Pose = common_msgs::InsLocation::InsLocation_Pose;

class TargetSpotComponent : public Component<UltrasonicList, InsLocation> {
  public:
  ~TargetSpotComponent();

    bool Init() override;
    bool Proc(const std::shared_ptr<UltrasonicList>& ultra,
              const std::shared_ptr<InsLocation>& location) override;

  private:
    QApplication* app_;
    QGraphicsScene* scene_;
    ParkingSpotVisualizer* view_;

    QVector<QRect> rectangles_;
    ParkingSpotType spot_type_;

    std::shared_ptr<Writer<TargetParkingSpotInfo>> target_spot_writer_;
};
CYBER_REGISTER_COMPONENT(TargetSpotComponent);