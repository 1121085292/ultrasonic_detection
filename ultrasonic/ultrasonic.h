#pragma once
#include <vector>
#include <cmath>

#include "ultrasonic_detection/proto/ultrasonic_coordinate.pb.h"
#include "ultrasonic_detection/common_msgs/ultrasonic.pb.h"
#include "ultrasonic_detection/common_msgs/InsLocation.pb.h"
#include "ultrasonic_detection/min_filter/min_filter.h"
#include "ultrasonic_detection/base/params.h"

using ultrasonic_detection::proto::UltrasonicCoordinate;
using common_msgs::ultrasonic::Status;
using common_msgs::InsLocation::InsLocation;
using Pose = common_msgs::InsLocation::InsLocation_Pose;

struct Point{
  double x, y, angle;
  Point(){}

  Point(double x_val, double y_val, double angle_val)
   : x(x_val), y(y_val), angle(angle_val){}
  
  Point operator-(const Point& other) const {
    return Point(x - other.x, y - other.y, 0.0);
  }
};

class Ultrasonic {
  public:
    Ultrasonic(){}
    Ultrasonic(Point coordinate, int distance)
     : coordinate_(coordinate), meas_dis_(distance) {}
    ~Ultrasonic(){};

    // 获取探头测量距离
    int GetMeasDistance() const { return meas_dis_; }

    // 获取探头安装位置
    Point GetUltraCoordinate() const { return coordinate_; }

    // 获取障碍物位置
    Point GetPosition() const { return point_; }
    // 获取全局坐标系下障碍物位置
    Point GetGlobalPosition() const { return point_global_; }

    // 更新检测状态
    void SetStatus(const Status& ultra_status){
      ultra_status_ = ultra_status;
    }
    // 获取检测状态
    Status GetStatus() const { return ultra_status_; }

    // 直接测距计算障碍物位置
    void DirectMeasuredPositionCalculate();

    // 全局坐标系下障碍物位置
    void GlobalDirectPositionCalculate(const std::shared_ptr<Pose>& pose);

  private:
    // 探头安装位置
    Point coordinate_;
    // 探头测距信息/mm
    int meas_dis_;
    // 障碍物位置
    Point point_;
    // 全局坐标系下障碍物位置
    Point point_global_;

    UltrasonicParams ultra_params_;
    Status ultra_status_;
};