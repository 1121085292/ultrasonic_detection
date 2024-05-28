/**
 * @file ultrasonic.h
 * @brief
 * @author tangpeng
 * @version 1.0
 * @date 2024-05-28
 * @copyright Copyright (c) 2024 tangpeng. All rights reserved.
 */
#pragma once

#include "ultrasonic_detection/base/params.h"
#include "ultrasonic_detection/base/point.h"

#include "ultrasonic_detection/common_msgs/InsLocation.pb.h"
#include "ultrasonic_detection/common_msgs/ultrasonic.pb.h"

using common_msgs::InsLocation::InsLocation;
using common_msgs::ultrasonic::Status;
using Pose = common_msgs::InsLocation::InsLocation_Pose;

class Ultrasonic {
 public:
  Ultrasonic() {}
  Ultrasonic(Point3D coordinate, int distance)
      : coordinate_(coordinate), meas_dis_(distance) {}
  Ultrasonic(Point3D coordinate, Point2D point)
      : coordinate_(coordinate), point_global_(point) {}
  ~Ultrasonic(){};

  // 获取探头测量距离
  int GetMeasDistance() const { return meas_dis_; }

  // 获取探头安装位置
  Point3D GetUltraCoordinate() const { return coordinate_; }

  // 获取障碍物位置
  Point2D GetPosition() const { return point_; }
  // 获取全局坐标系下障碍物位置
  Point2D GetGlobalPosition() const { return point_global_; }

  // 更新检测状态
  void SetStatus(const Status& ultra_status) { ultra_status_ = ultra_status; }
  // 获取检测状态
  Status GetStatus() const { return ultra_status_; }

  // 直接测距计算障碍物位置
  void DirectMeasuredPositionCalculate();

  // 全局坐标系下障碍物位置
  void GlobalDirectPositionCalculate(const std::shared_ptr<Pose>& pose);

 private:
  // 探头安装位置
  Point3D coordinate_;
  // 探头测距信息/mm
  int meas_dis_;
  // 障碍物位置
  Point2D point_;
  // 全局坐标系下障碍物位置
  Point2D point_global_;

  UltrasonicParams ultra_params_;
  Status ultra_status_;
};