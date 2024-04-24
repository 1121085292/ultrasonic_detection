#pragma once
#include <vector>
#include <algorithm>
#include <cmath>

#include "ultrasonic_detection/ultrasonic/ultrasonic.h"

class UltrasonicDetection {
  public:
    UltrasonicDetection(){}
    UltrasonicDetection(const Ultrasonic& ul, const Ultrasonic& ur)
      : ul_(ul), ur_(ur) {}
    ~UltrasonicDetection(){};

    // 获取障碍物位置
    Point2D GetPosition() const { return point_; }
    // 获取全局坐标系下障碍物位置
    Point2D GetGlobalPosition() const { return point_global_; }
    // 获取检测状态
    Status GetStatus() const { return ul_.GetStatus(); }

    // 通过三角测距计算位置
    void TriangleMeasuredPositionCalculate();

    // 全局坐标系下障碍物位置
    void GlobalTrianglePositionCalculate(const std::shared_ptr<Pose>& pose);
  private:
    // 探头的测距信息
    std::vector<int> distances_;
    // 三角测距的两个探头
    Ultrasonic ul_;
    Ultrasonic ur_;
    // 障碍物位置
    Point2D point_;
    // 全局坐标系下障碍物位置
    Point2D point_global_;
};