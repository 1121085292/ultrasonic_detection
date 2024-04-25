#pragma once
#include <map>

#include "cyber/cyber.h"
#include "ultrasonic_detection/common/line_segment.h"

enum ParkingSpaceType {
  VERTICAL_PLOT = 0,
  PARALLEL_PARKING = 1
};

class ParkingSpotDetection {
  public:
    // 空间车位搜索
    void ParkingSpotSearch(
        const Point2D& point,
        const Pose &pose);
    // 车位类别
    ParkingSpaceType GetParkingSpaceType() { return parking_space_type_; }
    // 车位位姿计算
    double CalculateParkingSpotAngle(
        const pair<LineSegment, LineSegment>& parking_spot,
        const vector<LineSegment>& curbs,
        const vector<LineSegment>& obstacles,
        double vehicleHeading);
        
  private:
    // 潜在车位搜索
    void FindPotentialParkingSpots(
        vector<LineSegment> &fit_lines, const Pose &pose,
        std::map<int, std::vector<LineSegment>>& line_segments);

    // 潜在车位再判断
    bool ReEvaluateParallelParkingSpot(
        std::map<int, std::vector<LineSegment>> &line_segment_pair,
        const Pose &pose);

    // 车位约束条件判断
    bool CheckParkingSpotConstraints(const pair<LineSegment, LineSegment>& parking_spot);

    // 生长线段
    double GrowLineSegment(
        LineSegment& line, int key,
        bool flag, const Pose& pose);
    // 计算两线段夹角
    double AngleBetweenLines(const LineSegment& line1, const LineSegment& line2);

    std::shared_ptr<LineSegment> line_segment_ptr_;
    std::map<int, LineSegment> fit_lines_map_;
    ParkingSpaceType parking_space_type_;
};