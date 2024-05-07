#pragma once
#include <map>

#include "cyber/cyber.h"
#include "ultrasonic_detection/common/line_segment.h"
#include "ultrasonic_detection/common_msgs/InsLocation.pb.h"

using Pose = common_msgs::InsLocation::InsLocation_Pose;

enum ParkingSpaceType {
  VERTICAL_PLOT = 0,
  PARALLEL_PARKING = 1
};

class ParkingSpotDetection {
  public:
    // 空间车位搜索
    void ParkingSpotSearch(
        const Point2D& point, const std::shared_ptr<Pose> &pose,
        const LineFitParams& line_fit_params,
        const ParkingSpotParams& parking_spot_params,
        const CurbParams& curb_params,
        std::vector<Point2D>& parking_vertices);
    // 车位类别
    ParkingSpaceType GetParkingSpaceType() { return parking_space_type_; }
    // 车位位姿计算
    double CalculateParkingSpotAngle(const std::shared_ptr<Pose> &pose);
        
  private:
    // 潜在车位搜索
    bool FindPotentialParkingSpots(
        const std::vector<LineSegment> &fit_lines,
        const std::shared_ptr<Pose> &pose,
        const ParkingSpotParams& parking_spot_params,
        const CurbParams& curb_params,
        std::map<int, std::vector<LineSegment>>& line_segments);

    // 潜在车位再判断
    bool ReEvaluateParallelParkingSpot(
        const std::pair<int, std::vector<LineSegment>> &line_segment_pair,
        const ParkingSpotParams& parking_spot_params,
        const std::shared_ptr<Pose> &pose);

    // 车位约束条件判断
    bool CheckParkingSpotConstraints(
      const std::shared_ptr<Pose>& pose,
      const ParkingSpotParams& parking_spot_params);

    // 车位角点计算
    void CalculateParkingVertices(double angle, std::vector<Point2D>& parking_vertices);
    // 生长线段
    double GrowLineSegment(
        const LineSegment& line, int key, bool flag,
        const ParkingSpotParams& parking_spot_params,
        const std::shared_ptr<Pose>& pose);

    // 判断是否为路牙特征线段
    bool IsCurbLine(
        const LineSegment& line,
        const CurbParams& curb_params,
        const std::shared_ptr<Pose>& pose);

    // 匹配线段对
    std::vector<LineSegment> parking_spot_;

    // 障碍物点集
    std::vector<Point2D> points_;

    std::shared_ptr<LineSegment> line_segment_ptr_;
    std::vector<LineSegment> fit_lines_;
    std::vector<LineSegment> curb_lines_;
    ParkingSpaceType parking_space_type_;
};