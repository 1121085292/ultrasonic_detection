#pragma once
#include <map>

#include "cyber/cyber.h"
#include "ultrasonic_detection/common/line_segment.h"
#include "ultrasonic_detection/common_msgs/InsLocation.pb.h"
#include "ultrasonic_detection/common_msgs/parking_perception.pb.h"

using Pose = common_msgs::InsLocation::InsLocation_Pose;
using common_msgs::parking::proto::ParkingSpotType;
using common_msgs::parking::proto::ParkingSpot;

class ParkingSpotDetection {
  public:
    // 空间车位搜索
    void ParkingSpotSearch(
        const Point2D& point, const std::shared_ptr<Pose> &pose,
        const LineFitParams& line_fit_params,
        const ParkingSpotParams& parking_spot_params,
        const CurbParams& curb_params,
        ParkingSpot& parking_spot);
    // 车位类别
    ParkingSpotType GetParkingSpotType() { return parking_spot_type_; }
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
    void CalculateParkingVertices(double angle, ParkingSpot& parking_spot);
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
    std::vector<LineSegment> parking_spot_pair_;

    // 障碍物点集
    std::vector<Point2D> points_;

    std::shared_ptr<LineSegment> line_segment_ptr_;
    std::vector<LineSegment> fit_lines_;
    std::vector<LineSegment> curb_lines_;
    ParkingSpotType parking_spot_type_;
};