#include <vector>
#include <cmath>
#include "line_segment.h"

using namespace std;

vector<pair<LineSegment, LineSegment>> findPotentialParkingSpots(const vector<LineSegment>& lineSegments, double parkingSpotWidthThreshold) {
    vector<pair<LineSegment, LineSegment>> potentialParkingSpots;

    for (size_t i = 0; i < lineSegments.size() - 1; ++i) {
        const LineSegment& line1 = lineSegments[i];
        const LineSegment& line2 = lineSegments[i + 1];

        // 判断线段长度是否足够
        if (line1.Length() < parkingSpotWidthThreshold || line2.Length() < parkingSpotWidthThreshold) {
            continue;
        }

        // 将线段投影到车辆行驶方向
        double angle = atan2(line1.p2.y - line1.p1.y, line1.p2.x - line1.p1.x);
        Point2D line1Proj1(line1.p1.x * cos(angle) - line1.p1.y * sin(angle), line1.p1.x * sin(angle) + line1.p1.y * cos(angle));
        Point2D line1Proj2(line1.p2.x * cos(angle) - line1.p2.y * sin(angle), line1.p2.x * sin(angle) + line1.p2.y * cos(angle));
        Point2D line2Proj1(line2.p1.x * cos(angle) - line2.p1.y * sin(angle), line2.p1.x * sin(angle) + line2.p1.y * cos(angle));
        Point2D line2Proj2(line2.p2.x * cos(angle) - line2.p2.y * sin(angle), line2.p2.x * sin(angle) + line2.p2.y * cos(angle));

        // 计算投影后的线段近端点距离
        double projDist = fabs(line1Proj2.x - line2Proj1.x);

        // 判断近端点距离是否大于车位宽度阈值
        if (projDist > parkingSpotWidthThreshold) {
            potentialParkingSpots.emplace_back(line1, line2);
        }
    }

    return potentialParkingSpots;
}

bool ReEvaluateParallelParkingSpot(const pair<LineSegment, LineSegment>& potentialSpot) {
    const LineSegment& line1 = potentialSpot.first;
    const LineSegment& line2 = potentialSpot.second;

    // 让线段向外生长
    LineSegment grownLine1 = GrowLineSegment(line1);
    LineSegment grownLine2 = GrowLineSegment(line2);

    // 计算生长后线段的投影累计长度
    double projectedLength1 = ProjectedLength(grownLine1);
    double projectedLength2 = ProjectedLength(grownLine2);

    // 判断投影长度是否大于阈值
    double parallelSpotLengthThreshold = 2.5; // 平行车位长度阈值
    return projectedLength1 > parallelSpotLengthThreshold &&
           projectedLength2 > parallelSpotLengthThreshold;
}
bool CheckParkingSpotConstraints(const pair<LineSegment, LineSegment>& parkingSpot) {
    const LineSegment& line1 = parkingSpot.first;
    const LineSegment& line2 = parkingSpot.second;

    // 计算车槽深度
    double depthThreshold = 0.8; // 车槽深度阈值
    double depth = CalculateDepth(line1, line2);
    if (depth < depthThreshold) return false;

    // 计算车槽长度
    double lengthThreshold = 5.0; // 车槽长度阈值
    double length = CalculateLength(line1, line2);
    if (length < lengthThreshold) return false;

    // 计算横向偏移
    double offsetThreshold = 0.3; // 横向偏移阈值
    double offset = CalculateOffset(line1, line2);
    if (offset > offsetThreshold) return false;

    // 计算偏移角度
    double angleThreshold = 10.0 * M_PI / 180.0; // 偏移角度阈值(10度)
    double angle = CalculateAngle(line1, line2);
    if (angle > angleThreshold) return false;

    // 所有约束条件均满足
    return true;
}
double CalculateParkingSpotAngle(const pair<LineSegment, LineSegment>& parkingSpot,
                                  const vector<LineSegment>& curbs,
                                  const vector<LineSegment>& obstacles,
                                  double vehicleHeading) {
    const LineSegment& line1 = parkingSpot.first;
    const LineSegment& line2 = parkingSpot.second;

    // 尝试使用路牙线段计算车位角度
    double curbAngle = GetAngleFromCurb(line1, line2, curbs);
    if (curbAngle != numeric_limits<double>::max()) {
        return curbAngle;
    }

    // 尝试使用障碍物线段计算车位角度
    double obstacleAngle = GetAngleFromObstacles(line1, line2, obstacles);
    double offsetAngleThreshold = 10.0 * M_PI / 180.0; // 偏移角度阈值(10度)
    if (obstacleAngle != numeric_limits<double>::max() &&
        fabs(obstacleAngle - atan2(line2.p2.y - line2.p1.y, line2.p2.x - line2.p1.x)) < offsetAngleThreshold) {
        return obstacleAngle;
    }

    // 使用车辆航向作为车位角度
    return vehicleHeading;
}
// 让线段向外延长一定距离
LineSegment GrowLineSegment(const LineSegment& line, double growLength = 1.0) {
    Point2D dir = line.p2 - line.p1;
    dir = dir.Normalize();

    Point2D p1 = line.p1 - dir * growLength;
    Point2D p2 = line.p2 + dir * growLength;

    return LineSegment(p1, p2);
}

// 计算线段在车辆行驶方向上的投影长度
double ProjectedLength(const LineSegment& line) {
    double angle = atan2(line.p2.y - line.p1.y, line.p2.x - line.p1.x);
    Point2D p1Proj(line.p1.x * cos(angle) - line.p1.y * sin(angle),
                   line.p1.x * sin(angle) + line.p1.y * cos(angle));
    Point2D p2Proj(line.p2.x * cos(angle) - line.p2.y * sin(angle),
                   line.p2.x * sin(angle) + line.p2.y * cos(angle));

    return fabs(p2Proj.x - p1Proj.x);
}

// 计算两条线段之间的深度（近端点间距离）
double CalculateDepth(const LineSegment& line1, const LineSegment& line2) {
    double angle = atan2(line1.p2.y - line1.p1.y, line1.p2.x - line1.p1.x);
    Point2D line1Proj1(line1.p1.x * cos(angle) - line1.p1.y * sin(angle),
                       line1.p1.x * sin(angle) + line1.p1.y * cos(angle));
    Point2D line1Proj2(line1.p2.x * cos(angle) - line1.p2.y * sin(angle),
                       line1.p2.x * sin(angle) + line1.p2.y * cos(angle));
    Point2D line2Proj1(line2.p1.x * cos(angle) - line2.p1.y * sin(angle),
                       line2.p1.x * sin(angle) + line2.p1.y * cos(angle));
    Point2D line2Proj2(line2.p2.x * cos(angle) - line2.p2.y * sin(angle),
                       line2.p2.x * sin(angle) + line2.p2.y * cos(angle));

    double minDist = numeric_limits<double>::max();
    minDist = min(minDist, fabs(line1Proj2.x - line2Proj1.x));
    minDist = min(minDist, fabs(line1Proj2.x - line2Proj2.x));
    minDist = min(minDist, fabs(line1Proj1.x - line2Proj1.x));
    minDist = min(minDist, fabs(line1Proj1.x - line2Proj2.x));

    return minDist;
}

// 计算两条线段之间的长度
double CalculateLength(const LineSegment& line1, const LineSegment& line2) {
    double angle = atan2(line1.p2.y - line1.p1.y, line1.p2.x - line1.p1.x);
    Point2D line1Proj1(line1.p1.x * cos(angle) - line1.p1.y * sin(angle),
                       line1.p1.x * sin(angle) + line1.p1.y * cos(angle));
    Point2D line1Proj2(line1.p2.x * cos(angle) - line1.p2.y * sin(angle),
                       line1.p2.x * sin(angle) + line1.p2.y * cos(angle));
    Point2D line2Proj1(line2.p1.x * cos(angle) - line2.p1.y * sin(angle),
                       line2.p1.x * sin(angle) + line2.p1.y * cos(angle));
    Point2D line2Proj2(line2.p2.x * cos(angle) - line2.p2.y * sin(angle),
                       line2.p2.x * sin(angle) + line2.p2.y * cos(angle));

    double maxX = max(max(line1Proj1.x, line1Proj2.x), max(line2Proj1.x, line2Proj2.x));
    double minX = min(min(line1Proj1.x, line1Proj2.x), min(line2Proj1.x, line2Proj2.x));

    return maxX - minX;
}

// 计算两条线段的横向偏移
double CalculateOffset(const LineSegment& line1, const LineSegment& line2) {
    double angle = atan2(line1.p2.y - line1.p1.y, line1.p2.x - line1.p1.x);
    Point2D line1Proj1(line1.p1.x * cos(angle) - line1.p1.y * sin(angle),
                       line1.p1.x * sin(angle) + line1.p1.y * cos(angle));
    Point2D line1Proj2(line1.p2.x * cos(angle) - line1.p2.y * sin(angle),
                       line1.p2.x * sin(angle) + line1.p2.y * cos(angle));
    Point2D line2Proj1(line2.p1.x * cos(angle) - line2.p1.y * sin(angle),
                       line2.p1.x * sin(angle) + line2.p1.y * cos(angle));
    Point2D line2Proj2(line2.p2.x * cos(angle) - line2.p2.y * sin(angle),
                       line2.p2.x * sin(angle) + line2.p2.y * cos(angle));

    double offset = numeric_limits<double>::max();
    offset = min(offset, fabs(line1Proj1.y - line2Proj1.y));
    offset = min(offset, fabs(line1Proj1.y - line2Proj2.y));
    offset = min(offset, fabs(line1Proj2.y - line2Proj1.y));
    offset = min(offset, fabs(line1Proj2.y - line2Proj2.y));

    return offset;
}

// 计算两条线段之间的夹角
double CalculateAngle(const LineSegment& line1, const LineSegment& line2) {
    Point2D dir1 = line1.p2 - line1.p1;
    Point2D dir2 = line2.p2 - line2.p1;

    double len1 = dir1.Length();
    double len2 = dir2.Length();

    if (len1 == 0 || len2 == 0) {
        return 0;
    }

    dir1 = dir1 / len1;
    dir2 = dir2 / len2;

    double dotProduct = dir1.x * dir2.x + dir1.y * dir2.y;
    double angle = acos(dotProduct);

    return angle;
}

// 从路牙线段中计算车位角度
double GetAngleFromCurb(const LineSegment& line1, const LineSegment& line2, const vector<LineSegment>& curbs) {
    double minAngle = numeric_limits<double>::max();
    LineSegment bestCurb;

    for (const auto& curb :
