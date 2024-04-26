#pragma once
#include <vector>
#include <algorithm>

#include "ultrasonic_detection/base/point.h"
#include "ultrasonic_detection/base/params.h"

using Cluster = std::vector<Point2D>;

class LineSegment {
  public:
    LineSegment(){}
    LineSegment(const Point2D& start, const Point2D& end)
      :  start_(start), end_(end) {}
    // 线的长度
    double Length() const {
      return (end_ - start_).Length();
    }
    // 向量
    Point2D GetDirection() const {
      return end_ - start_;
    }
    // 起点
    Point2D GetStart() const { return start_; }
    // 终点
    Point2D GetEnd() const { return end_; }

    // 特征线段拟合
    std::vector<LineSegment> FitLineSegments(const std::vector<Point2D>& points);
    // 线段在车辆航向上的投影
    double ProjectLength(double heading) const;
    // 计算点到线段的距离
    double DistanceToLine(const Point2D& p, const LineSegment& line);
  private:
    // 对点云进行聚类
    std::vector<Cluster> ClusterPoints(const std::vector<Point2D> &points);
    // 对点云簇拟合线段
    LineSegment FitLineSegment(const Cluster& cluster);
    // 合并相邻线段
    std::vector<LineSegment> MergeLineSegments(const std::vector<LineSegment>& segments);

    Point2D start_, end_;
    LineFitParams line_fit_params_;
};