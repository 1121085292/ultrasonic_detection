#pragma once
#include <vector>
#include <algorithm>

#include "ultrasonic_detection/base/point.h"
#include "ultrasonic_detection/base/params.h"

using Cluster = std::vector<Point2D>;

class LineSegment {
  public:
    LineSegment(){}
    LineSegment(const Point2D& p1, const Point2D& p2)
      :  p1_(p1), p2_(p2) {}
    // 线的长度
    double Length() const {
      return hypot((p2_ - p1_).x, (p2_ - p1_).y);
    }
    // 端点一
    Point2D GetPoint1() const { return p1_; }
    // 端点二
    Point2D GetPoint2() const { return p2_; }

    // 特征线段拟合
    std::vector<LineSegment> FitLineSegments(const std::vector<Point2D>& points);
  private:
    // 计算点到线段的距离
    double DistanceToLine(const Point2D& p, const LineSegment& line);
    // 对点云进行聚类
    std::vector<Cluster> ClusterPoints(const std::vector<Point2D> &points);
    // 对点云簇拟合线段
    LineSegment FitLineSegment(const Cluster& cluster);
    // 合并相邻线段
    std::vector<LineSegment> MergeLineSegments(const std::vector<LineSegment>& segments);
    Point2D p1_, p2_;
    LineFitParams line_fit_params_;
};