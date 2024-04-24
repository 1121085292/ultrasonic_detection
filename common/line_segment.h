#pragma once
#include <vector>

#include "cluster.h"

class LineSegment {
  public:
    LineSegment(const Point2D& p1, const Point2D& p2)
      :  p1_(p1), p2_(p2) {}
    // 线的长度
    double Length(){
      return hypot((p2_ - p1_).x, (p2_ - p1_).y);
    }
    // 特征线段拟合
    std::vector<LineSegment> FitLineSegments(const std::vector<Point2D>& points);
  private:
    // 计算点到线段的距离
    double DistanceToLine(const Point2D& p, const LineSegment& line);
    // 对点云簇拟合线段
    LineSegment FitLineSegment(const Cluster& cluster);
    // 合并相邻线段
    std::vector<LineSegment> MergeLineSegments(const std::vector<LineSegment>& segments);
    
    Cluster cluster_;
    Point2D p1_, p2_;
    LineFitParams line_fit_params_;
    
};