#pragma once
#include <algorithm>
#include <vector>

#include "ultrasonic_detection/base/params.h"
#include "ultrasonic_detection/base/point.h"

using Cluster = std::vector<Point2D>;

class LineSegment {
 public:
  LineSegment() {}
  LineSegment(const Point2D& start, const Point2D& end)
      : start_(start), end_(end) {}
  // 线段长度
  double Length() const { return (end_ - start_).Length(); }
  // 向量
  Point2D GetDirection() const { return end_ - start_; }
  // 起点
  Point2D GetStart() const { return start_; }
  // 终点
  Point2D GetEnd() const { return end_; }

  // 初始化，参数赋值
  void Init(const LineFitParams& line_fit_params);

  // 特征线段拟合
  std::vector<LineSegment> FitLineSegments(
      const std::vector<Point2D>& points, const LineFitParams& line_fit_params);
  // 计算两线段夹角
  double AngleBetweenLines(const LineSegment& line);
  // 线段在车辆航向上的投影
  double ProjectLength(double heading) const;
  // 计算点到直线的距离
  double DistanceToLine(const Point2D& p, const LineSegment& line);

 private:
  // 对点云进行聚类
  std::vector<Cluster> ClusterPoints(const std::vector<Point2D>& points);
  // 递归划分线段
  void DivideLineSegments(std::vector<LineSegment>& segments,
                          const std::vector<Point2D>& points);
  // 合并相邻线段
  void MergeLineSegments(std::vector<LineSegment>& segments);
  // 计算线段与X轴正向的夹角
  double LineAngle(const LineSegment& line);

  bool is_init_ = false;
  Point2D start_, end_;
  LineFitParams line_fit_params_;
};