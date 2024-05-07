#include "line_segment.h"

void LineSegment::Init(const LineFitParams &line_fit_params)
{
  if(!is_init_){
    line_fit_params_.min_fit_num = line_fit_params.min_fit_num;
    line_fit_params_.min_cluster_size = line_fit_params.min_cluster_size;
    line_fit_params_.cluster_dist_threshold = line_fit_params.cluster_dist_threshold;
    line_fit_params_.merge_angle_threshold = line_fit_params.merge_angle_threshold;
    line_fit_params_.merge_dist_threshold = line_fit_params.merge_dist_threshold;
    
    is_init_ = true;
  }
}

std::vector<LineSegment> LineSegment::FitLineSegments(
    const std::vector<Point2D> &points,
    const LineFitParams &line_fit_params)
{
  if(!is_init_){
    Init(line_fit_params);
  }
  std::vector<LineSegment> segments;
  // 聚类
  std::vector<Cluster> clusters = ClusterPoints(points);

  if(!clusters.empty()){
    for (const auto& cluster : clusters) {
      if (static_cast<int>(cluster.size()) < line_fit_params_.min_cluster_size) continue;
      // 初始线段为起点和终点连线
      LineSegment initial_segment(cluster.front(), cluster.back());
      
      // 递归划分线段
      std::vector<LineSegment> divided_segments = {initial_segment};
      DivideLineSegments(divided_segments, cluster);
      
      // 合并相邻线段
      MergeLineSegments(divided_segments);
      
      // 将合并后的线段添加到结果
      segments.insert(segments.end(), divided_segments.begin(), divided_segments.end());
    }
  }
  
  return segments;
}

double LineSegment::AngleBetweenLines(const LineSegment &line){
  auto dir1 = GetDirection();
  auto dir2 = line.GetDirection();

  double dot = dir1.dot(dir2);
  // 弧长
  return fabs(acos( dot / (Length() * line.Length())));
}

double LineSegment::ProjectLength(double heading) const
{
  auto line1 = LineSegment(start_, end_);
  // 单位向量
  Point2D direction(cos(heading), sin(heading));
  LineSegment line2(Point2D(0.0, 0.0), direction);
  // 两线段夹角
  double angle = line1.AngleBetweenLines(line2);
  return Length() * angle;
}

double LineSegment::DistanceToLine(const Point2D &p, const LineSegment &line)
{
  // 线段长度
  const double length = line.Length();
  // 直线
  double A = (line.end_ - line.start_).y;
  double B = (line.start_ - line.end_).x;
  double C = line.end_.x * line.start_.y - line.start_.x * line.end_.y;
  
  return fabs(A * p.x + B * p.y + C) / length;
}

std::vector<Cluster> LineSegment::ClusterPoints(const std::vector<Point2D> &points){
  std::vector<Cluster> clusters;
  std::vector<bool> clustered(points.size(), false);

  for(size_t i = 0; i < points.size(); ++i){
    if (clustered[i]) continue;
    Cluster cluster = {points[i]};
    clustered[i] = true;
    // 计算相邻点的距离，满足阈值认为是一个簇
    for(size_t j = i + 1; j < points.size(); ++j){
      if(clustered[j]) continue;
      double dist = hypot(points[i].x - points[j].x, points[i].y - points[j].y);
      if(dist < line_fit_params_.cluster_dist_threshold){
        cluster.emplace_back(points[j]);
        clustered[j] = true;
      }
    }
    // 当簇内点的数量大于阈值时，认为是有效簇
    if(static_cast<int>(cluster.size()) >= line_fit_params_.min_cluster_size){
      clusters.emplace_back(cluster);
    }
  }
  return clusters;
}

// 计算线段与X轴正向的夹角
double LineSegment::LineAngle(const LineSegment& line) {
  double dx = line.end_.x - line.start_.x;
  double dy = line.end_.y - line.start_.y;
  return atan2(dy, dx);
}

// 递归划分线段
void LineSegment::DivideLineSegments(std::vector<LineSegment>& segments, const std::vector<Point2D>& points) {
  std::vector<LineSegment> new_segments;
  for (const auto& segment : segments) {
    double max_dist = 0;
    size_t split_idx = -1;
    for (size_t j = 0; j < points.size(); j++) {
      double dist = DistanceToLine(points[j], segment);
      if (dist > max_dist) {
        max_dist = dist;
        split_idx = j;
      }
    }
    
    if (max_dist > line_fit_params_.merge_dist_threshold && static_cast<int>(split_idx) != -1) {
      LineSegment seg1(segment.start_, points[split_idx]);
      LineSegment seg2(points[split_idx], segment.end_);
      new_segments.push_back(seg1);
      new_segments.push_back(seg2);
      
      std::vector<Point2D> points1, points2;
      for (size_t j = 0; j < points.size(); j++) {
        if (j < split_idx) {
          points1.push_back(points[j]);
        } else if (j > split_idx) {
          points2.push_back(points[j]);
        }
      }
      
      DivideLineSegments(new_segments, points1);
      DivideLineSegments(new_segments, points2);
    } else {
      new_segments.push_back(segment);
    }
  }
  segments = new_segments;
}

// 合并相邻线段
void LineSegment::MergeLineSegments(std::vector<LineSegment>& segments) {
  std::vector<LineSegment> merged_segments;
  for (size_t i = 0; i < segments.size(); i++) {
    if (i == segments.size() - 1 || 
        fabs(LineAngle(segments[i]) - LineAngle(segments[i + 1])) > line_fit_params_.merge_angle_threshold) {
      merged_segments.push_back(segments[i]);
    } else {
      std::vector<Point2D> points = {segments[i].start_, segments[i].end_};
      while (static_cast<size_t>(i + 1) < segments.size() &&
            fabs(LineAngle(segments[i]) - LineAngle(segments[i + 1])) <= line_fit_params_.merge_angle_threshold) {
        points.push_back(segments[i + 1].end_);
        i++;
      }
      // 最小二乘法拟合线段
      double min_error = INFINITY;
      LineSegment best_fit;
      for (size_t j = 0; j < points.size() - 1; j++) {
        for (size_t k = j + 1; k < points.size(); k++) {
          LineSegment candidate(points[j], points[k]);
          double error = 0;
          for (const auto& p : points) {
            error += pow(DistanceToLine(p, candidate), 2);
          }
          if (error < min_error) {
            min_error = error;
            best_fit = candidate;
          }
        }
      }
      merged_segments.push_back(best_fit);
    }
  }
  
  segments = merged_segments;
}
