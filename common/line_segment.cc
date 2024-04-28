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
  std::vector<LineSegment> line_segments;
  // 聚类
  std::vector<Cluster> clusters = ClusterPoints(points);
  for (const auto& cluster : clusters) {
    if (static_cast<int>(cluster.size()) < line_fit_params_.min_cluster_size) continue;
    
    LineSegment line(cluster.front(), cluster.back());
    std::vector<Point2D> inliers = {line.start_, line.end_};
    std::vector<Point2D> remaining_points = cluster;
    remaining_points.erase(remaining_points.begin());
    remaining_points.erase(remaining_points.end() - 1);

    while(!remaining_points.empty()){
      double max_distance = 0;
      Point2D max_point;
      for(const auto& p : remaining_points){
        double dist = DistanceToLine(p, line);
        if(dist > max_distance){
          max_distance = dist;
          max_point = p;
        }
      }
      if(max_distance > line_fit_params_.merge_dist_threshold){
        Cluster split1(inliers.begin(), inliers.end());
        split1.emplace_back(max_point);
        LineSegment line1 = FitLineSegment(split1);
        line_segments.emplace_back(line1);

        std::vector<Point2D> split2 = {max_point};
        split2.insert(split2.end(), remaining_points.begin(), remaining_points.end());
        inliers = split2;
        line = FitLineSegment(inliers);
      } else {
        inliers.insert(inliers.end(), remaining_points.begin(), remaining_points.end());
        line = FitLineSegment(inliers);
        break;
      }
      remaining_points.erase(std::remove_if(remaining_points.begin(), remaining_points.end(),
                          [&max_point](const Point2D& p) { return p.x == max_point.x && p.y == max_point.y;}),
                           remaining_points.end());
    }
    line_segments.emplace_back(line);
  }
  return MergeLineSegments(line_segments);
}

double LineSegment::ProjectLength(double heading) const
{
  Point2D direction(cos(heading), sin(heading));
  Point2D d = GetDirection();
  double dp = d.x * direction.x + d.y * direction.y;
  Point2D proj = direction * dp;
  return LineSegment(start_, start_ + proj).Length();
}

double LineSegment::DistanceToLine(const Point2D &p, const LineSegment &line)
{
  // 线段长度
  const double length = line.Length();
  // 
  double pd_x = p.x - line.start_.x;
  double pd_y = p.y - line.start_.y;
  Point2D d((line.end_ - line.start_).x, (line.end_ - line.start_).y);
  double offset = pd_x * d.x;
  if(length > 0){
    offset /= length;
  }

  if(offset < 0){
    return hypot(pd_x, pd_y);
  } else if(offset > length){
    return hypot(p.x - line.end_.x, p.y - line.end_.y);
  } else {
    return fabs(pd_x * d.x - pd_y * d.y) / length;
  }
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

LineSegment LineSegment::FitLineSegment(const Cluster &cluster){
  double sum_x = 0, sum_y = 0;
  for(const auto& point : cluster){
    sum_x += point.x;
    sum_y += point.y;
  }
  // 簇的质心代替线段中心
  Point2D centroid(sum_x / cluster.size(), sum_y / cluster.size());
  // 簇相对质心的二阶矩
  double sumXX = 0, sumXY = 0, sumYY = 0;
  for(const auto& p : cluster) {
    double dx =  p.x - centroid.x;
    double dy = p.y - centroid.y;
    sumXX += dx * dx;
    sumXY += dx * dy;
    sumYY += dy * dy;
  }
  // 拟合线段的角度
  double theta = 0.5 * atan2(2*sumXY, sumXX - sumYY);
  double cos_theta = cos(theta);
  double sin_theta = sin(theta);
  // 拟合线段端点
  Point2D p1(centroid.x + cos_theta, centroid.y + sin_theta);
  Point2D p2(centroid.x - cos_theta, centroid.y - sin_theta);
  return LineSegment(p1, p2);
}

std::vector<LineSegment> LineSegment::MergeLineSegments(const std::vector<LineSegment> &segments){
  std::vector<LineSegment> merged;
  merged.emplace_back(segments[0]);

  for (size_t i = 1; i < segments.size(); ++i) {
    const LineSegment& curr = segments[i];
    const LineSegment& prev = merged.back();

    double dx1 = prev.end_.x - prev.start_.x;
    double dy1 = prev.end_.y - prev.start_.y;
    double dx2 = curr.end_.x - curr.start_.x;
    double dy2 = curr.end_.y - curr.start_.y;
    double cos_angle = (dx1*dx2 + dy1*dy2) / (sqrt(dx1*dx1 + dy1*dy1) * sqrt(dx2*dx2 + dy2*dy2));
    double angle = acos(cos_angle);
    if (angle < line_fit_params_.merge_angle_threshold * M_PI / 180) {
      double dist = DistanceToLine(curr.start_,  prev);
      if (dist < line_fit_params_.merge_dist_threshold) {
        Point2D newP2(prev.end_.x + dx1, prev.end_.y + dy1);
        merged.back() = LineSegment(prev.start_, newP2);
      } else {
        merged.emplace_back(curr);
      }
    } else {
      merged.emplace_back(curr);
    }
  }
  return merged;
}
