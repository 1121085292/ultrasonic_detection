#include "line_segment.h"

std::vector<LineSegment> LineSegment::FitLineSegments(const std::vector<Point2D> &points)
{
  std::vector<LineSegment> line_segments;
  // 聚类
  std::vector<Cluster> clusters = cluster_.ClusterPoints(points);
  for (const auto& cluster : clusters) {
    if (cluster.size() < cluster_.cluster_params_.min_cluster_size) continue;
    
    LineSegment line(cluster.front(), cluster.back());
    vector<Point2D> inliers = {line.p1, line.p2};
    vector<Point2D> remaining_points = cluster;
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
        std::vector<Point2D> split1(inliers.begin(), inliers.end());
        split1.emplace_back(max_point);
        LineSegment line1 = FitLineSegment(split1);
        line_segments.emplace_back(line1);

        std::vector<Point2D> split2 = {max_point};
        split2.insert(split2.end(), remaining_points.begin(), remaining_points.end());
        inliers = split2;
        line = FitLineSegment(inliers);
      } else {
        inliers.insert(inliers.end(), remaining_points.begin(), remaining_points.end())
        line = FitLineSegment(inliers);
        break;
      }
      remaining_points.erase(remove_if(remaining_points.begin(), remaining_points.end(),
                          [&maxPoint](const Point2D& p) { return p.x == max_point.x && p.y == max_point.y;}),
                           remaining_points.end());
    }
    line_segments.emplace_back(line);
  }
  return MergeLineSegments(line_segments);
}

double LineSegment::DistanceToLine(const Point2D &p, const LineSegment &line)
{
  // 线段长度
  double length = line.Length(line.p2_, line.p1_);
  // 
  double pd_x = p.x - line.p1_.x;
  double pd_y = p.y - line.p1_.y;
  Point2D d((line.p2_ - line.p1_).x, (line.p2_ - line.p1_).y);
  double offset = pd_x * d.x;
  if(length > 0){
    offset /= length;
  }

  if(offset < 0){
    return hypot(pd_x, pd_y);
  } else if(offset > length){
    return hypot(p.x - line.p2_.x, p.y - line.p2_.y);
  } else {
    return fabs(pd_x * d.x - pd_y * d.y) / len;
  }
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
    double dy = p.y - centtroid.y;
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

    double dx1 = prev.p2.x - prev.p1.x;
    double dy1 = prev.p2.y - prev.p1.y;
    double dx2 = curr.p2.x - curr.p1.x;
    double dy2 = curr.p2.y - curr.p1.y;
    double cos_angle = (dx1*dx2 + dy1*dy2) / (sqrt(dx1*dx1 + dy1*dy1) * sqrt(dx2*dx2 + dy2*dy2));
    double angle = acos(cos_angle);
    if (angle < line_fit_params_.merge_angle_threshold) {
      double dist = DistanceToLine(curr.p1,  prev);
      if (dist < line_fit_params_.merge_dist_threshold) {
        Point2D newP2(prev.p2.x + dx1, prev.p2.y + dy1);
        merged.back() = LineSegment(prev.p1, newP2);
      } else {
        merged.emplace_back(curr);
      }
    } else {
      merged.emplace_back(curr);
    }
  }
  return merged;
}
