#include "cluster.h"

std::vector<Cluster> Cluster::ClusterPoints(const std::vector<Point2D> &points)
{
  std::vector<Cluster> clusters;
  std::vector<bool> clustered(points.size(), false);

  for(size_t i = 0; i < points.size(); ++i){
    if (clustered[i]) continue;
    Cluster cluster = {points[i]};
    clustered[i] = true;
    // 计算相邻点的距离，满足阈值认为是一个簇
    for(size_t j = i + 1; j < points.size(); ++j){
      if(clustered[j]) continue;
      double dist = hypot((points[i] - points[j]).x, (points[i] - points[j]).y);
      if(dist < line_fit_params_.cluster_dist_threshold){
        cluster.emplace_back(points[j]);
        clustered[j] = true;
      }
    }
    // 当簇内点的数量大于阈值时，认为是有效簇
    if(cluster.size() > line_fit_params_.min_cluster_size){
      clusters.emplace_back(cluster);
    }
  }
  return clusters;
}