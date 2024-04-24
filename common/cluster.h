#pragma once

#include "ultrasonic_detection/base/point.h"
#include "ultrasonic_detection/base/params.h"

class Cluster {
  public:
    // 对点云进行聚类
    std::vector<Cluster> ClusterPoints(const std::vector<Point2D> &points);
  private:
    ClusterParams cluster_params_;
};