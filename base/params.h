#pragma once
#include <cmath>

struct MinFilterParams {
  int window_size = 3;
  int threshold = 300;    //mm
};

struct UltrasonicParams {
  int max_range = 6000;
};

struct Vehicle{
  int width = 1875;
  int wheel_base = 2920;
  int front_delta = 354;
  int rear_delta = 383;
};

struct LineFitParams
{
  int min_fit_num = 10;
  float min_fit_distance = 0.6f;
  double merge_angle_threshold = 10 * M_PI / 180;
  double merge_dist_threshold = 0.05;
  double cluster_dist_threshold = 0.5;
  int min_cluster_size = 5;
};
