#pragma once
#include <cmath>

struct MinFilterParams {
  int window_size = 3;
  int threshold = 300;    //mm
};

struct UltrasonicParams {
  int max_range = 6000;
};

struct Vehicle {
  int width = 1875;
  int wheel_base = 2920;
  int front_delta = 354;
  int rear_delta = 383;
};

struct LineFitParams {
  int min_fit_num = 10;
  double merge_angle_threshold = 10 * M_PI / 180;
  double merge_dist_threshold = 0.05;
  double cluster_dist_threshold = 0.5;
  int min_cluster_size = 5;
  double min_fit_distance = 0.6;
};

struct ParkingSpotParams{
  double line_length_threshold = 2500.0;
  double line_width_threshold = 800.0;
  double line_dist = 0.5;
  double line_angle = 10 * M_PI / 180;
  // 横向偏移
  double offset_threshold = 300.0;
  // 车位长度
  double spot_length_threshold = 5200.0;
  // 车位深度
  double spot_width_threshold = 2500.0;
  // 偏移角度
  double angle_threshold = 10 * M_PI / 180;
};

struct CurbParams
{
  double angle;
  double length;
};

