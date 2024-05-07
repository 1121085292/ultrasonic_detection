#pragma once

struct MinFilterParams {
  int window_size = 3;
  int threshold = 20;    //mm
};

struct UltrasonicParams {
  int max_range = 5000;
};

struct LineFitParams {
  int min_fit_num;
  int min_cluster_size;
  double cluster_dist_threshold;
  double merge_angle_threshold;
  double merge_dist_threshold;
};

struct ParkingSpotParams{
  // 边界长度阈值
  double line_length_threshold = 2500.0;
  double line_width_threshold = 800.0;
  // 车位长度
  double spot_length_threshold = 5200.0;
  // 车位深度
  double spot_width_threshold = 2500.0;
  // 线段生长阈值
  double grow_line_dist;
  double grow_line_angle;
  // 特征线段长度阈值
  double min_line_length;
  // 横向偏移
  double offset_threshold;
  // 偏移角度
  double angle_threshold;
};

struct CurbParams
{
  double angle_threshold;
  double length_threshold;
};

