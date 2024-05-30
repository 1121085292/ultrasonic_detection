#pragma once
#include <algorithm>
#include <cassert>
#include <vector>

class MinFilter {
 public:
  MinFilter(int size, int threshold)
      : window_size_(size), threshold_(threshold) {}

  // @brief 最小值滤波，去除异常点
  // @param[in] distances: 探头输出的障碍物距离序列
  // @param[out] filtered_distances：最小值滤波后的障碍物距离序列
  void FilterData(const std::vector<int>& distances,
                  std::vector<int>& filtered_distances);

 private:
  int window_size_;
  int threshold_;
};