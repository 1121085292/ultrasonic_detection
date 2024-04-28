#include "min_filter.h"

void MinFilter::FilterData(const std::vector<int> &distances, 
                           std::vector<int> &filtered_distances){
  assert(!distances.empty());
  filtered_distances.clear();
  
  int data_size = distances.size();

  // 对每个数据点应用最小值滤波
  for (int i = 0; i < data_size; ++i) {
    int min_value = distances[i];
    
    // 确保窗口范围不超出数据范围
    int start = std::max(0, i - window_size_ / 2);
    int end = std::min(data_size - 1, i + window_size_ / 2);

    // 在窗口范围内寻找最小值
    // 子序列
    std::vector<int> sub_quene;
    for(int i = start; i <= end; ++i){
      sub_quene.emplace_back(distances[i]);
    }
    std::sort(sub_quene.begin(), sub_quene.end());
    // 找到序列中满足距离阈值的最小值
    for(size_t i = 0; i < sub_quene.size(); ++i){
      if(sub_quene[i] < threshold_){
        continue;
      }
      min_value = sub_quene[i];
      break;
    }
    // 序列中所有数用最小值填充
    if(min_value >= threshold_){
      filtered_distances.emplace_back(min_value);
    }
  }
}