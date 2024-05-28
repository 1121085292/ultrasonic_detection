/**
 * @file min_filter_test.cc
 * @brief
 * @author tangpeng
 * @version 1.0
 * @date 2024-05-28
 * @copyright Copyright (c) 2024 tangpeng. All rights reserved.
 */
#include "gtest/gtest.h"

#include "min_filter.h"

TEST(MinFilterTest, case1) {
  // 所有数据满足距离阈值
  MinFilter min_filter(3, 15);
  std::vector<int> distance = {15, 27, 31, 45, 23, 56, 67, 68, 92, 88, 104};
  std::vector<int> filtered_data;
  min_filter.FilterData(distance, filtered_data);

  EXPECT_EQ(15, filtered_data[0]);
  EXPECT_EQ(15, filtered_data[1]);
  EXPECT_EQ(27, filtered_data[2]);
  EXPECT_EQ(23, filtered_data[3]);
  EXPECT_EQ(23, filtered_data[4]);
  EXPECT_EQ(23, filtered_data[5]);
  EXPECT_EQ(56, filtered_data[6]);
  EXPECT_EQ(67, filtered_data[7]);
  EXPECT_EQ(68, filtered_data[8]);
  EXPECT_EQ(88, filtered_data[9]);
  EXPECT_EQ(88, filtered_data[10]);
}

TEST(MinFilterTest, case2) {
  // 部分数据满足距离阈值
  MinFilter min_filter(3, 25);
  std::vector<int> distance = {15, 27, 31, 45, 23, 56, 67, 68, 92, 88, 104};
  std::vector<int> filtered_data;
  min_filter.FilterData(distance, filtered_data);

  EXPECT_EQ(27, filtered_data[0]);
  EXPECT_EQ(27, filtered_data[1]);
  EXPECT_EQ(27, filtered_data[2]);
  EXPECT_EQ(31, filtered_data[3]);
  EXPECT_EQ(45, filtered_data[4]);
  EXPECT_EQ(56, filtered_data[5]);
  EXPECT_EQ(56, filtered_data[6]);
  EXPECT_EQ(67, filtered_data[7]);
  EXPECT_EQ(68, filtered_data[8]);
  EXPECT_EQ(88, filtered_data[9]);
  EXPECT_EQ(88, filtered_data[10]);
}

TEST(MinFilterTest, case3) {
  // 部分数据满足距离阈值，同时更改窗口大小
  MinFilter min_filter(5, 25);
  std::vector<int> distance = {15, 27, 31, 45, 23, 56, 67, 68, 92, 88, 104};
  std::vector<int> filtered_data;
  min_filter.FilterData(distance, filtered_data);

  EXPECT_EQ(27, filtered_data[0]);
  EXPECT_EQ(27, filtered_data[1]);
  EXPECT_EQ(27, filtered_data[2]);
  EXPECT_EQ(27, filtered_data[3]);
  EXPECT_EQ(31, filtered_data[4]);
  EXPECT_EQ(45, filtered_data[5]);
  EXPECT_EQ(56, filtered_data[6]);
  EXPECT_EQ(56, filtered_data[7]);
  EXPECT_EQ(67, filtered_data[8]);
  EXPECT_EQ(68, filtered_data[9]);
  EXPECT_EQ(88, filtered_data[10]);
}

TEST(MinFilterTest, case4) {
  // 整个子序列都不满足距离阈值
  MinFilter min_filter(3, 25);
  std::vector<int> distance = {15, 18, 20, 22, 23, 56, 67, 68, 92, 88, 104};
  std::vector<int> filtered_data;
  min_filter.FilterData(distance, filtered_data);

  EXPECT_EQ(56, filtered_data[0]);
  EXPECT_EQ(56, filtered_data[1]);
  EXPECT_EQ(56, filtered_data[2]);
  EXPECT_EQ(67, filtered_data[3]);
  EXPECT_EQ(68, filtered_data[4]);
  EXPECT_EQ(88, filtered_data[5]);
  EXPECT_EQ(88, filtered_data[6]);
}