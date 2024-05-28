/**
 * @file parking_spot_test.cc
 * @brief
 * @author tangpeng
 * @version 1.0
 * @date 2024-05-28
 * @copyright Copyright (c) 2024 tangpeng. All rights reserved.
 */
#include <fstream>
#include <iostream>
#include <map>
#include <vector>

#include <opencv2/opencv.hpp>

#include "ultrasonic_detection/base/point.h"

using namespace cv;

bool ReadParkingSpotLog(const std::string& read_file,
                        std::map<size_t, std::vector<Point2D>>& spots) {
  std::ifstream input_file(read_file);
  if (!input_file.is_open()) {
    std::cerr << "open failed.\n";
    return false;
  }

  std::string line;
  while (std::getline(input_file, line)) {
    std::istringstream iss(line);
  }

  return true;
}

int main() {
  // 读取日志消息，提取车位信息

  // 车位0角点坐标
  std::vector<Point2D> spot0;
  spot0.emplace_back(Point2D(10, 10));
  spot0.emplace_back(Point2D(40, 10));
  spot0.emplace_back(Point2D(40, 40));
  spot0.emplace_back(Point2D(10, 40));

  // 车位1角点坐标
  std::vector<Point2D> spot1;
  spot1.emplace_back(Point2D(50, 10));
  spot1.emplace_back(Point2D(80, 10));
  spot1.emplace_back(Point2D(80, 40));
  spot1.emplace_back(Point2D(50, 40));

  // 所有的检测车位
  std::map<size_t, std::vector<Point2D>> spots;
  spots[0] = spot0;
  spots[1] = spot1;

  while (1) {
    // 创建图像
    // Scalar(蓝， 绿， 红)
    Mat image(800, 600, CV_8UC3, Scalar(255, 255, 255));

    // 绘制矩形框
    for (const auto& spot : spots) {
      cv::Point pt1(spot.second[0].x, spot.second[0].y);
      cv::Point pt2(spot.second[2].x, spot.second[2].y);

      rectangle(image, pt1, pt2, Scalar(0, 255, 0), 2);
    }

    imshow("Parking Spot", image);
    waitKey(0);
  }

  return 0;
}