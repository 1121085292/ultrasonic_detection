/**
 * @file point.h
 * @brief
 * @author tangpeng
 * @version 1.0
 * @date 2024-05-28
 * @copyright Copyright (c) 2024 tangpeng. All rights reserved.
 */
#pragma once
#include <cmath>

struct Point2D {
  double x, y;
  Point2D(double x = 0, double y = 0) : x(x), y(y) {}
  // 重载-号
  Point2D operator-(const Point2D& other) const {
    return Point2D(x - other.x, y - other.y);
  }
  // 重载+号
  Point2D operator+(const Point2D& other) const {
    return Point2D(x + other.x, y + other.y);
  }
  // 重载*号
  Point2D operator*(double value) const {
    return Point2D(x * value, y * value);
  }
  // 点乘
  double dot(const Point2D& other) const { return x * other.x + y * other.y; }
  // 叉乘
  double cross(const Point2D& other) const { return x * other.y - y * other.x; }
  // 模长
  double Length() const { return hypot(x, y); }
  // 单位向量
  Point2D unit() const {
    double n = Length();
    return Point2D(x / n, y / n);
  }
};

struct Point3D {
  double x, y, angle;
  Point3D() {}

  Point3D(double x_val, double y_val, double angle_val)
      : x(x_val), y(y_val), angle(angle_val) {}

  double Length() const { return hypot(x, y); }

  // 重载-号
  Point3D operator-(const Point3D& other) const {
    return Point3D(x - other.x, y - other.y, 0.0);
  }
};