#pragma once
#include <cmath>

struct Point2D {
  double x, y;
  Point2D(double x = 0, double y = 0) 
    :  x(x), y(y) {}
  // 重载-号
  Point2D operator-(const Point2D& other) const {
    return Point2D(x - other.x, y - other.y);
  }
  // 重载+号
  Point2D operator+(const Point2D& other) const {
    return Point2D(x + other.x, y + other.y);
  }
  // 点乘
  double dot(const Point2D& other) const {
    return x * other.x + y * other.y;
  }
  // 叉乘
  double cross(const Point2D& other) const {
    return x * other.y - y * other.x;
  }
  // 模长
  double Length(){
    return hypot(x, y);
  }
  // 单位向量
  Point2D unit() const {
    double n = Length();
    return Point2D(x / n, y / n);
  }

};

struct Point3D{
  double x, y, angle;
  Point3D(){}

  Point3D(double x_val, double y_val, double angle_val)
   : x(x_val), y(y_val), angle(angle_val){}

  double Length(){
    return hypot(x, y);
  }
};