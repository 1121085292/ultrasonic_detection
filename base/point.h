#pragma once
#include <cmath>

struct Point2D {
  double x, y;
  Point2D(double x = 0, double y = 0) 
    :  x(x), y(y) {}
  
  Point2D& operator-(const Point2D& other){
    return Point2D(x - other.x, y - other.y);
  }

  double Length(){
    return hypot(x, y);
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