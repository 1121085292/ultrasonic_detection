#include "line_segment.h"

int main(){
  std::srand(time(nullptr));
  std::vector<Point2D> points;
  for(int i = 0; i < 20; ++i){
    int x = std::random() % 1000 + 1000;
    int y = std::random() % 1000 + 1000;
    points.emplace_back(Point2D(x, y));
  }

  LineSegment line_segment;
  auto line = line_segment.FitLineSegments(points);
  
}