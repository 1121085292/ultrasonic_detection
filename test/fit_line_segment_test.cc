#include <iostream>
#include <vector>
#include <cstdlib>
#include <ctime>

#include "ultrasonic_detection/common/line_segment.h"

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace cv;
using namespace std;

const int WIDTH = 800;
const int HEIGHT = 600;
const int MAX_POINTS = 1000;

vector<Point2D> generateRandomPoints(int numPoints, double stdDev, double dirX, double dirY)
{
  vector<Point2D> points;
  double meanX = WIDTH / 2.0;
  double meanY = HEIGHT / 2.0;

  for (int i = 0; i < numPoints; i++)
  {
    double x = meanX + dirX * i + stdDev * (rand() % 5 + 3.5);
    double y = meanY + dirY * i + stdDev * (rand() % 5 + 3.5);
    points.emplace_back(Point2D(x, y));
  }

  return points;
}

int main()
{
  LineFitParams lf;
  lf.min_fit_num = 10;
  lf.min_cluster_size = 5;
  lf.cluster_dist_threshold = 50;
  lf.merge_angle_threshold = 10;
  lf.merge_dist_threshold = 50;

  srand(time(0));
  namedWindow("Line Segment Fitting", WINDOW_AUTOSIZE);

  double stdDev = 10.0;
  double dirX = 3.0;
  double dirY = 2.0;

  vector<Point2D> points;
  int numPoints = 0;

  while (true)
  {
    Mat image(HEIGHT, WIDTH, CV_8UC3, Scalar(0, 0, 0));

    if (numPoints < MAX_POINTS)
    {
      int numNewPoints = rand() % 10 + 1;
      vector<Point2D> newPoints = generateRandomPoints(numNewPoints, stdDev, dirX, dirY);
      points.insert(points.end(), newPoints.begin(), newPoints.end());
      numPoints += numNewPoints;
    }

    for (const Point2D& p : points)
    {
      circle(image, Point(p.x, p.y), 2, Scalar(0, 0, 0), -1);
    }

    vector<LineSegment> lineSegments = LineSegment().FitLineSegments(points, lf);
    for (const LineSegment& ls : lineSegments)
    {
      line(image, Point(ls.GetStart().x, ls.GetStart().y),
                  Point(ls.GetEnd().x, ls.GetEnd().y), Scalar(0, 0, 255), 2);
    }

    imshow("Line Segment Fitting", image);
    char key = waitKey(30);
    if (key == 27) // Esc键退出
      break;
  }

  return 0;
}
