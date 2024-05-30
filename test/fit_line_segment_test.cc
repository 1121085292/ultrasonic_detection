#include <cstdlib>
#include <ctime>
#include <iostream>
#include <vector>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

#include "ultrasonic_detection/common/line_segment.h"

#include "ultrasonic_detection/common_msgs/ultrasonic.pb.h"

#include "cyber/cyber.h"

#define TEST 0

using namespace cv;
using namespace std;
using common_msgs::ultrasonic::UltrasonicList;

const int WIDTH = 600;
const int HEIGHT = 800;

vector<Point2D> generateRandomPoints(int numPoints, double stdDev, double dirX,
                                     double dirY) {
  vector<Point2D> points;
  double meanX = 0.0;
  double meanY = 0.0;

  for (int i = 0; i < numPoints; i++) {
    double x = meanX + dirX * i + stdDev * (rand() % 10 + 3.5);
    double y = meanY + dirY * i + stdDev * (rand() % 10 + 3.5);
    points.emplace_back(Point2D(x, y));
  }

  return points;
}

int main(int argc, char* argv[]) {
  // 初始化
  apollo::cyber::Init(argv[0]);

  LineFitParams lf;
  lf.min_fit_num = 10;
  lf.min_cluster_size = 5;
  lf.cluster_dist_threshold = 400;
  lf.merge_angle_threshold = 10;
  lf.merge_dist_threshold = 100;

#if TEST == 1
  srand(time(nullptr));

  double stdDev = 10.0;
  double dirX = 50.0;
  double dirY = 20.0;
#endif
  // 障碍物点
  vector<Point2D> points;
  auto listener_node = apollo::cyber::CreateNode("points");
  auto point_listener = listener_node->CreateReader<UltrasonicList>(
      "perception/ultrasonic",
      [&points](const std::shared_ptr<UltrasonicList>& ultra) {
        for (const auto& obj : ultra->ul_objs()) {
          if (obj.orientation() == "FRONT_SIDE_RIGHT") {
            points.emplace_back(
                Point2D(obj.position_global().x(), obj.position_global().y()));
          }
        }
      });

  while (true) {
    Mat image(HEIGHT, WIDTH, CV_8UC3, Scalar(255, 255, 255));
#if TEST == 1
    int numNewPoints = rand() % 50 + 10;
    // 随机点
    points = generateRandomPoints(numNewPoints, stdDev, dirX, dirY);
#endif

    if (points.empty()) continue;
    vector<LineSegment> lineSegments =
        LineSegment().FitLineSegments(points, lf);
    // 输出拟合结果
    cout << lineSegments.size() << endl;
    for (const auto& seg : lineSegments) {
      cout << "Line segment: (" << seg.GetStart().x << "," << seg.GetStart().y
           << ") -> (" << seg.GetEnd().x << "," << seg.GetEnd().y << ")"
           << endl;
    }
    // 绘制输入点云
    for (const Point2D& p : points) {
      circle(image, Point(p.x * 0.5, p.y * 0.5), 2, Scalar(0, 0, 0), -1);
    }
    // 绘制拟合线段
    for (const LineSegment& ls : lineSegments) {
      line(image, Point(ls.GetStart().x * 0.5, ls.GetStart().y * 0.5),
           Point(ls.GetEnd().x * 0.5, ls.GetEnd().y * 0.5), Scalar(0, 0, 255),
           2);
    }
    // 显示结果
    imshow("Line Segment Fitting", image);
    waitKey(0);
    char key = waitKey(30);
    if (key == 27)  // Esc键退出
      break;
  }

  return 0;
}
