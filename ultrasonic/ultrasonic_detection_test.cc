#include "ultrasonic_detection.h"
#include "ultrasonic_detection/proto/ultrasonic_conf.pb.h"

#include "gtest/gtest.h"

using ultrasonic_detection::proto::UltrasonicCoordinate;

class UltrasonicDetectionTest : public testing::Test {
  public:
    UltrasonicDetectionTest() {
      // 左侧2个探头在整车坐标系下的位置
      coordinate_map_[0] = Point3D(ultra_coordinate.fsl_x(), ultra_coordinate.fsl_y(), ultra_coordinate.fsl_angle() * M_PI / 180);
      coordinate_map_[11] = Point3D(ultra_coordinate.rsr_x(), ultra_coordinate.rsr_y(), ultra_coordinate.rsr_angle() * M_PI / 180);
      // 前4个探头在整车坐标系下的位置
      coordinate_map_[1] = Point3D(ultra_coordinate.fol_x(), ultra_coordinate.fol_y(), ultra_coordinate.fol_angle() * M_PI / 180);
      coordinate_map_[2] = Point3D(ultra_coordinate.fcl_x(), ultra_coordinate.fcl_y(), ultra_coordinate.fcl_angle() * M_PI / 180);
      coordinate_map_[3] = Point3D(ultra_coordinate.fcr_x(), ultra_coordinate.fcr_y(), ultra_coordinate.fcr_angle() * M_PI / 180);
      coordinate_map_[4] = Point3D(ultra_coordinate.for_x(), ultra_coordinate.for_y(), ultra_coordinate.for_angle() * M_PI / 180);
      // 后4个探头在整车坐标系下的位置
      coordinate_map_[7] = Point3D(ultra_coordinate.rol_x(), ultra_coordinate.rol_y(), ultra_coordinate.rol_angle() * M_PI / 180);
      coordinate_map_[8] = Point3D(ultra_coordinate.rcl_x(), ultra_coordinate.rcl_y(), ultra_coordinate.rcl_angle() * M_PI / 180);
      coordinate_map_[9] = Point3D(ultra_coordinate.rcr_x(), ultra_coordinate.rcr_y(), ultra_coordinate.rcr_angle() * M_PI / 180);
      coordinate_map_[10] = Point3D(ultra_coordinate.ror_x(), ultra_coordinate.ror_y(), ultra_coordinate.ror_angle() * M_PI / 180);
      // 右侧2个探头在整车坐标系下的位置
      coordinate_map_[5] = Point3D(ultra_coordinate.fsr_x(), ultra_coordinate.fsr_y(), ultra_coordinate.fsr_angle() * M_PI / 180);
      coordinate_map_[6] = Point3D(ultra_coordinate.rsl_x(), ultra_coordinate.rsl_y(), ultra_coordinate.rsl_angle() * M_PI / 180);

      int dis0 = 5000;
      int dis1 = 5000;
      distances_.emplace_back(dis0);
      distances_.emplace_back(dis1);
      for(int i = 2; i < 12; ++i){
        distances_.emplace_back(1000);
      }
    }
  protected:
    void SetUp(){}
    void TearDown(){}
  public:
    std::map<int, Point3D> coordinate_map_;
    std::vector<int> distances_;
    UltrasonicCoordinate ultra_coordinate;
};

TEST_F(UltrasonicDetectionTest, TriangleMeasuredPositionCalculateTest){
  // 只对1-4,7-10前后8个雷达进行三角测距
  std::vector<Point2D> pos;
  std::vector<Status> sta;
  for(int i = 0; i < 12; ++i){
    Ultrasonic ultra(coordinate_map_[i], distances_[i]);
    ultra.DirectMeasuredPositionCalculate();
    Point2D position = ultra.GetPosition();
    pos.emplace_back(position);
    Status status = ultra.GetStatus();
    sta.emplace_back(status);
    if(i == 1 || i == 2 || i == 3 ||
       i == 7 || i == 8 || i == 9){
      auto ul = Ultrasonic(coordinate_map_[i], distances_[i]);
      auto ur = Ultrasonic(coordinate_map_[i + 1], distances_[i + 1]);
      UltrasonicDetection ultra_detec(ul, ur);
      ultra_detec.TriangleMeasuredPositionCalculate();
      Point2D position = ultra_detec.GetPosition();
      pos[i] = position;
      Status status = ultra_detec.GetStatus();
      sta[i] = status;
    } else if(i == 4 || i == 10){
      pos[i] = pos[i-1];
      sta[i] = sta[i-1];
    }
  }
  double tolerance = 9 * 1e-1;
  // FOL主探头，FCL辅助探头，FOL未收到回波信号
  EXPECT_NEAR(pos[1].x, 0, tolerance);
  EXPECT_NEAR(pos[1].y, 0, tolerance);
  EXPECT_EQ(sta[1], Status::OverDetection);
  // FCL主探头，FCR辅助探头
  EXPECT_NEAR(pos[2].x, 4637.1, tolerance);
  EXPECT_NEAR(pos[2].y, 0, tolerance);
  EXPECT_EQ(sta[2], Status::Normal);
  // FCR主探头，FOR辅助探头
  EXPECT_NEAR(pos[3].x, 4521.8, tolerance);
  EXPECT_NEAR(pos[3].y, -940.3, tolerance);
  EXPECT_EQ(sta[3], Status::Normal);
  // FOR主探头
  EXPECT_NEAR(pos[4].x, 4521.8, tolerance);
  EXPECT_NEAR(pos[4].y, -940.3, tolerance);
  EXPECT_EQ(sta[4], Status::Normal);
  // ROL主探头，RCL辅助探头
  EXPECT_NEAR(pos[7].x, -1752.9, tolerance);
  EXPECT_NEAR(pos[7].y, 909.1, tolerance);
  EXPECT_EQ(sta[7], Status::Normal);
  // RCL主探头，RCR辅助探头
  EXPECT_NEAR(pos[8].x, -1871.7, tolerance);
  EXPECT_NEAR(pos[8].y, 0, tolerance);
  EXPECT_EQ(sta[8], Status::Normal);
  // RCR主探头，ROR辅助探头
  EXPECT_NEAR(pos[9].x, -1752.9, tolerance);
  EXPECT_NEAR(pos[9].y, -909.1, tolerance);
  EXPECT_EQ(sta[9], Status::Normal);
  // ROR主探头
  EXPECT_NEAR(pos[10].x, -1752.9, tolerance);
  EXPECT_NEAR(pos[10].y, -909.1, tolerance);
  EXPECT_EQ(sta[10], Status::Normal);
}