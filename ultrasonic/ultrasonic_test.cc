#include "ultrasonic.h"
#include "ultrasonic_detection/proto/ultrasonic_coordinate.pb.h"

#include "gtest/gtest.h"

using ultrasonic_detection::proto::UltrasonicCoordinate;

class UltrasonicTest : public testing::Test {
  public:
    UltrasonicTest() {
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

      int dis0 = 6000;
      distances_.emplace_back(dis0);
      for(int i = 1; i < 12; ++i){
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

TEST_F(UltrasonicTest, DirectMeasuredPositionCalculateTest){
  std::vector<Point2D> pos;
  std::vector<Status> sta;
  for(int i = 0; i < 12; ++i){
    Ultrasonic ultra(coordinate_map_[i], distances_[i]);
    ultra.DirectMeasuredPositionCalculate();
    Point2D position = ultra.GetPosition();
    pos.emplace_back(position);
    Status status = ultra.GetStatus();
    sta.emplace_back(status);
  }
  double tolerance = 9 * 1e-1;
  // FSL未检测到障碍物
  EXPECT_NEAR(pos[0].x, 0, tolerance);
  EXPECT_NEAR(pos[0].y, 0, tolerance);
  EXPECT_EQ(sta[0], Status::OverDetection);
  // FOL
  EXPECT_NEAR(pos[1].x, 4330.1, tolerance);
  EXPECT_NEAR(pos[1].y, 1338.4, tolerance);
  EXPECT_EQ(sta[1], Status::Normal);
  // FCL
  EXPECT_NEAR(pos[2].x, 4693, tolerance);
  EXPECT_NEAR(pos[2].y, 519.4, tolerance);
  EXPECT_EQ(sta[2], Status::Normal);
  // FCR
  EXPECT_NEAR(pos[3].x, 4693, tolerance);
  EXPECT_NEAR(pos[3].y, -519.4, tolerance);
  EXPECT_EQ(sta[3], Status::Normal);
  // FOR
  EXPECT_NEAR(pos[4].x, 4330.1, tolerance);
  EXPECT_NEAR(pos[4].y, -1338.4, tolerance);
  EXPECT_EQ(sta[4], Status::Normal);
  // FSR
  EXPECT_NEAR(pos[5].x, 3361.4, tolerance);
  EXPECT_NEAR(pos[5].y, -1913.4, tolerance);
  EXPECT_EQ(sta[5], Status::Normal);
  // RSL
  EXPECT_NEAR(pos[6].x, -530.4, tolerance);
  EXPECT_NEAR(pos[6].y, 1906.3, tolerance);
  EXPECT_EQ(sta[6], Status::Normal);
  // ROL
  EXPECT_NEAR(pos[7].x, -1511.6, tolerance);
  EXPECT_NEAR(pos[7].y, 1372.6, tolerance);
  EXPECT_EQ(sta[7], Status::Normal);
  // RCL
  EXPECT_NEAR(pos[8].x, -1926.9, tolerance);
  EXPECT_NEAR(pos[8].y, 430.6, tolerance);
  EXPECT_EQ(sta[8], Status::Normal);
  // RCR
  EXPECT_NEAR(pos[9].x, -1926.9, tolerance);
  EXPECT_NEAR(pos[9].y, -430.6, tolerance);
  EXPECT_EQ(sta[9], Status::Normal);
  // ROR
  EXPECT_NEAR(pos[10].x, -1511.6, tolerance);
  EXPECT_NEAR(pos[10].y, -1372.6, tolerance);
  EXPECT_EQ(sta[10], Status::Normal);
  // RSR
  EXPECT_NEAR(pos[11].x, -530.4, tolerance);
  EXPECT_NEAR(pos[11].y, -1906.3, tolerance);
  EXPECT_EQ(sta[11], Status::Normal);
}

TEST_F(UltrasonicTest, GlobalDirectPositionCalculateTest){
  Pose pose;
  pose.mutable_position()->set_x(1080 * sin(45 * M_PI / 180));
  pose.mutable_position()->set_y(1080 * cos(45 * M_PI / 180));
  pose.set_heading(45 * M_PI / 180);

  std::vector<Point2D> pos;
  std::vector<Status> sta;
  for(int i = 0; i < 12; ++i){
    Ultrasonic ultra(coordinate_map_[i], distances_[i]);
    ultra.DirectMeasuredPositionCalculate();
    ultra.GlobalDirectPositionCalculate(pose);
    Point2D position_global = ultra.GetGlobalPosition();
    pos.emplace_back(position_global);
    Status status = ultra.GetStatus();
    sta.emplace_back(status);
  }
  double tolerance = 9 * 1e-1;
  // FSL未检测到障碍物
  EXPECT_NEAR(pos[0].x, 0, tolerance);
  EXPECT_NEAR(pos[0].y, 0, tolerance);
  EXPECT_EQ(sta[0], Status::OverDetection);
  // FOL
  EXPECT_NEAR(pos[1].x, 2879.1, tolerance);
  EXPECT_NEAR(pos[1].y, 4771.9, tolerance);
  EXPECT_EQ(sta[1], Status::Normal);
  // FCL
  EXPECT_NEAR(pos[2].x, 3714.8, tolerance);
  EXPECT_NEAR(pos[2].y, 4449.4, tolerance);
  EXPECT_EQ(sta[2], Status::Normal);
  // FCR
  EXPECT_NEAR(pos[3].x, 4449.4, tolerance);
  EXPECT_NEAR(pos[3].y, 3714.8, tolerance);
  EXPECT_EQ(sta[3], Status::Normal);
  // FOR
  EXPECT_NEAR(pos[4].x, 4771.9, tolerance);
  EXPECT_NEAR(pos[4].y, 2879.1, tolerance);
  EXPECT_EQ(sta[4], Status::Normal);
  // FSR
  EXPECT_NEAR(pos[5].x, 4493.5, tolerance);
  EXPECT_NEAR(pos[5].y, 1787.6, tolerance);
  EXPECT_EQ(sta[5], Status::Normal);
  // RSL
  EXPECT_NEAR(pos[6].x, -959.3, tolerance);
  EXPECT_NEAR(pos[6].y, 1736.6, tolerance);
  EXPECT_EQ(sta[6], Status::Normal);
  // ROL
  EXPECT_NEAR(pos[7].x, -1275.8, tolerance);
  EXPECT_NEAR(pos[7].y, 665.4, tolerance);
  EXPECT_EQ(sta[7], Status::Normal);
  // RCL
  EXPECT_NEAR(pos[8].x, -903.4, tolerance);
  EXPECT_NEAR(pos[8].y, -294.4, tolerance);
  EXPECT_EQ(sta[8], Status::Normal);
  // RCR
  EXPECT_NEAR(pos[9].x, -294.4, tolerance);
  EXPECT_NEAR(pos[9].y, -903.4, tolerance);
  EXPECT_EQ(sta[9], Status::Normal);
  // ROR
  EXPECT_NEAR(pos[10].x, 665.4, tolerance);
  EXPECT_NEAR(pos[10].y, -1275.8, tolerance);
  EXPECT_EQ(sta[10], Status::Normal);
  // RSR
  EXPECT_NEAR(pos[11].x, 1736.6, tolerance);
  EXPECT_NEAR(pos[11].y, -959.3, tolerance);
  EXPECT_EQ(sta[11], Status::Normal);
}