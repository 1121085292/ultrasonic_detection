#include "ultrasonic_detection.h"

void UltrasonicDetection::TriangleMeasuredPositionCalculate() {
  int ul_dis = ul_.GetMeasDistance();
  int ur_dis = ur_.GetMeasDistance();
  Point3D ul_pos = ul_.GetUltraCoordinate();
  Point3D ur_pos = ur_.GetUltraCoordinate();

  if (ul_dis == ultra_params_.max_range || ur_dis == ultra_params_.max_range) {
    point_ = Point2D(0.0, 0.0);
    ul_.SetStatus(Status::OverDetection);
  } else {
    double bottom_edge = (ul_pos - ur_pos).Length();
    if ((ul_dis + ur_dis > bottom_edge) &&
        fabs(ul_dis - ur_dis < bottom_edge)) {
      // 计算α角
      double alpha = std::atan2((ur_pos.y - ul_pos.y), (ur_pos.x - ul_pos.x));
      // 计算β角
      double beta;
      if (ul_pos.x < 0) {
        beta = -acos(
            (ul_dis * ul_dis + bottom_edge * bottom_edge - ur_dis * ur_dis) /
            (2 * ul_dis * bottom_edge));
      } else {
        beta = acos(
            (ul_dis * ul_dis + bottom_edge * bottom_edge - ur_dis * ur_dis) /
            (2 * ul_dis * bottom_edge));
      }
      // 计算坐标
      point_.x = ul_pos.x + ul_dis * cos(alpha + beta);
      point_.y = ul_pos.y + ul_dis * sin(alpha + beta);
      ul_.SetStatus(Status::Normal);
    } else {
      point_ = Point2D(ul_.GetUltraCoordinate().x, ul_.GetUltraCoordinate().y);
      ul_.SetStatus(Status::InvalidPoint);
    }
  }
}

void UltrasonicDetection::GlobalTrianglePositionCalculate(
    const std::shared_ptr<Pose>& pose) {
  if (ul_.GetStatus() != Status::Normal) {
    point_global_ = point_;
  } else {
    point_global_.x = pose->position().x() + point_.x * cos(pose->heading()) -
                      point_.y * sin(pose->heading());
    point_global_.y = pose->position().y() + point_.x * sin(pose->heading()) +
                      point_.y * cos(pose->heading());
  }
}