#include "ultrasonic.h"

void Ultrasonic::DirectMeasuredPositionCalculate(){
  if(meas_dis_ == ultra_params_.max_range){
    point_ = Point(0.0, 0.0, 0.0);
    SetStatus(Status::OverDetection);
  } else {
    point_.x = coordinate_.x + meas_dis_ * cos(coordinate_.angle);
    if(coordinate_.y < 0){
      point_.y = coordinate_.y - meas_dis_ * sin(coordinate_.angle);
    } else {
      point_.y = coordinate_.y + meas_dis_ * sin(coordinate_.angle);
    }
    SetStatus(Status::Normal);
  }
}

void Ultrasonic::GlobalDirectPositionCalculate(const std::shared_ptr<Pose>& pose){
  if(ultra_status_ != Status::Normal){
    point_global_ = point_;
  } else {
    point_global_.x = pose->position().x()
          + point_.x * cos(pose->heading()) - point_.y * sin(pose->heading());
    point_global_.y = pose.position().y()
          + point_.x * sin(pose->heading()) + point_.y * cos(pose->heading());
  }
}

void Ultrasonic::HoughFilter(const std::queue<Point> &points, LineFitInfo &line){
  if(points.size() )

}
