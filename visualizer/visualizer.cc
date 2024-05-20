#include "ultrasonic_detection/ui/visualizer.h"

void ParkingSpotVisualizer::mousePressEvent(QMouseEvent* event) {
  if (itemAt(event->pos())) {
    QGraphicsRectItem* item = static_cast<QGraphicsRectItem*>(itemAt(event->pos()));
    if (selected_rect_) {
      selected_rect_->setPen(QPen(Qt::black));  // Reset previous selected color
    }
    selected_rect_ = item;
    selected_rect_->setPen(QPen(Qt::red));  // Highlight selected color
    // emit ParkingSpotSelected(selected_rect_->rect());
    onParkingSpotSelected(selected_rect_->rect());
  }
}

void ParkingSpotVisualizer::onParkingSpotSelected(const QRectF& rect){
  // 创建并发布选定的车位信息
  selected_spot_ = std::make_shared<TargetParkingSpotInfo>();
  auto target_spot = selected_spot_->mutable_target_parking_spot();

  target_spot->set_spot_left_top_x(rect.topLeft().x());
  target_spot->set_spot_left_top_y(rect.topLeft().y());

  target_spot->set_spot_right_down_x(rect.bottomRight().x());
  target_spot->set_spot_right_down_y(rect.bottomRight().y());

  target_spot->set_spot_left_down_x(rect.bottomLeft().x());
  target_spot->set_spot_left_down_y(rect.bottomLeft().y());

  target_spot->set_spot_right_top_x(rect.topRight().x());
  target_spot->set_spot_right_top_y(rect.topRight().y());
}