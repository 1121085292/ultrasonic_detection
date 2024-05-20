#include "ultrasonic_detection/target_spot_component.h"

TargetSpotComponent::~TargetSpotComponent() {
  delete view_;
  delete scene_;
  delete app_;
}

bool TargetSpotComponent::Init() {
  int argc = 0;
  char** argv = nullptr;
  app_ = new QApplication(argc, argv);
  scene_ = new QGraphicsScene();
  view_ = new ParkingSpotVisualizer(scene_);
  view_->setScene(scene_);

  view_->show();

  target_spot_writer_ = node_->CreateWriter<TargetParkingSpotInfo>("perception/ultrasonic/parking_spot");
  return true;
}
bool TargetSpotComponent::Proc(const std::shared_ptr<UltrasonicList>& ultra,
                               const std::shared_ptr<UltrasonicList>& location) {
    // 清理场景
    scene_->clear();
    // 获取车位矩形框
    auto spots = ultra->parking_spots_info();
    // 保存车位矩形框
    for(const auto& spot : spots.parking_spots()){
      // ParkingSpot类型的spot到QRect类型转换，调用QRect(topleft, bottomright)
      QPoint topleft(spot.spot_left_top_x(), spot.spot_left_top_y());
      QPoint bottomright(spot.spot_right_down_x(), spot.spot_right_down_y());
      QRect qrect = QRect(topleft, bottomright);
      rectangles_.push_back(qrect);
    }
    // 绘制车位矩形框
    for (const auto& parking_spot : rectangles_) {
      auto rect = new QGraphicsRectItem(parking_spot.x(), parking_spot.y(), parking_spot.width(), parking_spot.height());
      rect->setPen(QPen(Qt::green));
      scene_->addItem(rect);
    }
    // 自车位置,后轴中心
    auto pose = location->pose();
    double x = pose.position().x();
    double y = pose.position().y();
    // 航向角
    double heading = pose.heading();
    // topleft
    double topleft_x = x + car_width / 2 * cos(heading) - car_length / 2 * sin(heading);
    double topleft_y = y + car_width / 2 * sin(heading) + car_length / 2 * cos(heading);
    // bottomright
    double bottomright_x = x - car_width / 2 * cos(heading) + car_length / 2 * sin(heading);
    double bottomright_y = y - car_width / 2 * sin(heading) - car_length / 2 * cos(heading);
    // 绘制自车矩形
    auto ego_rect = new QGraphicsRectItem(topleft_x, topleft_y, bottomright_x - topleft_x, bottomright_y - topleft_y);
    ego_rect->setPen(QPen(Qt::blue));
    scene_->addItem(ego_rect);
    // 刷新显示
    app_->processEvents();

    // 发送目标车位信息
    auto target_spot = view_->getTargetParkingSpotInfo();
    if (target_spot != nullptr) {
      target_spot_writer_->Write(target_spot);
    }

    return true;
}