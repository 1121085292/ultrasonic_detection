#include "ultrasonic_detection/ui/ui.h"

void MainWindow::paintEvent(QPaintEvent *event) {
  QPainter painter(this);
  // 遍历所有矩形框
  for (int i = 0; i < rectangles_.size(); ++i) {
    const QRect& rect = rectangles_[i];

    // 设置画笔颜色
    if (i == spot_id_) {
      // 如果是选中的矩形框,使用红色
      painter.setPen(Qt::red);
    } else {
      // 其他矩形框使用绿色
      painter.setPen(Qt::green);
    }
    // 绘制矩形框
    painter.drawRect(rect);
    // 绘制字母P在矩形框的中心
    painter.setPen(Qt::black);  // 设置文本颜色
    QFont font = painter.font();
    font.setPointSize(20);  // 设置字体大小
    painter.setFont(font);

    QRect textRect = rect;  // 以矩形框为基础计算文本位置
    painter.drawText(textRect, Qt::AlignCenter, "P");
  }
  // 绘制自车
  painter.setPen(Qt::blue);
  painter.drawRect(car_ego_);
}

void MainWindow::mousePressEvent(QMouseEvent *event) {
  // 在鼠标点击时获取所选矩形框的 ID 信息
  for (int i = 0; i < rectangles_.size(); ++i) {
    if (rectangles_[i].contains(event->pos())) {
      // 设置 GFLAGS_id 的值为所选矩形框的 ID 信息
      spot_id_ = i;
      qDebug() << "Selected Spot ID:" << spot_id_;

      auto msg = std::make_shared<HMI>();
      msg->set_spot_id(spot_id_);

      hmi_writer_->Write(msg);

      break;
    }
  }
  repaint(); // 立即触发重绘
}

bool MainWindow::event(QEvent *event) { 
  if (event->type() == UltrasonicEvent::TYPE) {
    const auto ultrasonicEvent = static_cast<UltrasonicEvent*>(event);
    const auto& ultra = ultrasonicEvent->data();
    // 在此更新GUI，处理ultra消息
    updateUltra(ultra);
    return true;
  }
  if (event->type() == LocationEvent::TYPE) {
    const auto locationEvent = static_cast<LocationEvent*>(event);
    const auto& location = locationEvent->data();
    // 在此更新GUI，处理location消息
    updateLocation(location);
    return true;
  }
  return QWidget::event(event);
}

void MainWindow::updateUltra(const std::shared_ptr<UltrasonicList> &ultra) {
  // set QVector<QRect> rectangles_
  rectangles_.clear();

  auto spots = ultra->parking_spots_info();
  for(const auto& spot : spots.parking_spots()){
    // ParkingSpot类型的spot到QRect类型转换，调用QRect(topleft, bottomright)
    QPoint topleft(spot.spot_left_top_x(), spot.spot_left_top_y());
    QPoint bottomright(spot.spot_right_down_x(), spot.spot_right_down_y());
    QRect qrect = QRect(topleft, bottomright);
    rectangles_.push_back(qrect);
  }
  update();
}

void MainWindow::updateLocation(const std::shared_ptr<InsLocation>& location) {
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

  car_ego_ = QRect(QPoint(topleft_x, topleft_y), QPoint(bottomright_x, bottomright_y));
  
  update();
}