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
  }
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
    updateGUI(ultra);
    return true;
  }
  return QWidget::event(event);
}

void MainWindow::updateGUI(const std::shared_ptr<UltrasonicList> &ultra) {
  // set QVector<QRect> rectangles_
  rectangles_.clear();

  auto spots = ultra->parking_spots_info();
  for(const auto& spot : spots.parking_spots()){
    // ParkingSpot类型的spot到QRect类型转换，调用QRect(topleft, size)
    QPoint topleft(spot.spot_left_top_x(), spot.spot_left_top_y());
    QSize qsize;
    if(spot.spot_type() == ParkingSpotType::VERTICAL_PLOT){
      qsize = QSize(25, 52);
    } else if(spot.spot_type() == ParkingSpotType::PARALLEL_PARKING){
      qsize = QSize(52, 25);
    }
    QRect qrect = QRect(topleft, qsize);
    rectangles_.push_back(qrect);
  }

  update();
}