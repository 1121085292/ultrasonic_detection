/*
 * 订阅UltrasonicList，可视化空间车位，选择目标车位高亮显示，输出target_parking_spot_info
 */
#pragma once
#include <QEvent>
#include <QtWidgets>

#include "ultrasonic_detection/common_msgs/InsLocation.pb.h"
#include "ultrasonic_detection/common_msgs/parking_spots.pb.h"
#include "ultrasonic_detection/common_msgs/ultrasonic.pb.h"

#include "cyber/cyber.h"

using common_msgs::InsLocation::InsLocation;
using Pose = common_msgs::InsLocation::InsLocation_Pose;
using uss::common_msgs::HMI;
using uss::common_msgs::ParkingSpotType;
using uss::common_msgs::UltrasonicList;

const int car_width = 48;
const int car_length = 19;

class UltrasonicEvent : public QEvent {
 public:
  static const QEvent::Type TYPE = static_cast<QEvent::Type>(QEvent::User + 1);

  UltrasonicEvent(const std::shared_ptr<UltrasonicList>& data)
      : QEvent(static_cast<QEvent::Type>(TYPE)), data_(data) {}

  std::shared_ptr<UltrasonicList> data() const { return data_; }

 private:
  std::shared_ptr<UltrasonicList> data_;
};

class LocationEvent : public QEvent {
 public:
  static const QEvent::Type TYPE = static_cast<QEvent::Type>(QEvent::User + 2);

  LocationEvent(const std::shared_ptr<InsLocation>& data)
      : QEvent(static_cast<QEvent::Type>(TYPE)), data_(data) {}

  std::shared_ptr<InsLocation> data() const { return data_; }

 private:
  std::shared_ptr<InsLocation> data_;
};

class MainWindow : public QWidget {
 public:
  MainWindow(QWidget* parent = nullptr) : QWidget(parent) {
    setMouseTracking(true);
    // 初始化Cyber RT
    apollo::cyber::Init("hmi");

    // 创建Writer
    auto node = apollo::cyber::CreateNode("hmi");
    hmi_writer_ = node->CreateWriter<HMI>("hmi/");
  }
  void SetQRect(const QVector<QRect>& rectangles) { rectangles_ = rectangles; };

 protected:
  void paintEvent(QPaintEvent* event) override;

  void mousePressEvent(QMouseEvent* event) override;
  bool event(QEvent* event) override;

 private:
  void updateUltra(const std::shared_ptr<UltrasonicList>& ultra);

  void updateLocation(const std::shared_ptr<InsLocation>& location);

  QVector<QRect> rectangles_;

  int spot_id_ = -1;  // 初始值为-1,表示没有选中任何矩形
  bool parking_inwards_ = false;

  std::shared_ptr<apollo::cyber::Writer<HMI>> hmi_writer_ = nullptr;

  // ego
  QPoint ego_point_;
  double ego_heading_;
  QRect car_ego_;
};