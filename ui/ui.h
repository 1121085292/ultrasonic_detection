/*
* 订阅UltrasonicList，可视化空间车位，选择目标车位高亮显示，输出target_parking_spot_info
*/
#pragma once
#include <QtWidgets>
#include <QEvent>

#include "cyber/cyber.h"
#include "ultrasonic_detection/common/parking_spot_gflags.h"
#include "ultrasonic_detection/common_msgs/ultrasonic.pb.h"
#include "ultrasonic_detection/common_msgs/parking_perception.pb.h"

using common_msgs::ultrasonic::UltrasonicList;
using common_msgs::parking::proto::ParkingSpotType;
using common_msgs::hmi::HMI;

class UltrasonicEvent : public QEvent
{
public:
    static const QEvent::Type TYPE = static_cast<QEvent::Type>(QEvent::User + 1);

    UltrasonicEvent(const std::shared_ptr<common_msgs::ultrasonic::UltrasonicList>& data)
        : QEvent(static_cast<QEvent::Type>(TYPE)), data_(data) {}

    std::shared_ptr<common_msgs::ultrasonic::UltrasonicList> data() const { return data_; }

private:
    std::shared_ptr<common_msgs::ultrasonic::UltrasonicList> data_;
};

class MainWindow : public QWidget {
  public:
    MainWindow(QWidget *parent = nullptr) : QWidget(parent){
      setMouseTracking(true);
      // 初始化Cyber RT
      apollo::cyber::Init("hmi");

      // 创建Writer
      auto node = apollo::cyber::CreateNode("hmi");
      hmi_writer_ = node->CreateWriter<HMI>("hmi/");
    }
    void SetQRect(const QVector<QRect>& rectangles) {
      rectangles_ = rectangles;
    };

  protected:
    void paintEvent(QPaintEvent *event) override;

    void mousePressEvent(QMouseEvent *event) override;
    bool event(QEvent* event) override;

  private:
    void updateGUI(const std::shared_ptr<UltrasonicList>& ultra);

    QVector<QRect> rectangles_;

    int spot_id_ = -1; // 初始值为-1,表示没有选中任何矩形
    bool parking_inwards_ = false;

    std::shared_ptr<apollo::cyber::Writer<HMI>> hmi_writer_ = nullptr;
};