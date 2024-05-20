#include <QApplication>
#include <QGraphicsScene>
#include <QGraphicsView>
#include <QGraphicsRectItem>
#include <QMouseEvent>

#include "ultrasonic_detection/common_msgs/parking_perception.pb.h"

using common_msgs::parking::proto::TargetParkingSpotInfo;

class ParkingSpotVisualizer : public QGraphicsView {
  Q_OBJECT
  public:
    ParkingSpotVisualizer(QGraphicsScene* scene, QWidget* parent = nullptr)
    : QGraphicsView(scene, parent), selected_rect_(nullptr) {
      setRenderHint(QPainter::Antialiasing);
      setMouseTracking(true);

      // QObject::connect(this, &ParkingSpotVisualizer::ParkingSpotSelected,
      //                 this, &ParkingSpotVisualizer::onParkingSpotSelected);
    }
  std::shared_ptr<TargetParkingSpotInfo> getTargetParkingSpotInfo() const {
    return selected_spot_;
  }
  protected:
    void mousePressEvent(QMouseEvent* event) override;

  // signals:
  //   void ParkingSpotSelected(const QRectF& rect);

  // private slots:
    void onParkingSpotSelected(const QRectF& rect);

  private:
    QGraphicsRectItem* selected_rect_;
    std::shared_ptr<TargetParkingSpotInfo> selected_spot_;
};