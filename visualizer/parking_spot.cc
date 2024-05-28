#include "ultrasonic_detection/ui/ui.h"

#include "cyber/cyber.h"

MainWindow* mainWindow;

void MessageCallback1(const std::shared_ptr<UltrasonicList>& ultra) {
  QCoreApplication::postEvent(mainWindow, new UltrasonicEvent(ultra));
}

void MessageCallback2(const std::shared_ptr<InsLocation>& ultra) {
  QCoreApplication::postEvent(mainWindow, new LocationEvent(ultra));
}

int main(int argc, char* argv[]) {
  QApplication app(argc, argv);
  mainWindow = new MainWindow;

  // 2.初始化；
  apollo::cyber::Init(argv[0]);

  // 安装事件循环附加
  qApp->installEventFilter(nullptr);

  // 3.创建节点；
  auto listener_node = apollo::cyber::CreateNode("parking");
  // 4.创建订阅方；
  auto parking_spot_listener = listener_node->CreateReader<UltrasonicList>(
      "perception/ultrasonic",
      std::bind(MessageCallback1, std::placeholders::_1));

  auto ego_pose_listener = listener_node->CreateReader<InsLocation>(
      "location", std::bind(MessageCallback2, std::placeholders::_1));

  mainWindow->resize(800, 600);
  mainWindow->show();
  // 等待窗口关闭后再退出
  int ret = app.exec();
  // 6.等待关闭。
  apollo::cyber::WaitForShutdown();
  delete mainWindow;
  return ret;
}