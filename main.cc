#include <future>
#include <atomic>

#include "cyber/cyber.h"

#include "ultrasonic_detection/ui/ui.h"

MainWindow* mainWindow;
void callback(const std::shared_ptr<UltrasonicList>& ultra){
  QCoreApplication::postEvent(mainWindow, new UltrasonicEvent(ultra));
}

int main(int argc, char* argv[]){
  QApplication app(argc, argv);
  mainWindow = new MainWindow;

  // 2.初始化；
  apollo::cyber::Init(argv[0]);

  // 安装事件循环附加
    qApp->installEventFilter(nullptr);

  // 3.创建节点；
  auto parking_spot_listener_node = apollo::cyber::CreateNode("parking_spots");
  // 4.创建订阅方；
  auto parking_spot_listener = parking_spot_listener_node->CreateReader<UltrasonicList>("perception/ultrasonic",callback);

  mainWindow->resize(800, 600);
  mainWindow->show();
  // 等待窗口关闭后再退出
  int ret = app.exec();
  // 6.等待关闭。
  apollo::cyber::WaitForShutdown();

  return ret;
}