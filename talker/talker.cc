#include "cyber/cyber.h"

void PublishEcho();
void PublishLocation();

int main(int argc, char* argv[]){
  // 初始化
  apollo::cyber::Init(argv[0]);

  std::thread thread1 = std::thread(PublishEcho);
  std::thread thread2 = std::thread(PublishLocation);

  thread1.join();
  thread2.join();

  return 0;
}