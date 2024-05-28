/**
 * @file talker.cc
 * @brief
 * @author tangpeng
 * @version 1.0
 * @date 2024-05-28
 * @copyright Copyright (c) 2024 tangpeng. All rights reserved.
 */
#include "cyber/cyber.h"

void PublishEcho();
void PublishLocation();

int main(int argc, char* argv[]) {
  // 初始化
  apollo::cyber::Init(argv[0]);

  std::thread thread1 = std::thread(PublishEcho);
  std::thread thread2 = std::thread(PublishLocation);

  thread1.join();
  thread2.join();

  return 0;
}