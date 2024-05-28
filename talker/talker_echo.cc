/**
 * @file talker_echo.cc
 * @brief
 * @author tangpeng
 * @version 1.0
 * @date 2024-05-28
 * @copyright Copyright (c) 2024 tangpeng. All rights reserved.
 */
#include "ultrasonic_detection/common_msgs/echo_list.pb.h"

#include "cyber/cyber.h"
#include "cyber/time/clock.h"

using apollo::cyber::Clock;
using common_msgs::echo_list::EchoList;
using common_msgs::error_code::ErrorCode;

void PublishEcho() {
  // 创建节点
  auto talker_node = apollo::cyber::CreateNode("echo");
  // 创建发布者
  auto talker = talker_node->CreateWriter<EchoList>("echo_list");
  // 设置频率
  apollo::cyber::Rate rate(1.0);  // 1Hz
  while (apollo::cyber::OK()) {
    auto msg = std::make_shared<EchoList>();
    // 组织数据
    msg->set_timestamp(Clock::NowInSeconds());
    // sensor_id = 0
    auto echo = msg->add_echo_list();
    echo->set_sensor_id(0);
    echo->add_distances(1000);
    echo->set_error_code(ErrorCode::PERCEPTION_ERROR);
    // 探头数据
    for (int i = 1; i < 12; ++i) {
      auto echo = msg->add_echo_list();
      echo->set_sensor_id(i);
      echo->add_distances(1000);
      echo->set_error_code(ErrorCode::OK);
    }
    // 打印
    AINFO << msg->DebugString();
    // 消息写入通道
    talker->Write(msg);
    rate.Sleep();
  }

  // 等待关闭
  apollo::cyber::WaitForShutdown();
}