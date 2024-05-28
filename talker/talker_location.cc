/**
 * @file talker_location.cc
 * @brief
 * @author tangpeng
 * @version 1.0
 * @date 2024-05-28
 * @copyright Copyright (c) 2024 tangpeng. All rights reserved.
 */
#include "ultrasonic_detection/common_msgs/InsLocation.pb.h"

#include "cyber/cyber.h"
#include "cyber/time/clock.h"

using apollo::cyber::Clock;
using common_msgs::InsLocation::InsLocation;

void PublishLocation() {
  // 创建节点
  auto talker_node = apollo::cyber::CreateNode("location");
  // 创建发布者
  auto talker = talker_node->CreateWriter<InsLocation>("location");
  // 设置频率
  apollo::cyber::Rate rate(1.0);  // 1Hz
  while (apollo::cyber::OK()) {
    auto msg = std::make_shared<InsLocation>();
    // 组织数据
    msg->mutable_header()->set_timestamp_sec(Clock::NowInSeconds());
    msg->mutable_pose()->set_heading(45 * M_PI / 180);
    msg->mutable_pose()->mutable_position()->set_x(1080 * sin(45 * M_PI / 180));
    msg->mutable_pose()->mutable_position()->set_y(1080 * cos(45 * M_PI / 180));

    // 打印
    AINFO << msg->DebugString();
    // 消息写入通道
    talker->Write(msg);
    rate.Sleep();
  }

  // 等待关闭
  apollo::cyber::WaitForShutdown();
}