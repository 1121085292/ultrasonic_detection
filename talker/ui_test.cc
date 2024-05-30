#include <random>

#include "ultrasonic_detection/common_msgs/parking_spots.pb.h"
#include "ultrasonic_detection/common_msgs/ultrasonic.pb.h"

#include "cyber/cyber.h"
#include "cyber/time/clock.h"

using common_msgs::parking::proto::ParkingSpotType;
using common_msgs::ultrasonic::UltrasonicList;

int main(int argc, char* argv[]) {
  std::srand(time(nullptr));
  // 初始化
  apollo::cyber::Init(argv[0]);
  // 创建节点
  auto talker_node = apollo::cyber::CreateNode("ultrasonic");
  // 创建发布者
  auto talker =
      talker_node->CreateWriter<UltrasonicList>("perception/ultrasonic");
  // 设置频率
  apollo::cyber::Rate rate(1.0);  // 1Hz
  while (apollo::cyber::OK()) {
    auto msg = std::make_shared<UltrasonicList>();
    // 组织数据
    msg->set_timestamp(apollo::cyber::Clock::NowInSeconds());

    auto parking_spots_info = msg->mutable_parking_spots_info();
    size_t cnt = rand() % 10 + 1;
    parking_spots_info->set_spots_cnt(cnt);
    for (size_t i = 0; i < cnt; ++i) {
      auto spot = parking_spots_info->add_parking_spots();
      spot->set_spot_left_top_x(200 + i * 60);
      spot->set_spot_left_top_y(200);

      spot->set_spot_left_down_x(200 + i * 60);
      spot->set_spot_left_down_y(720);

      spot->set_spot_right_down_x(450 + i * 60);
      spot->set_spot_right_down_y(720);

      spot->set_spot_right_top_x(450 + i * 60);
      spot->set_spot_right_top_y(200);

      spot->set_spot_type(ParkingSpotType::PARALLEL_PARKING);
    }
    // 打印
    AINFO << msg->DebugString();

    // 消息写入通道
    talker->Write(msg);
    rate.Sleep();
  }
  // 等待关闭
  apollo::cyber::WaitForShutdown();

  return 0;
}