#include "car/byd/uss_interface.h"

CANParserPy* USSInterface::createUSSCanParser() {
  std::vector<std::tuple<std::string, int, int>> signals_all;
  std::vector<std::tuple<int, int>> check_all;
  std::vector<std::tuple<std::string, int, int>> signals_A;
  for (int i = 1; i <= 12; ++i) {
    if (i < 7) {
      auto tuple = std::make_tuple("RadarSensorDistance_" + std::to_string(i),
                                   0x28A, 255);
      signals_A.emplace_back(tuple);
    } else {
      auto tuple = std::make_tuple("RadarSensorDistance_" + std::to_string(i),
                                   0x289, 255);
      signals_A.emplace_back(tuple);
    }
  }

  // auto front_PDC_work_sts = std::make_tuple("FrontPDCWorkSts", 0x28A, 255);
  // signals_A.emplace_back(front_PDC_work_sts);
  // auto rear_PDC_work_sts = std::make_tuple("RearPDCWorkSts", 0x289, 255);
  // signals_A.emplace_back(rear_PDC_work_sts);

  signals_all.insert(signals_all.end(), signals_A.begin(), signals_A.end());

  return new CANParserPy("PDC_USS", signals_all, check_all, 1);
}
USSInterface::USSInterface() {
  for (auto& key : RADAR_A_MSGS) {
    valid_cnt_[key] = 0;
  }
  for (int i = RADAR_A_MSGS_start; i <= RADAR_A_MSGS_end; ++i) {
    RADAR_A_MSGS.push_back(i);
  }

  rcp_ = createUSSCanParser();
}
EchoList USSInterface::update(const std::vector<std::string>& can_strings) {
  // 此处解析can_strings logmonotime+candatalist n
  auto tmp = rcp_->update_strings(can_strings);

  updated_messages_.insert(tmp.begin(), tmp.end());

  // if(updated_messages_.find(trigger_msg_) == updated_messages_.end()){
  //   return nullptr;
  // }
  auto echo_list = _update();
  updated_messages_.clear();
  return echo_list;
}
EchoList USSInterface::_update() {
  // // 检查是否有效
  // if (!rcp_->getCanValid()) {
  //   ret->add_errors(common_msgs::radar_data::CANERROR);
  // }
  ACHECK(!updated_messages_.empty());

  EchoList echo_list;
  std::unordered_set<uint32_t> all_updated_messages;
  for (const auto& updated_message : updated_messages_) {
    const std::unordered_set<uint32_t>& second = updated_message.second;
    all_updated_messages.insert(second.begin(), second.end());
  }
  // 遍历更新的消息
  std::map<int, double> distances;
  // bool front_PDC_work_sts = false;
  // bool rear_PDC_work_sts = false;
  for (int ii : all_updated_messages) {
    if (ii >= RADAR_A_MSGS.front() && ii <= RADAR_A_MSGS.back()) {
      // 信号id与信号名之间差一个0x289
      int index = ii - 0x289;  // 此处检索出RADAR_A_MSGS消息
      auto cpt = rcp_->getVlAddress();
      auto cpt_A = cpt[ii];
      if (index == 1) {
        for (int i = 1; i < 7; ++i) {
          double distance = cpt_A["RadarSensorDistance_" + std::to_string(i)];
          distances[i] = distance;
        }
        // front_PDC_work_sts = cpt_A["FrontPDCWorkSts"];
      }
      if (index == 0) {
        for (int i = 7; i < 13; ++i) {
          double distance = cpt_A["RadarSensorDistance_" + std::to_string(i)];
          distances[i] = distance;
        }
        // rear_PDC_work_sts = cpt_A["RearPDCWorkSts"];
      }
    }
  }
  for (const auto& pair : distances) {
    auto echo = echo_list.add_echo();
    echo->set_sensor_id(pair.first);
    echo->set_distance(pair.second * 1e1);
  }
  // writer timestamp
  echo_list.set_timestamp(updated_messages_.begin()->first);
  return echo_list;
}