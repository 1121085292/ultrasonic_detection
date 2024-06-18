#pragma once

#include "opendbc/can/common.h"

#include "ultrasonic_detection/common_msgs/echo_list.pb.h"

#include "cyber/cyber.h"
using uss::common_msgs::Echo;
using uss::common_msgs::EchoList;

const int RADAR_A_MSGS_start = 0x289;
const int RADAR_A_MSGS_end = 0x28A;

class USSInterface {
 public:
  USSInterface();
  ~USSInterface() { delete rcp_; }
  EchoList update(const std::vector<std::string>& can_strings);

  EchoList _update();

  CANParserPy* createUSSCanParser();

 private:
  std::unordered_map<int, int> valid_cnt_;
  std::map<size_t, std::unordered_set<uint32_t>> updated_messages_;
  CANParserPy* rcp_;
  std::vector<int> RADAR_A_MSGS;
};