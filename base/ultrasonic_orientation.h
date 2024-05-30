#pragma once
#include <map>

enum class UltrasonicOrientation {
  FRONT_SIDE_LEFT = 0,
  FRONT_OUTER_LEFT = 1,
  FRONT_CENTER_LEFT = 2,
  FRONT_CENTER_RIGHT = 3,
  FRONT_OUTER_RIGHT = 4,
  FRONT_SIDE_RIGHT = 5,

  REAR_SIDE_LEFT = 6,
  REAR_OUTER_LEFT = 7,
  REAR_CENTER_LEFT = 8,
  REAR_CENTER_RIGHT = 9,
  REAR_OUTER_RIGHT = 10,
  REAR_SIDE_RIGHT = 11,
};

const std::map<UltrasonicOrientation, std::string>
    kUltrasonicOrientation2NameMap = {
        {UltrasonicOrientation::FRONT_SIDE_LEFT, "FRONT_SIDE_LEFT"},
        {UltrasonicOrientation::FRONT_OUTER_LEFT, "FRONT_OUTER_LEFT"},
        {UltrasonicOrientation::FRONT_CENTER_LEFT, "FRONT_CENTER_LEFT"},
        {UltrasonicOrientation::FRONT_CENTER_RIGHT, "FRONT_CENTER_RIGHT"},
        {UltrasonicOrientation::FRONT_OUTER_RIGHT, "FRONT_OUTER_RIGHT"},
        {UltrasonicOrientation::FRONT_SIDE_RIGHT, "FRONT_SIDE_RIGHT"},

        {UltrasonicOrientation::REAR_SIDE_LEFT, "REAR_SIDE_LEFT"},
        {UltrasonicOrientation::REAR_OUTER_LEFT, "REAR_OUTER_LEFT"},
        {UltrasonicOrientation::REAR_CENTER_LEFT, "REAR_CENTER_LEFT"},
        {UltrasonicOrientation::REAR_CENTER_RIGHT, "REAR_CENTER_RIGHT"},
        {UltrasonicOrientation::REAR_OUTER_RIGHT, "REAR_OUTER_RIGHT"},
        {UltrasonicOrientation::REAR_SIDE_RIGHT, "REAR_SIDE_RIGHT"},
};