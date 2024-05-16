load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test", "cc_binary")

package(default_visibility = ["//visibility:public"])

cc_library(
  name = "talker_echo",
  srcs = ["talker_echo.cc"],
  deps = [
    "//cyber",
    "//ultrasonic_detection/common_msgs:echo_list_cc_proto",
  ]
)

cc_library(
  name = "talker_location",
  srcs = ["talker_location.cc"],
  deps = [
    "//cyber",
    "//ultrasonic_detection/common_msgs:InsLocation_cc_proto",
  ]
)

cc_binary(
  name = "talker",
  srcs = ["talker.cc"],
  deps = [
    ":talker_echo",
    ":talker_location"
  ]
)

cc_binary(
  name = "ui_test",
  srcs = ["ui_test.cc"],
  deps = [
    "//cyber",
    "//ultrasonic_detection/common_msgs:ultrasonic_cc_proto",
    "//ultrasonic_detection/common_msgs:parking_perception_cc_proto",
  ]
)