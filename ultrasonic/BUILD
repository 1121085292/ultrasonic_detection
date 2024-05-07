load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test", "cc_binary")

package(default_visibility = ["//visibility:public"])

cc_library(
  name = "ultrasonic_detection",
  srcs = ["ultrasonic_detection.cc"],
  hdrs = ["ultrasonic_detection.h"],
  deps = [
    ":ultrasonic",
  ]
)

cc_test(
  name = "ultrasonic_detection_test",
  size = "small",
  srcs = ["ultrasonic_detection_test.cc"],
  deps = [
    ":ultrasonic_detection",
    "//ultrasonic_detection/proto:ultrasonic_conf_cc_proto",
    "@com_google_googletest//:gtest_main"
  ]
)

cc_library(
  name = "ultrasonic",
  srcs = ["ultrasonic.cc"],
  hdrs = ["ultrasonic.h"],
  deps = [
    "//ultrasonic_detection/common_msgs:ultrasonic_cc_proto",
    "//ultrasonic_detection/common_msgs:InsLocation_cc_proto",
    "//ultrasonic_detection/base:point",
    "//ultrasonic_detection/base:params",
  ]
)

cc_test(
  name = "ultrasonic_test",
  size = "small",
  srcs = ["ultrasonic_test.cc"],
  deps = [
    ":ultrasonic",
    "//ultrasonic_detection/proto:ultrasonic_conf_cc_proto",
    "@com_google_googletest//:gtest_main"
  ]
)

cc_library(
  name = "parking_spot_detection",
  srcs = ["parking_spot_detection.cc"],
  hdrs = ["parking_spot_detection.h"],
  deps = [
    "//cyber",
    "//ultrasonic_detection/common:line_segment",
    "//ultrasonic_detection/common_msgs:InsLocation_cc_proto"
  ]
)