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
    "@com_google_googletest//:gtest_main"
  ]
)

cc_library(
  name = "ultrasonic",
  srcs = ["ultrasonic.cc"],
  hdrs = ["ultrasonic.h"],
  deps = [
    "//ultrasonic_detection/proto:ultrasonic_coordinate_cc_proto",
    "//ultrasonic_detection/common_msgs:ultrasonic_cc_proto",
    "//ultrasonic_detection/common_msgs:InsLocation_cc_proto",
    "//ultrasonic_detection/min_filter:min_filter",
    "//ultrasonic_detection/base:params",
  ]
)

cc_test(
  name = "ultrasonic_test",
  size = "small",
  srcs = ["ultrasonic_test.cc"],
  deps = [
    ":ultrasonic",
    "@com_google_googletest//:gtest_main"
  ]
)