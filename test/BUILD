load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test", "cc_binary")

package(default_visibility = ["//visibility:public"])

cc_binary(
  name = "fit_line_segment_test",
  srcs = ["fit_line_segment_test.cc"],
  deps = [
    "//cyber",
    "//ultrasonic_detection/common_msgs:ultrasonic_cc_proto",
    "//ultrasonic_detection/common:line_segment",
    "@opencv//:core",
    "@opencv//:highgui"
  ]
)

cc_binary(
  name = "parking_spot_test",
  srcs = ["parking_spot_test.cc"],
  deps = [
    "//ultrasonic_detection/base:point",
    "@opencv//:core",
    "@opencv//:highgui"
  ]
)