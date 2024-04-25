load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test", "cc_binary")

package(default_visibility = ["//visibility:public"])

cc_library(
  name = "line_segment",
  hdrs = ["line_segment.h"],
  srcs = ["line_segment.cc"],
  deps = [
    "//ultrasonic_detection/base:point",
    "//ultrasonic_detection/base:params",
  ]
)

cc_binary(
  name = "line_segment_test",
  srcs = ["line_segment_test.cc"],
  deps = [
    ":line_segment",
    "@opencv//:core",
    "@opencv//:highgui"
  ]
)