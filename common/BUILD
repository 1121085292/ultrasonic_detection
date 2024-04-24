load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test", "cc_binary")

package(default_visibility = ["//visibility:public"])

cc_library(
  name = "cluster",
  hdrs = ["cluster.h"],
  srcs = ["cluster.cc"],
  deps = [
    "//ultrasonic_detection/base:point",
    "//ultrasonic_detection/base:params",
  ]
)

cc_library(
  name = "line_segment",
  hdrs = ["line_segment.h"],
  srcs = ["line_segment.cc"],
  deps = [
    ":cluster",
  ]
)