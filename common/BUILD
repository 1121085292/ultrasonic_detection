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

cc_library(
  name = "min_filter",
  srcs = ["min_filter.cc"],
  hdrs = ["min_filter.h"],
)

cc_test(
  name = "min_filter_test",
  size = "small",
  srcs = ["min_filter_test.cc"],
  deps = [
    ":min_filter",
    "@com_google_googletest//:gtest_main"
  ]
)

cc_library(
  name = "parking_spot_gflags",
  hdrs = ["parking_spot_gflags.h"],
  srcs = ["parking_spot_gflags.cc"],
  deps = [
    "@com_github_gflags_gflags//:gflags"
  ]
)