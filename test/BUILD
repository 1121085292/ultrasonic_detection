load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test", "cc_binary")

package(default_visibility = ["//visibility:public"])

cc_binary(
  name = "fit_line_segment_test",
  srcs = ["fit_line_segment_test.cc"],
  deps = [
    "//ultrasonic_detection/common:line_segment",
    "@opencv//:core",
    "@opencv//:highgui"
  ]
)