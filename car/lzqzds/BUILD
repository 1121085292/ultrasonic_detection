load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library", "cc_test")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "uss_interface",
    srcs = ["uss_interface.cc"],
    hdrs = ["uss_interface.h"],
    copts = ["-std=c++17"],
    deps = [
        "//cyber",
        "//opendbc/can:can_parser",
        "//ultrasonic_detection/common_msgs:echo_list_cc_proto",
    ]
)