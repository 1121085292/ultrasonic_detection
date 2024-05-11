load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library", "cc_test")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "ultrasonic_perception_lib",
    srcs = ["ultrasonic_perception_component.cc"],
    hdrs = ["ultrasonic_perception_component.h"],
    deps = [
        "//cyber",
        "//ultrasonic_detection/base:ultrasonic_orientation",
        "//ultrasonic_detection/common_msgs:echo_list_cc_proto",
        "//ultrasonic_detection/common_msgs:parking_perception_cc_proto",
        "//ultrasonic_detection/common_msgs:ultrasonic_cc_proto",
        "//ultrasonic_detection/proto:ultrasonic_conf_cc_proto",
        "//ultrasonic_detection/ultrasonic:parking_spot_detection",
        "//ultrasonic_detection/ultrasonic:ultrasonic_detection",
        "//ultrasonic_detection/common:min_filter",
        "//ultrasonic_detection/common:parking_spot_gflags",
    ],
    alwayslink = True,
)

cc_binary(
    name = "libultrasonic_perception.so",
    linkshared = True,
    linkstatic = True,
    deps = [
        ":ultrasonic_perception_lib",
    ],
)
