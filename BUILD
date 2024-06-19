load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library", "cc_test")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "ultrasonic_perception_lib",
    srcs = ["ultrasonic_perception_component.cc"],
    hdrs = ["ultrasonic_perception_component.h"],
    deps = [
        "//ultrasonic_detection/base:ultrasonic_orientation",
        "//ultrasonic_detection/common_msgs:ultrasonic_cc_proto",
        "//ultrasonic_detection/common_msgs:hmi_cc_proto", 
        "//common_msgs/car:can_data_cc_proto", 
        "//ultrasonic_detection/proto:ultrasonic_conf_cc_proto",
        "//ultrasonic_detection/ultrasonic:parking_spot_detection",
        "//ultrasonic_detection/ultrasonic:ultrasonic_detection",
        "//car/lzqzds:uss_interface",
        "//opendbc/can/dbc_out:PDC_USS",
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