load("@rules_proto//proto:defs.bzl", "proto_library")
load("@rules_cc//cc:defs.bzl", "cc_proto_library")

package(default_visibility = ["//visibility:public"])

cc_proto_library(
  name = "InsLocation_cc_proto",
  deps = [
      ":InsLocation_proto",
  ],
)

proto_library(
  name = "InsLocation_proto",
  srcs = ["InsLocation.proto"]
)

cc_proto_library(
  name = "parking_spots_cc_proto",
  deps = [
      ":parking_spots_proto",
  ],
)

proto_library(
  name = "parking_spots_proto",
  srcs = ["parking_spots.proto"],
  deps = [
    ":hmi_proto"
  ]
)

cc_proto_library(
  name = "error_code_cc_proto",
  deps = [
      ":error_code_proto",
  ],
)

proto_library(
  name = "error_code_proto",
  srcs = ["error_code.proto"]
)

cc_proto_library(
  name = "echo_list_cc_proto",
  deps = [
      ":echo_list_proto",
  ],
)

proto_library(
  name = "echo_list_proto",
  srcs = ["echo_list.proto"],
  deps = [
    ":error_code_proto"
  ]
)

cc_proto_library(
  name = "ultrasonic_cc_proto",
  deps = [
      ":ultrasonic_proto",
  ],
)

proto_library(
  name = "ultrasonic_proto",
  srcs = ["ultrasonic.proto"],
  deps = [
    ":error_code_proto",
    ":parking_spots_proto"
  ]
)

cc_proto_library(
  name = "hmi_cc_proto",
  deps = [
      ":hmi_proto",
  ],
)

proto_library(
  name = "hmi_proto",
  srcs = ["hmi.proto"],
)