load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test", "cc_binary")
load("//third_party/qt5:qt.bzl", "qt_cc_library")

package(default_visibility = ["//visibility:public"])

cc_library(
  name = "ui",
  copts = [
        "-Iexternal/qt",
    ],
  deps = [
    ":ui_lib"
  ]
)

qt_cc_library(
    name = "ui_lib",
    srcs = glob(
        ["*.cc"],
    ),
    hdrs = glob([
        "*.h",
    ]),
    copts = [
        "-Iexternal/qt",
    ],
    deps = [
      "//cyber",
      "//ultrasonic_detection/common:parking_spot_gflags",
      "//ultrasonic_detection/common_msgs:parking_perception_cc_proto",
      "//ultrasonic_detection/common_msgs:ultrasonic_cc_proto",
      "//ultrasonic_detection/common_msgs:hmi_cc_proto",
      "@qt//:qt_core",
      "@qt//:qt_gui",
      "@qt//:qt_opengl",
      "@qt//:qt_widgets",
    ],
)
