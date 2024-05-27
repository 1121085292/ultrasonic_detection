<h1 align="center">Ultrasonic Detection</h1>

## Table of Contents

- [Table of Contents](#table-of-contents)
- [Introduction](#introduction)
- [Architecture](#architecture)
- [running the code](#running-the-code)
- [line segment fitting algorithm test](#line-segment-fitting-algorithm-test)

## Introduction

Implement distance measurement and parking spot recognition using ultrasonic based on the [Apollo](https://github.com/ApolloAuto/apollo) open-source Cyber RT framework.Modules inputs are [ultrasonic signals](common_msgs/echoList.proto) and [location information](common_msgs/InsLocation.proto),outputs are [obstacal position](common_msgs/ultrasonic.proto) and [parking spot infomation](common_msgs/parking_spots.proto)

## Architecture

- **Hardware**

<div align="center"><img src=docs/image/hardware.png alt="hardware" width="600"></div>

- **Software**

<div align="center"><img src=docs/image/software.png alt="software" width="600"></div>

## running the code

1. clone the repository and copy the `ultrasonic_detection` folder to the `apollo-master` folder.
2. install the required third-party libraries by running the following command in the `apollo-master` folder:

    ```bash
    bash docker/scripts/install_opencv.sh
    bash docker/scripts/install_qt.sh
    ```

3. run the following command in the `apollo-master` folder:

    ```bash
    bash run_ultra_perception.sh
    ```

    open another terminal and run the following command to start the GUI:

    ```bash
    bash run_visualizer.sh
    ```

4. open a new terminal and run the following command to start the Cyber RT monitor:

    ```bash
    cyber_monitor
    ```

    <div align="center">channels infomation</div>
    <div align="center"><img src=docs/image/channels.png alt="channels infomation"></div>
    <div align="center">parking spots infomation</div>
    <div align="center"><img src=docs/image/parking_spots_info.png alt="parking spots infomation"></div>
5. use your mouse to choose the target parking spot on the Qt display.And then, you can see the target parking spot's infomation on the cyber_monitor terminal.
    <div align="center">parking spots</div>
    <div align="center"><img src=docs/image/parking_spots.png alt="parking spots" width="600"></div>
    <div align="center">target spot</div>
    <div align="center"><img src=docs/image/target_spot.png alt="target spot" width="600"></div>

## line segment fitting algorithm test

The line segment fitting algorithm is used to boundary fitting. The algorithm introduces the LT(Line Tracking) method into the IEPF(Iterative End-Point Fitting) method to make the line segment extraction method better achieve adaptive fitting and it takes in the boundary points and returns a line segment.The algorithm is implemented in the `common/line_segment.h` file.You can run the following command to test the line segment fitting algorithm:

```bash
bash run_line_segment_test.sh
```

If you want to test the algorithm with your own data, you can modify the `test/fit_line_segment_test.cc` file.The `test/fit_line_segment_test.cc` file uses a listener to subscribe to channel messages to get the boundary points,you can to write a talker node to publish `perception/ultrasonic` channel message or use other methods to get the boundary points.
<div align="center">listener node</div>

```c++
vector<Point2D> points;
auto listener_node = apollo::cyber::CreateNode("points");
auto point_listener = listener_node->CreateReader<UltrasonicList>(
      "perception/ultrasonic", [&points](const std::shared_ptr<UltrasonicList>& ultra){
        for(const auto& obj : ultra->ul_objs()){
          if(obj.orientation() == "FRONT_SIDE_RIGHT"){
            points.emplace_back(Point2D(obj.position_global().x(), obj.position_global().y()));
          }
        }
      });
```

<div align="center">talker node</div>

```c++
auto talker_node = apollo::cyber::CreateNode("points");
auto point_writer = talker_node->CreateWriter<UltrasonicList>("perception/ultrasonic");
auto ultra_msg = std::make_shared<UltrasonicList>();
for(int i = 0; i < 10; i++){
  auto obj = ultra_msg->add_ul_objs();
  obj->set_position_global(Point2D(i, i));
  obj->set_orientation("FRONT_SIDE_RIGHT");
}
point_writer->Write(ultra_msg);
```
