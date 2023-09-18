image_topic_to_rtsp_server
===

A `ROS2` package that subscribe image topic and provide a `RTSP` video stream

This package is developed based on `ROS2 Humble`, `Gstreamer 1.0`, and `OpenCV 4.6.0`

---

# How To Build

1. Git clone the repository to your `ROS2` workspace
    ```bash
    cd <your workspace>/src
    git clone https://github.com/BensonSin0218/image_topic_to_rtsp_server.git
    ```

2. Build the package
    ```bash
    cd ../
    colcon build --packages-select image_topic_to_rtsp_server
    ```

# How To Launch

1. Source `ROS2` and your workspace
    ```bash
    source /opt/ros/humble/setup.bash
    source <your workspace>/install/setup.bash
    ```

2. Launch the package
    ```bash
    ros2 launch image_topic_to_rtsp_server image_topic_to_rtsp_server_node.launch.py
    ```

    > If every thing goes right, you should see the following output (Or something similar to this based on your params):
    > ```bash
    > Parameters:
    > -> image_topics:
    > --> color:
    > ---> topic_name: /camera/color/image_raw
    > ---> message_type: <class 'sensor_msgs.msg._image.Image'>
    > ---> qos: 1
    > ---> color info:
    > ----> topic_name: /camera/color/camera_info
    > ----> message_type: <class 'sensor_msgs.msg._camera_info.CameraInfo'>
    > ----> qos: 10
    > --> depth:
    > ---> topic_name: /camera/depth/image_rect_raw
    > ---> message_type: <class 'sensor_msgs.msg._image.Image'>
    > ---> qos: 1
    > ---> depth info:
    > ----> topic_name: /camera/depth/camera_info
    > ----> message_type: <class 'sensor_msgs.msg._camera_info.CameraInfo'>
    > ----> qos: 10
    > -> streaming:
    > --> incoming_address: 0.0.0.0
    > --> incoming_port: 8554
    > image_topic_to_rtsp_server_node initialized!
    > GStreamer RTSP:
    > -> server
    > --> incoming_address: 0.0.0.0
    > --> incoming_port: 8554
    > -> factory:
    > --> color:
    > ---> profile: (480, 640)
    > --> depth:
    > ---> profile: (480, 640)
    > ```

    > You may view the `RTSP` stream with any video player that supports `RTSP` \
    > URL:
    > - rtsp://127.0.0.1:8554/streaming/color
    > - rtsp://127.0.0.1:8554/streaming/depth

---

# Configuration

Once you build the package, you can change parameters in `config/params.yaml` to setup new streaming or change other settings
> Full path: `<your workspace>/install/image_topic_to_rtsp_server/share/image_topic_to_rtsp_server/config/params.yaml`

> Then you may use the `ros2 launch` command to re-launch the package

## Params

`image_topics_list`

- Name to enable `RTSP` video streaming, this name should be equal to the sections in `image_topics`
    - Note: The name here will be used in the `image_topics` section and this name will be also used in the `RTSP` streaming address

`image_topics`

- Section to set information that will ask the node to subscribe
- Format:
    ```yaml
    <name in image_topics_list>:
      topic_name: /camera/color/image_raw
      message_type: sensor_msgs/msg/Image
      qos: 1
      image_info:
        topic_name: /camera/color/camera_info
        message_type: sensor_msgs/msg/CameraInfo
        qos: 10
    ```

`RTSP_server`

- Section to configure `Gstreamer RTSP server` settings
- Parameters:
    - `incoming_address`: Incoming request address, **default: 0.0.0.0**
    - `incoming_port`: Incoming request port, **default: 8554**

---

# Trouble Shoot

`Fatal`: `Unable to get any image_info within time! Exiting the program...`
- Please check the `image_info` topic is publishing message or not, the `RTSP` video streaming requires information from the `image_info` topic

`Warn`: `Missing profile for <image_source>! Ignoring this image source!`
- Please check the `image_info` topic is publishing message or not, the `RTSP` video streaming requires information from the `image_info` topic