from .gst_rtsp_server import ImageToGstRTSPServer
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import ParameterDescriptor
from threading import Thread
from utils_package.utils import import_ROS_module

import cv2
import time
import os

import gi
gi.require_version("Gst", "1.0")
from gi.repository import Gst, GObject


class ImageTopicToRTSPServerNode(Node):
    def __init__(self, node_name: str = "image_topic_to_rtsp_server_node") -> None:
        super().__init__(node_name,
                         allow_undeclared_parameters=True)

        self.__image_topics: dict = {}
        self.__image_infos: dict = {}
        self.__cv_bridge: CvBridge = CvBridge()
        self.__images: dict = {}
        self.__RTSP_server_settings: dict = {}
        self.__streaming_thread: Thread = None

        self.__declare_parameters()
        self.__start_ros()
        self.__start_streaming()

        self.get_logger().info(f"{node_name} initialized!")

    def __declare_parameters(self) -> None:
        image_topics_list: list[str] = self.declare_parameter(
            "image_topics_list",
            descriptor=ParameterDescriptor(type=Parameter.Type.STRING_ARRAY.value,
                                           dynamic_typing=True)).value

        if image_topics_list is None or not len(image_topics_list):
            self.get_logger().error("Did not get any image topics! Exiting the program...")
            raise SystemExit

        for image_topic in image_topics_list:
            # Image topic
            prefix: str = f"image_topics.{image_topic}"
            topic_name: str = self.declare_parameter(
                f"{prefix}.topic_name",
                descriptor=ParameterDescriptor(type=Parameter.Type.STRING.value)).value
            topic_message_type_name: str = self.declare_parameter(
                f"{prefix}.message_type",
                descriptor=ParameterDescriptor(type=Parameter.Type.STRING.value)).value
            topic_message_type: any = import_ROS_module(topic_message_type_name,
                                                        self.get_logger())
            topic_qos: int = self.declare_parameter(
                f"{prefix}.qos", 1,
                descriptor=ParameterDescriptor(type=Parameter.Type.INTEGER.value)).value

            if not any([topic_name, topic_message_type_name]):
                self.get_logger().error(
                    f"Failed to setup {image_topic}, properly one of the arguments below is undefined!")
                self.get_logger().error(
                    f"topic_name: {topic_name} | message_type_name: {topic_message_type_name} | message_type: {topic_message_type}")

                continue

            self.__image_topics[image_topic]: dict = {
                "topic_name": topic_name,
                "message_type": topic_message_type,
                "qos": topic_qos
            }

            # Image topic info
            prefix: str = f"image_topics.{image_topic}.image_info"
            topic_name: str = self.declare_parameter(
                f"{prefix}.topic_name",
                descriptor=ParameterDescriptor(type=Parameter.Type.STRING.value)).value
            topic_message_type_name: str = self.declare_parameter(
                f"{prefix}.message_type",
                descriptor=ParameterDescriptor(type=Parameter.Type.STRING.value)).value
            topic_message_type: any = import_ROS_module(topic_message_type_name,
                                                        self.get_logger())
            topic_qos: int = self.declare_parameter(
                f"{prefix}.qos", 1,
                descriptor=ParameterDescriptor(type=Parameter.Type.INTEGER.value)).value

            if not any([topic_name, topic_message_type_name]):
                self.get_logger().error(
                    f"Failed to setup {image_topic}, properly one of the arguments below is undefined!")
                self.get_logger().error(
                    f"topic_name: {topic_name} | message_type: {topic_message_type_name} | message_type: {topic_message_type}")

                continue

            self.__image_infos[image_topic]: dict = {
                "topic_name": topic_name,
                "message_type": topic_message_type,
                "qos": topic_qos
            }

            self.__images[image_topic]: any = None

        if not len(self.__image_topics):
            self.get_logger().fatal("Did not setup any image topics! Exiting the program...")
            raise SystemExit

        self.__RTSP_server_settings["incoming_address"]: str = self.declare_parameter(
            "RTSP_server.incoming_address", "0.0.0.0",
            descriptor=ParameterDescriptor(type=Parameter.Type.STRING.value)).value
        self.__RTSP_server_settings["incoming_port"]: int = self.declare_parameter(
            "RTSP_server.incoming_port", 8554,
            descriptor=ParameterDescriptor(type=Parameter.Type.INTEGER.value)).value

        self.get_logger().info("Parameters:")
        self.get_logger().info("-> image_topics:")
        for (image_topic_source, image_topic), (image_info_source, image_info) in \
            zip(self.__image_topics.items(), self.__image_infos.items()):
            self.get_logger().info(f"--> {image_topic_source}:")
            self.get_logger().info(f"---> topic_name: {image_topic['topic_name']}")
            self.get_logger().info(f"---> message_type: {image_topic['message_type']}")
            self.get_logger().info(f"---> qos: {image_topic['qos']}")
            self.get_logger().info(f"---> {image_info_source} info:")
            self.get_logger().info(f"----> topic_name: {image_info['topic_name']}")
            self.get_logger().info(f"----> message_type: {image_info['message_type']}")
            self.get_logger().info(f"----> qos: {image_info['qos']}")
        self.get_logger().info("-> streaming:")
        self.get_logger().info(f"--> incoming_address: {self.__RTSP_server_settings['incoming_address']}")
        self.get_logger().info(f"--> incoming_port: {self.__RTSP_server_settings['incoming_port']}")

    def __start_ros(self) -> None:
        for image_source in self.__image_topics.keys():
            self.create_subscription(
                self.__image_topics[image_source]["message_type"],
                self.__image_topics[image_source]["topic_name"],
                self.__get_image_callback(image_source),
                self.__image_topics[image_source]["qos"])

        for image_source in self.__image_infos.keys():
            self.__image_infos[image_source]["subscription"] = self.create_subscription(
                self.__image_infos[image_source]["message_type"],
                self.__image_infos[image_source]["topic_name"],
                self.__get_image_info_callback(image_source),
                self.__image_infos[image_source]["qos"])

    def __get_image_callback(self, image_source: str) -> callable:
        def __image_callback(msg: any) -> None:
            self.__images[image_source] = self.__cv_bridge.imgmsg_to_cv2(msg)

        return __image_callback

    def __get_image_info_callback(self, image_source: str) -> callable:
        def __image_info_callback(msg: any) -> None:
            self.__image_infos[image_source]["profile"] = (msg.height, msg.width)

        return __image_info_callback

    def __get_image(self, image_source: str) -> any:
        if not image_source in self.__images or self.__images[image_source] is None:
            return None

        return self.__images[image_source]

    def __start_streaming(self) -> None:
        self.__streaming_thread = Thread(target=self.__GST_RTSP_streaming_server_thread,
                                         daemon=True)
        self.__streaming_thread.start()

    def __GST_RTSP_streaming_server_thread(self) -> None:
        # Get image infos first before starting the RTSP streaming
        timeout = time.time() + 5
        while not any(["profile" in image_info for image_info in self.__image_infos.values()]):
            if time.time() >= timeout:
                self.get_logger().fatal("Unable to get any image_info within time! Exiting the program...")
                os._exit(1)

        # Cancel subscription
        for image_source in self.__image_infos.keys():
            self.destroy_subscription(self.__image_infos[image_source]["subscription"])

        # Start GStreamer RTSP server
        self.get_logger().info("GStreamer RTSP:")
        self.get_logger().info("-> server")
        self.get_logger().info(f"--> incoming_address: {self.__RTSP_server_settings['incoming_address']}")
        self.get_logger().info(f"--> incoming_port: {self.__RTSP_server_settings['incoming_port']}")
        self.get_logger().info("-> factory:")

        factory_configs: dict = {}
        for image_source, image_info in self.__image_infos.items():
            self.get_logger().info(f"--> {image_source}:")

            if not "profile" in image_info:
                self.get_logger().warning(f"---> Missing profile for {image_source}! Ignoring this image source!")
                continue

            self.get_logger().info(f"---> profile: {image_info['profile']}")

            factory_configs[image_source]: dict = {
                "profile": image_info["profile"],
                "get_image": self.__get_image
            }

        if not len(factory_configs):
            self.get_logger().error("No factory configurations created! Exiting the program...")
            raise SystemExit

        GObject.threads_init()
        Gst.init(None)

        _ = ImageToGstRTSPServer(
            self.__RTSP_server_settings["incoming_address"],
            self.__RTSP_server_settings["incoming_port"],
            factory_configs)
        loop = GObject.MainLoop()
        loop.run()
