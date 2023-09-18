from .models.image_topic_to_rtsp_server_node import ImageTopicToRTSPServerNode

import rclpy

def main(args=None) -> None:
    rclpy.init(args=args)

    node = ImageTopicToRTSPServerNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()