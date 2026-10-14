#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class CameraViewer(Node):
    def __init__(self):
        super().__init__('camera_viewer')

        self.bridge = CvBridge()
        self.frame = None

        self.create_subscription(
            Image,
            '/rgbd_camera/image',
            self.callback,
            10
        )

    def callback(self, msg):
        try:
            # Camera is actually publishing rgb8
            self.frame = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding='rgb8'
            )
        except Exception as e:
            self.get_logger().error(str(e))


def main():
    rclpy.init()
    node = CameraViewer()

    cv2.namedWindow(
        'ROVER CAMERA',
        cv2.WINDOW_NORMAL
    )

    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.01)

        if node.frame is not None:
            # RGB -> BGR for OpenCV display
            frame = cv2.cvtColor(
                node.frame,
                cv2.COLOR_RGB2BGR
            )

            cv2.imshow('ROVER CAMERA', frame)

        key = cv2.waitKey(1) & 0xFF

        if key == ord('q'):
            break

    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
