#!/usr/bin/env python3

import json
import math

import cv2
import numpy as np
import rclpy
import torch

from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String

from cv_bridge import CvBridge
from ultralytics import YOLO


class RGBDYOLO3D(Node):

    def __init__(self):
        super().__init__('rgbd_yolo_3d')

        self.bridge = CvBridge()

        self.rgb_image = None
        self.depth_image = None

        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None

        self.model = YOLO('yolo11n.pt')

        if torch.cuda.is_available():
            self.model.to('cuda')
            self.device = 'cuda:0'
        else:
            self.device = 'cpu'

        self.get_logger().info(
            f'YOLO device: {self.device}'
        )

        self.create_subscription(
            Image,
            '/rgbd_camera/image',
            self.image_callback,
            10
        )

        self.create_subscription(
            Image,
            '/rgbd_camera/depth_image',
            self.depth_callback,
            10
        )

        self.create_subscription(
            CameraInfo,
            '/rgbd_camera/camera_info',
            self.camera_info_callback,
            10
        )

        self.object_pub = self.create_publisher(
            String,
            '/semantic_objects',
            10
        )

        self.debug_pub = self.create_publisher(
            Image,
            '/semantic_view',
            10
        )

        self.frame_count = 0

    def camera_info_callback(self, msg):

        self.fx = msg.k[0]
        self.fy = msg.k[4]
        self.cx = msg.k[2]
        self.cy = msg.k[5]

    def depth_callback(self, msg):

        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding='passthrough'
            )
        except Exception as e:
            self.get_logger().error(
                f'Depth conversion error: {e}'
            )

    def image_callback(self, msg):

        if self.depth_image is None:
            return

        if self.fx is None:
            return

        try:
            rgb = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding='rgb8'
            )

            bgr = cv2.cvtColor(
                rgb,
                cv2.COLOR_RGB2BGR
            )

            results = self.model.predict(
                source=bgr,
                device=self.device,
                conf=0.35,
                verbose=False
            )

            objects = []

            annotated = bgr.copy()

            for result in results:

                if result.boxes is None:
                    continue

                for box in result.boxes:

                    xyxy = box.xyxy[0].cpu().numpy()
                    confidence = float(
                        box.conf[0].cpu().numpy()
                    )
                    class_id = int(
                        box.cls[0].cpu().numpy()
                    )

                    x1, y1, x2, y2 = map(
                        int,
                        xyxy
                    )

                    x1 = max(0, x1)
                    y1 = max(0, y1)
                    x2 = min(
                        bgr.shape[1] - 1,
                        x2
                    )
                    y2 = min(
                        bgr.shape[0] - 1,
                        y2
                    )

                    u = (x1 + x2) // 2
                    v = (y1 + y2) // 2

                    depth = self.get_object_depth(
                        x1, y1, x2, y2
                    )

                    if depth is None:
                        continue

                    # Camera optical-frame coordinates:
                    # X = right, Y = down, Z = forward
                    X = (u - self.cx) * depth / self.fx
                    Y = (v - self.cy) * depth / self.fy
                    Z = depth

                    distance = math.sqrt(
                        X * X +
                        Y * Y +
                        Z * Z
                    )

                    bearing_deg = math.degrees(
                        math.atan2(X, Z)
                    )

                    relative_height = -Y

                    class_name = result.names[class_id]

                    if bearing_deg < -5:
                        horizontal = 'left'
                    elif bearing_deg > 5:
                        horizontal = 'right'
                    else:
                        horizontal = 'center'

                    objects.append({
                        'class': class_name,
                        'confidence': round(
                            confidence,
                            3
                        ),
                        'distance_m': round(
                            distance,
                            3
                        ),
                        'x_m': round(X, 3),
                        'y_m': round(Y, 3),
                        'z_m': round(Z, 3),
                        'bearing_deg': round(
                            bearing_deg,
                            2
                        ),
                        'horizontal': horizontal,
                        'relative_height_m': round(
                            relative_height,
                            3
                        ),
                        'bbox': [
                            x1, y1, x2, y2
                        ]
                    })

                    cv2.rectangle(
                        annotated,
                        (x1, y1),
                        (x2, y2),
                        (0, 255, 0),
                        2
                    )

                    label = (
                        f'{class_name} '
                        f'{confidence:.2f} '
                        f'{distance:.2f}m'
                    )

                    cv2.putText(
                        annotated,
                        label,
                        (x1, max(20, y1 - 8)),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.55,
                        (0, 255, 0),
                        2
                    )

            message = String()

            message.data = json.dumps({
                'frame_id': msg.header.frame_id,
                'num_objects': len(objects),
                'objects': objects
            })

            self.object_pub.publish(message)

            debug_msg = self.bridge.cv2_to_imgmsg(
                annotated,
                encoding='bgr8'
            )

            debug_msg.header = msg.header

            self.debug_pub.publish(
                debug_msg
            )

            self.frame_count += 1

            if self.frame_count % 30 == 0:
                self.get_logger().info(
                    f'Processed {self.frame_count} frames | '
                    f'Objects: {len(objects)}'
                )

        except Exception as e:

            self.get_logger().error(
                f'RGB processing error: {e}'
            )

    def get_object_depth(
        self,
        x1,
        y1,
        x2,
        y2
    ):

        h, w = self.depth_image.shape[:2]

        # Use the central 30% of the bounding box
        # to reduce background-depth contamination.
        bx1 = int(
            x1 + 0.35 * (x2 - x1)
        )
        bx2 = int(
            x1 + 0.65 * (x2 - x1)
        )
        by1 = int(
            y1 + 0.35 * (y2 - y1)
        )
        by2 = int(
            y1 + 0.65 * (y2 - y1)
        )

        bx1 = max(0, min(w - 1, bx1))
        bx2 = max(0, min(w, bx2))
        by1 = max(0, min(h - 1, by1))
        by2 = max(0, min(h, by2))

        if bx2 <= bx1 or by2 <= by1:
            return None

        region = self.depth_image[
            by1:by2,
            bx1:bx2
        ]

        valid = region[
            np.isfinite(region) &
            (region > 0.1) &
            (region < 10.0)
        ]

        if valid.size == 0:
            return None

        return float(
            np.median(valid)
        )


def main(args=None):

    rclpy.init(args=args)

    node = RGBDYOLO3D()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
