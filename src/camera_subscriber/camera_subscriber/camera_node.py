#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image

import cv2
import numpy as np
from cv_bridge import CvBridge


class CmdVelPublisher(Node):

    def __init__(self):
        super().__init__('cmd_vel_publisher')

        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.subscription = self.create_subscription(
            Image,
            '/image_raw',
            self.listener_callback,
            10
        )

        self.bridge = CvBridge()

        self.debil = 0   # sterowanie
        self.marker_seen = False

        # ArUco
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(
            cv2.aruco.DICT_6X6_250
        )
        self.aruco_params = cv2.aruco.DetectorParameters_create()

        self.get_logger().info("Camera ArUco controller started")

    def timer_callback(self):
        msg = Twist()

        if self.marker_seen and self.debil == 1:
            msg.linear.x = 0.5
            msg.angular.z = 0.0
        elif self.marker_seen:
            msg.linear.x = -0.5
            msg.angular.z = 0.0
        else:
            msg.linear.x = 0.0
            msg.angular.z = 0.0

        self.publisher_.publish(msg)

    def listener_callback(self, image_msg):
        frame = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8')
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        corners, ids, _ = cv2.aruco.detectMarkers(
            gray,
            self.aruco_dict,
            parameters=self.aruco_params
        )

        if ids is None:
            self.marker_seen = False
            return

        self.marker_seen = True

        # środek pierwszego markera
        c = corners[0][0]
        cy = int(np.mean(c[:, 1]))

        h = frame.shape[0]

        if cy > h // 2:
            self.debil = 1
            self.get_logger().info("MARKER NISKO -> JAZDA")
        else:
            self.debil = 0
            self.get_logger().info("MARKER WYSOKO -> OBRÓT")


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

