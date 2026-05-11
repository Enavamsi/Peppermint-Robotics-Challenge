#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

from cv_bridge import CvBridge

import cv2
import numpy as np


class VisualServoing(Node):

    def __init__(self):

        super().__init__('visual_servoing')

        # =====================================================
        # CV Bridge
        # =====================================================

        self.bridge = CvBridge()

        # =====================================================
        # Camera Subscriber
        # =====================================================

        self.image_subscriber = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        # =====================================================
        # LiDAR Subscriber
        # =====================================================

        self.scan_subscriber = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        # =====================================================
        # Velocity Publisher
        # =====================================================

        self.cmd_publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        # =====================================================
        # Controller Parameters
        # =====================================================

        # Proportional gain
        self.Kp = 0.0025

        # Alignment threshold
        self.alignment_threshold = 40

        # Speeds
        self.search_speed = 0.25
        self.forward_speed = 0.05

        # Safe stopping distance (meters)
        self.safe_distance = 2.5

        # Current front obstacle distance
        self.front_distance = 999.0

        self.get_logger().info('Visual Servoing with LiDAR Started')


    # =========================================================
    # LiDAR Callback
    # =========================================================

    def scan_callback(self, msg):

        # Front-facing LiDAR sector
        front_ranges = (
            list(msg.ranges[-15:]) +
            list(msg.ranges[:15])
        )

        # Remove invalid readings
        valid_ranges = [
            r for r in front_ranges
            if not np.isinf(r) and not np.isnan(r)
        ]

        # Get minimum front distance
        if len(valid_ranges) > 0:
            self.front_distance = min(valid_ranges)


    # =========================================================
    # Camera Callback
    # =========================================================

    def image_callback(self, msg):

        # Convert ROS Image -> OpenCV
        frame = self.bridge.imgmsg_to_cv2(
            msg,
            desired_encoding='bgr8'
        )

        # Image dimensions
        height, width, _ = frame.shape

        image_center_x = width // 2

        # =====================================================
        # Convert BGR -> HSV
        # =====================================================

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Green HSV range
        lower_green = np.array([40, 50, 50])
        upper_green = np.array([80, 255, 255])

        # Create binary mask
        mask = cv2.inRange(hsv, lower_green, upper_green)

        # Noise removal
        kernel = np.ones((5, 5), np.uint8)

        mask = cv2.erode(mask, kernel, iterations=1)
        mask = cv2.dilate(mask, kernel, iterations=2)

        # =====================================================
        # Find Contours
        # =====================================================

        contours, _ = cv2.findContours(
            mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )

        # =====================================================
        # Draw Image Center Line
        # =====================================================

        cv2.line(
            frame,
            (image_center_x, 0),
            (image_center_x, height),
            (255, 0, 0),
            2
        )

        # Velocity command
        twist = Twist()

        # =====================================================
        # SAFETY STOP USING LIDAR
        # =====================================================

        if self.front_distance < self.safe_distance:

            twist.linear.x = 0.0
            twist.angular.z = 0.0

            self.cmd_publisher.publish(twist)

            cv2.putText(
                frame,
                'OBSTACLE TOO CLOSE - STOP',
                (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 0, 255),
                2
            )

            cv2.putText(
                frame,
                f'Distance: {self.front_distance:.2f} m',
                (20, 80),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (255, 255, 255),
                2
            )

            cv2.imshow("Visual Servoing", frame)
            cv2.imshow("Green Mask", mask)

            cv2.waitKey(1)

            return

        # =====================================================
        # SEARCH MODE
        # =====================================================

        if len(contours) == 0:

            # Rotate to search
            twist.linear.x = 0.0
            twist.angular.z = self.search_speed

            self.cmd_publisher.publish(twist)

            cv2.putText(
                frame,
                'SEARCHING...',
                (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                1,
                (0, 0, 255),
                2
            )

        # =====================================================
        # TRACKING MODE
        # =====================================================

        else:

            # Largest contour
            largest_contour = max(
                contours,
                key=cv2.contourArea
            )

            area = cv2.contourArea(largest_contour)

            # Ignore tiny noise
            if area > 500:

                # Draw contour
                cv2.drawContours(
                    frame,
                    [largest_contour],
                    -1,
                    (0, 255, 0),
                    3
                )

                # Contour moments
                M = cv2.moments(largest_contour)

                if M["m00"] != 0:

                    # Contour center
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])

                    # Draw centroid
                    cv2.circle(
                        frame,
                        (cx, cy),
                        7,
                        (0, 0, 255),
                        -1
                    )

                    # Draw error line
                    cv2.line(
                        frame,
                        (image_center_x, cy),
                        (cx, cy),
                        (0, 255, 255),
                        2
                    )

                    # =================================================
                    # Horizontal Error
                    # =================================================

                    error_x = image_center_x - cx

                    # =================================================
                    # P Controller
                    # =================================================

                    angular_z = self.Kp * error_x

                    # Clamp angular velocity
                    angular_z = max(
                        min(angular_z, 1.0),
                        -1.0
                    )

                    # =================================================
                    # Move Forward If Aligned
                    # =================================================

                    if abs(error_x) < self.alignment_threshold:

                        twist.linear.x = self.forward_speed
                        twist.angular.z = angular_z

                    else:

                        twist.linear.x = 0.0
                        twist.angular.z = angular_z

                    # Publish velocity
                    self.cmd_publisher.publish(twist)

                    # =================================================
                    # Visualization
                    # =================================================

                    cv2.putText(
                        frame,
                        f'Center: ({cx}, {cy})',
                        (20, 40),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.7,
                        (255, 255, 255),
                        2
                    )

                    cv2.putText(
                        frame,
                        f'Error X: {error_x}',
                        (20, 80),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.7,
                        (0, 255, 255),
                        2
                    )

                    cv2.putText(
                        frame,
                        f'Angular Z: {angular_z:.3f}',
                        (20, 120),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.7,
                        (0, 255, 0),
                        2
                    )

                    cv2.putText(
                        frame,
                        f'LiDAR Distance: {self.front_distance:.2f} m',
                        (20, 160),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.7,
                        (255, 255, 255),
                        2
                    )

                    cv2.putText(
                        frame,
                        'TRACKING OBJECT',
                        (20, 200),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.8,
                        (0, 255, 0),
                        2
                    )

        # =====================================================
        # Show Windows
        # =====================================================

        cv2.imshow("Visual Servoing", frame)
        cv2.imshow("Green Mask", mask)

        cv2.waitKey(1)


def main(args=None):

    rclpy.init(args=args)

    node = VisualServoing()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()

    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()