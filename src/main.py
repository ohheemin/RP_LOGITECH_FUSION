#!/usr/bin/env python3
# -- coding: utf-8 --

import cv2
import rclpy
import numpy as np
from rclpy.node import Node
from cam_cali import CameraCali  # Your custom camera calibration module
from lidar import LaserToPointCloud  # Your custom LaserScan to PointCloud2 module
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2


class MainNode(Node):
    def __init__(self):
        super().__init__('main_node')

        # Initialize LaserToPointCloud and CameraCali
        self.ltp = LaserToPointCloud()
        self.cam_cali = CameraCali()

        # Subscribe to the PointCloud2 topic published by LaserToPointCloud
        self.create_subscription(
            PointCloud2,
            '/pointcloud',
            self.pointcloud_callback,
            10
        )

        # Open the video feed
        self.cap = cv2.VideoCapture(4)
        self.points_list = []

        # Create a timer to process the video frames
        self.create_timer(0.1, self.process_video)

    def pointcloud_callback(self, cloud_msg):
        # Read points from the PointCloud2 message
        points = pc2.read_points(cloud_msg, skip_nans=True)
        points_list = list(points)
        # Extract (x, y, z) from the points
        self.points_list = [(x, y, z) for x, y, z, _, _ in points_list]

        self.get_logger().info(f"Received {len(self.points_list)} points from PointCloud2.")

    def process_video(self):
        if self.cap.isOpened():
            # Read a frame from the video feed
            success, frame = self.cap.read()

            if success and self.points_list:
                # Convert points to numpy array and reshape
                obj_points = np.array(self.points_list, dtype=np.float32).reshape(-1, 3)

                # Project 3D points to 2D image plane
                img_points, _ = cv2.projectPoints(
                    obj_points,
                    self.cam_cali.rvec,
                    self.cam_cali.tvec,
                    self.cam_cali.cameraMatrix,
                    np.array([0, 0, 0, 0], dtype=float)
                )

                # Annotate the frame with the projected points
                for i in range(len(img_points)):
                    x, y = int(img_points[i][0][0]), int(img_points[i][0][1])
                    cv2.circle(frame, (x, y), 3, (0, 0, 255), 1)

                # Display the annotated frame
                cv2.imshow("image", frame)

                # Exit if 'q' is pressed
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    self.destroy_node()
                    rclpy.shutdown()

    def destroy_node(self):
        # Release the video capture object and close the display window
        self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    # Create and spin the main node
    main_node = MainNode()
    try:
        rclpy.spin(main_node)
    except KeyboardInterrupt:
        pass

    # Shutdown and clean up
    main_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

