#!/usr/bin/env python3
# -- coding: utf-8 --

import cv2
import rclpy
import numpy as np
from rclpy.node import Node
from cam_cali import CameraCalibration  # Custom camera calibration module
from lidar import LaserToPointCloud  # Custom LaserScan to PointCloud2 module
from sensor_msgs.msg import PointCloud2

def open_webcam():
    # 웹캠 열기
    cap = cv2.VideoCapture(2)  # 0은 기본 웹캠을 의미합니다. 다른 장치를 사용하려면 1, 2로 변경.
    
    if not cap.isOpened():
        print("웹캠을 열 수 없습니다.")
        return None
    
    print("웹캠을 실행합니다. 'q'를 눌러 종료하세요.")
    
    return cap


class MainNode(Node):
    def __init__(self):
        super().__init__('main_node')

        # Initialize LaserToPointCloud and CameraCalibration
        self.ltp = LaserToPointCloud()
        self.cam_cali = CameraCalibration()

        # Subscribe to the PointCloud2 topic published by LaserToPointCloud
        self.create_subscription(
            PointCloud2,
            '/pointcloud',
            self.pointcloud_callback,
            10
        )

        # Open the video feed
        self.cap = open_webcam()
        if self.cap is None:
            self.get_logger().error("웹캠을 열 수 없습니다. 종료합니다.")
            rclpy.shutdown()

        self.points_list = []

        # Create a timer to process the video frames
        self.create_timer(0.1, self.process_video)

    def pointcloud_callback(self, cloud_msg):
        # Read points from the PointCloud2 message
        points = PointCloud2.read_points(cloud_msg, skip_nans=True)
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
                cv2.imshow("Webcam", frame)

                # Exit if 'q' is pressed
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    self.destroy_node()
                    rclpy.shutdown()

    def destroy_node(self):
        # Release the video capture object and close the display window
        if self.cap is not None and self.cap.isOpened():
            self.cap.release()
        cv2.destroyAllWindows()
        self.get_logger().info("노드 종료, 리소스 정리 완료")
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
