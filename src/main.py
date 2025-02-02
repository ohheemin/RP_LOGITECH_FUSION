#!/usr/bin/env python3
# -- coding: utf-8 --

import cv2
import rclpy
import numpy as np  # 올바른 import
from rclpy.node import Node
from cam_cali import CameraCalibration  # 사용자 정의 카메라 보정 모듈
from lidar import LaserToPointCloud  # 사용자 정의 LiDAR 변환 모듈
from sensor_msgs.msg import PointCloud2, Image
from sensor_msgs_py import point_cloud2 as pc2
from cv_bridge import CvBridge  # ROS2 Image ↔ OpenCV 변환 모듈

class MainNode(Node):
    def __init__(self):
        super().__init__('main_node')

        # LiDAR 및 카메라 보정 클래스 초기화
        self.ltp = LaserToPointCloud()
        self.cam_cali = CameraCalibration()
        self.bridge = CvBridge()
        self.points_list = []  # 3D 포인트 클라우드 데이터 저장

        # LiDAR PointCloud2 구독
        self.create_subscription(
            PointCloud2,
            '/pointcloud',  # LiDAR 토픽 이름
            self.pointcloud_callback,
            10
        )

        # 카메라 이미지 구독
        self.create_subscription(
            Image,
            '/image_topic',  # 카메라 토픽 이름
            self.image_callback,
            10
        )

        # 타이머 설정 (0.1초마다 영상 처리)
        self.create_timer(0.1, self.process_video)

        self.frame = None  # 최신 프레임 저장

    def pointcloud_callback(self, cloud_msg):
        """ LiDAR PointCloud2 메시지 처리 콜백 """
        points = pc2.read_points(cloud_msg, field_names=("x", "y", "z"), skip_nans=True)
        self.points_list = [(x, y, z) for x, y, z in points]

        self.get_logger().info(f"Received {len(self.points_list)} Lidar points.")

    def image_callback(self, msg):
        """ 카메라 이미지 메시지 처리 콜백 """
        try:
            self.frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')  # ROS2 Image → OpenCV NumPy 배열 변환
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")

    def process_video(self):
        """ LiDAR 포인트를 카메라 이미지에 투영하여 시각화 """
        if self.frame is None or not self.points_list:
            return  # 데이터가 없으면 처리하지 않음

        # LiDAR 포인트를 NumPy 배열로 변환
        obj_points = np.array(self.points_list, dtype=np.float32).reshape(-1, 3)

        # 3D 포인트를 2D 이미지 좌표로 변환
        img_points, _ = cv2.projectPoints(
            obj_points,
            self.cam_cali.rvec,
            self.cam_cali.tvec,
            self.cam_cali.cameraMatrix,
            np.array([0, 0, 0, 0], dtype=float)  # 왜곡 계수 (Distortion Coefficients)
        )

        # 변환된 포인트를 이미지 위에 그리기
        for point in img_points:
            x, y = int(point[0][0]), int(point[0][1])
            if 0 <= x < self.frame.shape[1] and 0 <= y < self.frame.shape[0]:  # 유효한 범위 내에서만 그림
                cv2.circle(self.frame, (x, y), 3, (0, 0, 255), 1)

        # 영상 출력
        cv2.imshow("Lidar Projection", self.frame)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            self.shutdown_node()

    def shutdown_node(self):
        """ 노드 종료 시 정리 작업 수행 """
        cv2.destroyAllWindows()
        self.get_logger().info("Shutting down node.")
        self.destroy_node()
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = MainNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.shutdown_node()


if __name__ == '__main__':
    main()
