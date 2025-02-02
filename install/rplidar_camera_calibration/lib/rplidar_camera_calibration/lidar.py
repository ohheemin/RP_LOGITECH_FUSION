#!/usr/bin/env python3
# -- coding: utf-8 --

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import PointCloud2
from laser_geometry.laser_geometry import LaserProjection
from std_msgs.msg import Header

class LaserToPointCloud(Node):
    def __init__(self):
        super().__init__('laser_to_pointcloud')
        # Initialize LaserProjection
        self.laserProj = LaserProjection()
        
        # Subscriber to LaserScan messages
        self.laser_scan_subscriber = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
        
        # Publisher for PointCloud2 messages
        self.point_cloud_publisher = self.create_publisher(
            PointCloud2,
            '/pointcloud',
            10
        )

        self.get_logger().info("LaserToPointCloud Node initialized.")

    def scan_callback(self, scan_msg):
        # Project the LaserScan data into a PointCloud2 message
        cloud_msg = self.laserProj.projectLaser(scan_msg)

        # Publish the PointCloud2 message
        self.point_cloud_publisher.publish(cloud_msg)

        # Log the operation
        self.get_logger().info("Published PointCloud2 from LaserScan.")


def main(args=None):
    # Initialize ROS 2 Python client library
    rclpy.init(args=args)

    # Create node
    laser_to_pointcloud_node = LaserToPointCloud()

    try:
        # Keep the node running
        rclpy.spin(laser_to_pointcloud_node)
    except KeyboardInterrupt:
        pass

    # Cleanup on shutdown
    laser_to_pointcloud_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

