#!/usr/bin/env python3
# -- coding: utf-8 --

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ImagePublisher(Node):
    def __init__(self):
        super().__init__('image_publisher')
        self.pub = self.create_publisher(Image, 'image_topic', 10)
        self.bridge = CvBridge()
        self.timer = self.create_timer(0.03, self.timer_callback)  # Adjust the timer period for ~30 FPS
        self.cap = cv2.VideoCapture(2)

    def timer_callback(self):
        if self.cap.isOpened():
            success, frame = self.cap.read()
            if success:
                # Publish the frame as a ROS 2 Image message
                msg = self.bridge.cv2_to_imgmsg(frame, 'bgr8')
                self.pub.publish(msg)

                # Display the frame using OpenCV
                cv2.imshow("image", frame)

                # Break the loop if 'q' is pressed
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    self.cap.release()
                    cv2.destroyAllWindows()
                    rclpy.shutdown()
            else:
                self.get_logger().info("End of video or failed to read frame.")
                self.cap.release()
                cv2.destroyAllWindows()
                rclpy.shutdown()
        else:
            self.get_logger().error("Failed to open video capture device.")
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = ImagePublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

