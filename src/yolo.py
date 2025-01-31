#!/usr/bin/env python3
# -- coding: utf-8 --

import rclpy
from rclpy.node import Node
from ultralytics import YOLO
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class YOLOv8Node(Node):
    def __init__(self):
        super().__init__('yolov8_node')

        # Load the YOLOv8 model
        self.model = YOLO('./best.pt')
        self.bridge = CvBridge()

        # Publisher for annotated frames
        self.image_publisher = self.create_publisher(Image, '/yolov8/annotated_frames', 10)

        # Open the video file or camera
        self.cap = cv2.VideoCapture(6)

        # Create a timer to process the video
        self.create_timer(0.1, self.process_frame)

    def process_frame(self):
        if self.cap.isOpened():
            # Read a frame from the video
            success, frame = self.cap.read()

            if success:
                # Run YOLOv8 inference on the frame
                results = self.model(frame)

                # Visualize the results on the frame
                annotated_frame = results[0].plot()

                # Publish the annotated frame as a ROS 2 Image message
                image_message = self.bridge.cv2_to_imgmsg(annotated_frame, encoding='bgr8')
                self.image_publisher.publish(image_message)

                # Display the annotated frame
                cv2.imshow("YOLOv8 Inference", annotated_frame)

                # Exit if 'q' is pressed
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    self.destroy_node()
                    rclpy.shutdown()
            else:
                self.get_logger().warn("Failed to read a frame from the video.")

    def destroy_node(self):
        # Release the video capture object and close the display window
        self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    # Create and spin the YOLOv8 node
    yolov8_node = YOLOv8Node()
    try:
        rclpy.spin(yolov8_node)
    except KeyboardInterrupt:
        pass

    # Shutdown and clean up
    yolov8_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

