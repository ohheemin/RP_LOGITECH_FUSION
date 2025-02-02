#!/usr/bin/env python3
# -- coding: utf-8 --

import rclpy
from rclpy.node import Node
import numpy as np
import cv2
from pathlib import Path
import os
import glob


class CameraCalibration(Node):
    def __init__(self):
        super().__init__('camera_calibration')
        
        # Calibration settings
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        wc = 8  # Number of horizontal checkerboard patterns - 1
        hc = 6  # Number of vertical checkerboard patterns - 1
        objp = np.zeros((wc * hc, 3), np.float32)
        objp[:, :2] = np.mgrid[0:wc, 0:hc].T.reshape(-1, 2)

        objpoints = []  # 3D points in real-world space
        imgpoints = []  # 2D points in image plane

        # Load checkerboard images from the directory
        script_path = Path(__file__).parent
        images = glob.glob(str(script_path / 'checkerboard_jpg/*.jpg'))

        for frame in images:
            img = cv2.imread(frame)
            self.gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)  # Convert to grayscale

            # Find the checkerboard corners
            ret, corners = cv2.findChessboardCorners(
                self.gray, (wc, hc), 
                cv2.CALIB_CB_ADAPTIVE_THRESH + 
                cv2.CALIB_CB_FAST_CHECK + 
                cv2.CALIB_CB_NORMALIZE_IMAGE
            )

            # Log the status of corner detection
            self.get_logger().info(f"Checkerboard detection result: {ret}")

            if ret:
                objpoints.append(objp)

                # Refine corner detection
                corners2 = cv2.cornerSubPix(self.gray, corners, (10, 10), (-1, -1), criteria)
                imgpoints.append(corners2)

                # Draw checkerboard corners and display the image
                img = cv2.drawChessboardCorners(img, (wc, hc), corners2, ret)
                cv2.imshow('Checkerboard', img)
                cv2.waitKey()

        # Perform camera calibration
        ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
            objpoints, imgpoints, self.gray.shape[::-1], None, None
        )

        # Log calibration results
        self.get_logger().info(f"Calibration successful: {ret}")
        self.get_logger().info(f"Camera Matrix:\n{mtx}")
        self.get_logger().info(f"Distortion Coefficients:\n{dist}")

        # Set up 2D-3D point correspondence
        points_2D = np.array([
            (155, 330),
            (210, 280),
            (320, 280),
            (415, 280),
        ], dtype="double")

        points_3D = np.array([
            (-1.0732, -0.17268, 0),
            (-1.6316, -0.17515, 0),
            (-1.6426, 0.39314, 0),
            (-1.6522, 0.68096, 0)
        ], dtype="double")

        self.cameraMatrix = mtx
        self.dist_coeffs = dist

        # SolvePnP to compute rotation and translation vectors
        retval, rvec, tvec = cv2.solvePnP(
            points_3D, points_2D, self.cameraMatrix, self.dist_coeffs
        )

        self.rvec, _ = cv2.Rodrigues(rvec)
        self.tvec = tvec

        # Construct intrinsic and extrinsic matrices
        self.intrinsic = np.append(self.cameraMatrix, [[0, 0, 0]], axis=0)
        self.intrinsic = np.append(self.intrinsic, [[0], [0], [0], [1]], axis=1)

        extrinsic = np.append(self.rvec, self.tvec, axis=1)
        self.extrinsic = np.append(extrinsic, [[0, 0, 0, 1]], axis=0)

        # Log matrices
        self.get_logger().info(f"Intrinsic Matrix:\n{self.intrinsic}")
        self.get_logger().info(f"Extrinsic Matrix:\n{self.extrinsic}")
        self.get_logger().info(f"Rotation Matrix:\n{self.rvec}")
        self.get_logger().info(f"Translation Vector:\n{self.tvec}")


def main(args=None):
    rclpy.init(args=args)
    node = CameraCalibration()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()

