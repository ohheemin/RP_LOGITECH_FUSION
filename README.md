# RP_LOGITECH_FUSION
2d 라이다와 로지텍 카메라의 프로젝션 과정입니다. 2d 카메라는 RP 라이다, 카메라는 logitech 카메라 사용했습니다. 

# 사용 방법
RP 라이다에서는 드라이버를 이용하여 /scan 토픽을 받아오고, 웹캠은 연결되어 있는 상태여야 합니다. 

터미널 각 창으로 실행하여 
ros2 run rplidar_camera_calibration camera.py (/image_topic 받아오기)
ros2 run rplidar_camera_calibration lidar.py (/scan을 /pointcloud로 받아오기)
ros2 run rplidar_camera_calibration main.py (cam_cali 모듈 실행 후 프로젝션 진행)
을 차례대로 실행합니다. 
