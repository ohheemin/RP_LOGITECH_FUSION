# About RP_LOGITECH_FUSION

2d 라이다와 로지텍 카메라의 프로젝션 과정입니다. 2d 카메라는 RP 라이다, 카메라는 logitech 카메라 사용했습니다. 

ubuntu 22.04 os 시스템에서 ros2 humble을 설치한 환경에서 진행하였습니다. 

# 사용 방법

ubuntu 22.04 os 시스템에서 ros2 humble을 설치한 환경에서 진행하였습니다. 
RP 라이다에서는 드라이버를 이용하여 /scan 토픽을 받아오고, 웹캠은 연결되어 있는 상태여야 합니다. 

터미널 각 창으로 실행하여 

ros2 run rplidar_camera_calibration camera.py (/image_topic 받아오기)

ros2 run rplidar_camera_calibration lidar.py (/scan을 /pointcloud로 받아오기)

ros2 run rplidar_camera_calibration main.py (cam_cali 모듈 실행 후 프로젝션 진행)

을 차례대로 실행합니다. 

# 결과 화면

![Image](https://github.com/user-attachments/assets/344de8ef-421c-4913-9ce0-4b22f421b493)
