# Rubik's Cube Solving 5 DoF Robotic Arm

In this project, we programmed px-150 a 5 DoF robotic arm to solve a Rubik's cube. 

Due to the limitations of px-150 we simplified the problem such that the robot's work space is only in x-direction. We added track/rails to support cube manipulation.

Results:

[![IMAGE ALT TEXT HERE]([https://www.youtube.com/watch?v=YOUTUBE_VIDEO_ID_HERE](https://www.youtube.com/watch?v=ZS7FbuDW03o))

To reproduce the results, follow these steps:

1. OS - Ubuntu 20.04
2. ROS version - Noetic
3. Follow this software setup - https://docs.trossenrobotics.com/interbotix_xsarms_docs/ros_interface/ros1/software_setup.html
4. Clone this repo inside the interbotix_ws workspace directory - interbotix_ros_manipulators/interbotix_ros_xsarms/
5. Run camera calibration to get the calibration matrix, this process is important since the position estimation programs use it. This will not modify your webcam calibration.
6. Run python3 solve_cube.py -c <camera_number> -n <camera_name>

References:
- https://docs.opencv.org/4.x/d5/dae/tutorial_aruco_detection.html
- https://github.com/Asadullah-Dal17/Basic-Augmented-reality-course-opencv?tab=readme-ov-file
- https://aliyasineser.medium.com/calculation-relative-positions-of-aruco-markers-eee9cc4036e3
- https://docs.trossenrobotics.com/interbotix_xsarms_docs/index.html
- https://github.com/Interbotix/interbotix_ros_manipulators
