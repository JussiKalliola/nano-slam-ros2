Commands:

`ros2 run orbslam3 stereo-inertial ~/colcon_ws/src/orbslam3_ros2/vocabulary/ORBvoc.txt ~/colcon_ws/src/orbslam3_ros2/config/stereo-inertial/TUM-VI.yaml BOOL_RECTIFY [BOOL_EQUALIZE]`

Run EuRoC dataset:
`ros2 bag play datasets/EuRoC/MH_01/ --remap /cam0/image_raw:=/camera/left /cam1/image_raw:=/camera/right /imu0:=/imu`  

Run TUM dataset:
`ros2 bag play datasets/TUM/room1/ --remap /cam0/image_raw:=/camera/left /cam1/image_raw:=/camera/right /imu0:=/imu`
