Commands:

`ros2 run orbslam3 mono ~/colcon_ws/src/orbslam3_ros2/vocabulary/ORBvoc.txt ~/colcon_ws/src/orbslam3_ros2/config/monocular/TUM-VI.yaml ~/result/TUM/room1/ true true`

Run EuRoC dataset:
`ros2 bag play datasets/EuRoC/MH_01/ --remap /cam0/image_raw:=/camera/left /cam1/image_raw:=/camera/right /imu0:=/imu`  

Run TUM dataset:
`ros2 bag play datasets/TUM/room1/ --remap /cam0/image_raw:=/camera`
`ros2 bag play datasets/TUM/room1/ --remap /cam0/image_raw:=/camera/left /cam1/image_raw:=/camera/right /imu0:=/imu`
