## Pull the repository and build docker container

Pull repository and submodules
`git clone https://github.com/JussiKalliola/nano-slam-ros2.git`

`git submodule update --init --recursive`


Build and run docker container

`cd docker`

Build docker container for amd64 architecture without gpu:
`./build.sh -v amd64 -p amd64 -g 0`

Run container with amd64 version:
`./run.sh -v amd64`

## Run experiments

### Run SLAM System with visualization:

TUM Monocular
`ros2 run orbslam3 mono ~/colcon_ws/src/orbslam3_ros2/vocabulary/ORBvoc.txt ~/colcon_ws/src/orbslam3_ros2/config/monocular/TUM-VI.yaml ~/results/TUM/room1/ true true`

EuRoC Monocular
`ros2 run orbslam3 mono ~/colcon_ws/src/orbslam3_ros2/vocabulary/ORBvoc.txt ~/colcon_ws/src/orbslam3_ros2/config/monocular/EuRoC.yaml ~/results/EuRoC/MH_01/ true true`


### Run ROS2 Bag datasets

#### EuRoC dataset:

Monocular
`ros2 bag play ~/datasets/EuRoC/MH_01/ --remap /cam0/image_raw:=/camera`

Stereo-inertial
`ros2 bag play ~/datasets/EuRoC/MH_01/ --remap /cam0/image_raw:=/camera/left /cam1/image_raw:=/camera/right /imu0:=/imu`  

#### TUM dataset

Monocular
`ros2 bag play ~/datasets/TUM/room1/ --remap /cam0/image_raw:=/camera`

Stereo-inertial
`ros2 bag play ~/datasets/TUM/room1/ --remap /cam0/image_raw:=/camera/left /cam1/image_raw:=/camera/right /imu0:=/imu`

## Evaluate

### Activate python virtual environment:

`cd ~/ && source ~/traj_eval/bin/activate`

### Evaluate

One trajectory with visualization
`evo_traj tum ~/results/TUM/room1/monocular/KeyFrameTrajectory.txt -p`

Multiple trajectories with visualization
`evo_traj tum ~/results/TUM/room1/monocular/* -p`

More information from the documentation in [github by Michael Grupp](https://github.com/MichaelGrupp/evo/wiki/evo_traj)
