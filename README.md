# Distributed ORB SLAM3

[ORBSLAM3](https://github.com/JussiKalliola/ORB_SLAM3) | [orbslam3 ros2](https://github.com/JussiKalliola/ORB_SLAM3_ROS2) | [orbslam3 interfaces](https://github.com/JussiKalliola/nano-slam-ros2) | [Dockerhub](https://hub.docker.com/repository/docker/jussikalliola/nano-ros2-orbslam3/general)

This is official implementation of distributed ORB SLAM3, which utilizes ROS2 framework for distributing data and computations of SLAM system into network of devices.

## System requirements

TBD.


## Pull the repository and build docker container

Pull repository and submodules
```
git clone https://github.com/JussiKalliola/nano-slam-ros2.git
git submodule update --init --recursive
```

Build and run docker container

```sh
cd docker

#Build docker container (NOT NEEDED) for amd64 architecture without GPU and with development tag
./build.sh -v amd64 -t dev -g 0

#Or run container with amd64 version and development tag (Pulls the docker image from dockerhub)
./run.sh -v amd64 -t dev
```

##### Build parameters
- `-h`: **Help**. summary of the parameters 
- `-v`: **Version**. Docker image version build for different architectures. E.g., arm64, amd64, ...
- `-p`: **Platforms**. Same as previous. E.g., arm64, amd64, ...
- `-c`: **Use Cache**. Use cache for building the image. Reduces the build time for subsequent builds, but might cause errors.
- `-g`: **GPU**. If GPU is available on the host comput **not implemented**. 1=True, 0=False.
- `-t`: **Tag**. Target build. Build process differs slightly with development/production, mainly that SLAM and other modules are prebuilt in the image. Faster for edge devices.


##### Run parameters
- `-h`: **Help**. summary of the parameters 
- `-v`: **Version**. Docker image version build for different architectures. E.g., arm64, amd64, ...
- `-g`: **GPU**. If GPU is available on the host comput **not implemented**. 1=True, 0=False.
- `-t`: **Tag**. Target build. Build process differs slightly with development/production, mainly that SLAM and other modules are prebuilt in the image. Faster for edge devices.

## Run experiments

#### TUM Monocular

Run the following command on "main" machine/robot

```
export SLAM_SYSTEM_ID=main
ros2 run orbslam3 mono ~/colcon_ws/src/orbslam3_ros2/vocabulary/ORBvoc.txt ~/colcon_ws/src/orbslam3_ros2/config/monocular/TUM-VI.yaml ~/results/TUM/room1/ false KeyFrameTrajectory-main.txt false true
```

And the following command on another machines in the network


```
export SLAM_SYSTEM_ID=sub
ros2 run orbslam3 mono ~/colcon_ws/src/orbslam3_ros2/vocabulary/ORBvoc.txt ~/colcon_ws/src/orbslam3_ros2/config/monocular/TUM-VI.yaml ~/results/TUM/room1/ false KeyFrameTrajectory-sub.txt true false
```

#### EuRoC Monocular

Run the following command on "main" machine/robot

```
export SLAM_SYSTEM_ID=main
ros2 run orbslam3 mono ~/colcon_ws/src/orbslam3_ros2/vocabulary/ORBvoc.txt ~/colcon_ws/src/orbslam3_ros2/config/monocular/EuRoC.yaml ~/results/EuRoC/MH_01/ false KeyFrameTrajectory-main.txt false true
```

And the following command on another machines in the network


```
export SLAM_SYSTEM_ID=sub
ros2 run orbslam3 mono ~/colcon_ws/src/orbslam3_ros2/vocabulary/ORBvoc.txt ~/colcon_ws/src/orbslam3_ros2/config/monocular/EuRoC.yaml ~/results/EuRoC/MH_01/ false KeyFrameTrajectory-sub.txt true false
```


#### Stereo, Stereo-inertial, Monocular-inertial, ...

TBD

#### Run experiments with robot/camera

TBD


##### List of parameters:
1. ORB Vocabulary file path
2. Config file path
3. Results directory path
4. Visualization on/off [BOOLEAN]
5. Results filename 
6. Subscribe to SLAM topics [BOOLEAN]
7. Main system [BOOLEAN]

```sh
ros2 run orbslam3 mono orb_path[STRING] config_path[STRING] results_dir[STRING] visualization[BOOLEAN] results_filename[STRING] subscribe[BOOLEAN] main_system[BOOLEAN]
```

## Datasets

#### ROS2 Bag
##### EuRoC dataset:

```sh
#Monocular
ros2 bag play ~/datasets/EuRoC/MH_01/ --remap /cam0/image_raw:=/camera

#Stereo-inertial
ros2 bag play ~/datasets/EuRoC/MH_01/ --remap /cam0/image_raw:=/camera/left /cam1/image_raw:=/camera/right /imu0:=/imu
```

##### TUM dataset

```sh
#Monocular
ros2 bag play ~/datasets/TUM/room1/ --remap /cam0/image_raw:=/camera

#Stereo-inertial
ros2 bag play ~/datasets/TUM/room1/ --remap /cam0/image_raw:=/camera/left /cam1/image_raw:=/camera/right /imu0:=/imu
```

## Evaluate

Activate python virtual environment and run the evaluation scripts
```sh
cd ~/ && source ~/traj_eval/bin/activate

#One trajectory with visualization
evo_traj tum ~/results/TUM/room1/monocular/KeyFrameTrajectory.txt -p

#Multiple trajectories with visualization
evo_traj tum ~/results/TUM/room1/monocular/* -p

# Alignment and scaling with GT data
evo_traj tum ~/results/EuRoC/MH_01/monocular/KeyFrameTrajectory* --ref ~/results/EuRoC/MH_01/monocular/gt.txt -p -as

```

More information from the documentation in [github by Michael Grupp](https://github.com/MichaelGrupp/evo/wiki/evo_traj)
