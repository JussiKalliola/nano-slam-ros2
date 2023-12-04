#!/bin/bash


REGISTRY=jussikalliola
IMAGE=nano-ros2-orbslam3
VERSION=latest
USE_CACHE=1
SYSTEM_ARCH=$(uname -m)
GPU_SUPPORT=0
PWD=$(pwd)
ROOT_DIR=$PWD/..

# Help function
show_help() {
  echo "Usage: $0 [OPTIONS]"
  echo "Options:"
  echo "  -h    Show help information"
  echo "  -v    Version; amd64,arm64,..."
  echo "  -g    GPU; 1=True, 0=False"
  echo "  -t    Target build; dev,prod,test... Default=dev"
  exit 0
}

if [[ "$1" == "-h" || "$1" == "--help" ]]; then
  show_help
fi




while getopts v:g:t: flag
do
    case "${flag}" in
        v) VERSION=${OPTARG};;
        g) GPU=${OPTARG};;
        t) TARGET_BUILD=${OPTARG};;
    esac
done

  
# Check version
if [ -z "${VERSION}" ]; 
then
  echo "No version defined. DEFAULT: latest"
  VERSION=latest
else
  echo "Version selected: ${VERSION}"
fi


# Check gpu
if [ nvidia-smi &> /dev/null ] && [ "$GPU" == "1" ]; then
  echo "Found gpu(s), pulling correct image and enable gpu support."
  # Here pull right image when its done.
  #VERSION+="_gpu"
  #DOCKERFILE=gpu.Dockerfile  
  GPU_SUPPORT=1

else
  echo "No GPU support, disable gpu(s)."
  #DOCKERFILE=Dockerfile
fi



# Check target build, default dev
if [ -z "$TARGET_BUILD" ]; then
  echo "No target build given. Default: dev"
  TARGET_BUILD="dev"
fi


echo -e "\n============================"
echo -e "Running with params:\n - VERSION=${VERSION}\n - TARGET_BUILD=${TARGET_BUILD}\n - GPU=${GPU} - HOME DIRECTORY=${ROOT_DIR}"
echo -e "============================\n"


echo "Pulling an image: ${REGISTRY}/${IMAGE}:${VERSION}"
docker pull $REGISTRY/$IMAGE:$VERSION

xhost +

if [ $GPU_SUPPORT == 1 ]; then

  docker run  -it \
              --rm \
              --net=host \
              --cap-add NET_ADMIN \
              --privileged \
              --gpus all \
              -e DISPLAY=$DISPLAY \
              -e XDG_RUNTIME_DIR=$XDG_RUNTIME_DIR \
              -e PYTHONBUFFERED=1 \
              -e TARGET_BUILD=${TARGET_BUILD} \
              -v /etc/timezone:/etc/timezone:ro \
              -v /etc/localtime:/etc/localtime:ro \
              -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
              -v $HOME/.Xauthority:/root/.Xauthority \
              -v $HOME/.tmux/:/root/.tmux \
              -v $HOME/.config/:/root/.config \
              -v /run/user/1000:/run/user/1000 \
              -v $ROOT_DIR/datasets/:/root/datasets \
              -v $ROOT_DIR/results/:/root/results \
              -v $ROOT_DIR/orbslam3/orbslam3_ros2/:/root/colcon_ws/src/orbslam3_ros2 \
              -v $ROOT_DIR/orbslam3/orbslam3_interfaces/:/root/colcon_ws/src/orbslam3_interfaces \
              -v $ROOT_DIR/orbslam3/ORB_SLAM3/:/root/ORB_SLAM3-dev \
              --device=/dev/bus/usb:/dev/bus/usb \
              $REGISTRY/$IMAGE:$VERSION \
              bash
else

  docker run  -it \
              --rm \
              --net=host \
              --cap-add NET_ADMIN \
              --privileged \
              -e DISPLAY=$DISPLAY \
              -e XDG_RUNTIME_DIR=$XDG_RUNTIME_DIR \
              -e PYTHONBUFFERED=1 \
              -e TARGET_BUILD=${TARGET_BUILD} \
              -v /etc/timezone:/etc/timezone:ro \
              -v /etc/localtime:/etc/localtime:ro \
              -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
              -v $HOME/.Xauthority:/root/.Xauthority \
              -v $HOME/.tmux/:/root/.tmux \
              -v $HOME/.config/:/root/.config \
              -v /run/user/1000:/run/user/1000 \
              -v $ROOT_DIR/datasets/:/root/datasets \
              -v $ROOT_DIR/results/:/root/results \
              -v $ROOT_DIR/orbslam3/orbslam3_ros2/:/root/colcon_ws/src/orbslam3_ros2 \
              -v $ROOT_DIR/orbslam3/orbslam3_interfaces/:/root/colcon_ws/src/orbslam3_interfaces \
              -v $ROOT_DIR/orbslam3/ORB_SLAM3:/root/ORB_SLAM3-dev \
              --device=/dev/bus/usb:/dev/bus/usb \
              $REGISTRY/$IMAGE:$VERSION \
              bash
fi
