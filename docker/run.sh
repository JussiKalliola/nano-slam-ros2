REGISTRY=jussikalliola
IMAGE=nano-ros2-orbslam3
VERSION=latest
USE_CACHE=1
SYSTEM_ARCH=$(uname -m)

while getopts v: flag
do
    case "${flag}" in
        v) VERSION=${OPTARG};;
    esac
done

if [ -z "${VERSION}" ]; 
then
  echo "No version defined. DEFAULT: latest"
  VERSION=latest
else
  echo "Version selected: ${VERSION}"
fi


echo "Pulling an image: ${REGISTRY}/${IMAGE}:${VERSION}"
docker pull $REGISTRY/$IMAGE:$VERSION

xhost +
docker run  -it \
            --rm \
	          --net=host \
	          --cap-add NET_ADMIN \
	          --privileged \
            -e DISPLAY=$DISPLAY \
            -e XDG_RUNTIME_DIR=$XDG_RUNTIME_DIR \
            -e PYTHONBUFFERED=1 \
            -v /etc/timezone:/etc/timezone:ro \
            -v /etc/localtime:/etc/localtime:ro \
            -v $PWD/../../ros_ws/:/root/ros_ws:rw \
            -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
            -v $HOME/.Xauthority:/root/.Xauthority \
            -v $HOME/.tmux/:/root/.tmux \
            -v $HOME/.config/:/root/.config \
            -v /run/user/1000:/run/user/1000 \
            -v $HOME/nano-slam-ros2/datasets/:/root/datasets \
            --device=/dev/bus/usb:/dev/bus/usb \
            $REGISTRY/$IMAGE:$VERSION \
            bash
