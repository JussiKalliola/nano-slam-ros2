docker pull jussikalliola/nano-ros2-orbslam3:latest

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
            jussikalliola/nano-ros2-orbslam3:latest \
            bash
