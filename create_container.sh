xhost local:root
XAUTH=/tmp/.docker.xauth

docker run -it \
    --name=prac_robot \
    --env="DISPLAY=$DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp./X11-unix:/tmp/.X11-unix:rw" \
    --env="XAUTHORITY=$XAUTH" \
    --volume="$XAUTH:$XAUTH" \
    --net=host \
    --mount type=bind,source="$(pwd)"/prac_robot,target=/ros2_ws/src/prac_robot \
    --privileged \
    prac_robot_img:latest \
    bash

echo "prac_robot_container created"