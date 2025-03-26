#!/bin/bash

XSOCK=/tmp/.X11-unix
XAUTH=/tmp/.docker.xauth
USER=jfantab

touch $XAUTH
chmod 777 $XAUTH  # Fix permissions for X11 authentication
xauth nlist $DISPLAY | sed -e 's/^..../ffff/' | xauth -f $XAUTH nmerge -

docker run -it --rm --user=$(id -u $USER):$(id -g $USER) \
    --env=DISPLAY=$DISPLAY --env=QT_X11_NO_MITSHM=1 \
    --env=XAUTHORITY=${XAUTH} --volume=/tmp/.X11-unix:/tmp/.X11-unix:rw \
    --volume=$XAUTH:$XAUTH:rw --privileged --volume=/etc/group:/etc/group:ro \
    --volume=/etc/passwd:/etc/passwd:ro --volume=/etc/shadow:/etc/shadow:ro \
    --volume=/etc/sudoers.d:/etc/sudoers.d:ro \
    --volume=/home/$USER/lite3_ws/docker_scripts/script.sh:/home/$USER/lite3_ws/docker_scripts/script.sh \
    --device=/dev/video0:/dev/video0 \
    --net host -v /home:/home -v $HOME/Volumes:/home/user/Volumes -v /dev/shm:/dev/shm \
    --stop-signal SIGINT \
    foxy \
    bash \
    /home/$USER/lite3_ws/docker_scripts/script.sh
    