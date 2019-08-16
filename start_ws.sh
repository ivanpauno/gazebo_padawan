#! /bin/bash
SCRIPT_PATH=$(dirname "$(readlink -f "$0")")

xhost +
docker run --privileged --rm \
       -e DISPLAY=${DISPLAY} \
       --device=/dev/video0:/dev/video0 \
       --device=/dev/dri:/dev/dri \
       -v $SCRIPT_PATH:$SCRIPT_PATH \
       -v /home/ivanpauno/.ssh:/home/ivanpauno/.ssh \
       -v /tmp/.X11-unix:/tmp/.X11-unix:ro $@ \
       -it gazebo9-kinetic:v1 /bin/bash
