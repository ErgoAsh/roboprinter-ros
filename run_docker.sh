#!/bin/bash

xhost +local:
export WORK_DIR=/home/ergoash/roboprinter_ws

docker run -it --rm \
    --network host \
    --device=/dev/dri \
    --group-add video \
    --env DISPLAY=$DISPLAY \
    --volume /tmp/.X11-unix/:/tmp/.X11-unix \
    --env=QT_X11_NO_MITSHM=1 \
    -v $(pwd):$WORK_DIR/src \
    -w $WORK_DIR/ \
    roboprinter-ros \
    bash
    #/bin/bash -ci "source /root/.bashrc && /bin/bash"
    #ergoash/roboprinter-ros \
    #-m=2G \
