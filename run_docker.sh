#!/bin/bash

xhost +local:
export WORK_DIR=/home/ergoash/roboprinter_ws

docker run -it --rm \
    --network host \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix/:/tmp/.X11-unix \
    -v $(pwd):$WORK_DIR/src \
    -w $WORK_DIR/ \
    ergoash/roboprinter-ros \
    bash
    #/bin/bash -ci "source /root/.bashrc && /bin/bash"
