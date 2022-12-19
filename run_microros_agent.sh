#!/bin/bash

docker run -it --privileged \
    --network host \
    --device /dev/ttyUSB0 \
    -v /dev:/dev \
    microros/micro-ros-agent:rolling \
        serial -D /dev/ttyUSB0
#    	udp4 -p 8888 -v6


