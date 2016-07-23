#!/bin/sh

docker run -ti --rm --net=ros --env ROS_MASTER_URI=http://master:11311 --privileged=true --device=/dev/bus/usb -v /usr/lib/x86_64-linux-gnu/dri:/usr/lib/x86_64-linux-gnu/dri:ro -v /usr/lib/x86_64-linux-gnu/mesa:/usr/lib/x86_64-linux-gnu/mesa:ro -v /usr/lib/x86_64-linux-gnu/libdrm_nouveau.so.2.0.0:/usr/lib/x86_64-linux-gnu/libdrm_nouveau.so.2.0.0:ro --device=/dev/dri -e DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix semio/ros-jade-semio-ros:built rosrun semio_attention_ros semio_attention_node

#docker run -ti --rm --net=ros --env ROS_MASTER_URI=http://master:11311 -e QT_GRAPHICSSYSTEM=native --privileged=true --device=/dev/bus/usb --device=/dev/dri -e DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix semio/ros-jade-semio-ros:built rosrun semio_attention_ros semio_attention_node
