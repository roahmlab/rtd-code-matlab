#!/usr/bin/env bash

## Configuration of script vars
NAME="armour-cuda-planner"
USE_UNIQUE=true
ADD_UNAME=true
IMAGE=rtd-code/armour_force_cuda_planner:latest
PORT=65535

if $USE_UNIQUE;then
    NAME+="-$(cat /proc/sys/kernel/random/uuid)"
fi
if $ADD_UNAME;then
    NAME="$(id -un)-$NAME"
fi

## Build out the Docker options
DOCKER_OPTIONS=""
DOCKER_OPTIONS+="-it "
DOCKER_OPTIONS+="--rm "
DOCKER_OPTIONS+="--gpus all "
#DOCKER_OPTIONS+="--net=host "
DOCKER_OPTIONS+="-p 127.0.0.1:${PORT}:65535 "
DOCKER_OPTIONS+="--mount type=tmpfs,destination=/armour/buffer "
DOCKER_OPTIONS+="--name $NAME "

## RUN
docker run $DOCKER_OPTIONS $IMAGE
