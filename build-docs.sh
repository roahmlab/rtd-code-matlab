#!/usr/bin/env bash

## First build the docs container
docker build -t roahmlab/matlab-sphinx - < Dockerfile.docs

## Configuration for script vars
SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
MOUNT_DIR=$SCRIPT_DIR
STARTING_DIR=$SCRIPT_DIR
NAME="matlab-sphinx"
USE_UNIQUE=true
ADD_UNAME=true
#IMAGE="mathworks/matlab:r2022b"
IMAGE="roahmlab/matlab-sphinx:latest"
if $USE_UNIQUE;then
    NAME+="-$(cat /proc/sys/kernel/random/uuid)"
fi
if $ADD_UNAME;then
    NAME="$(id -un)-$NAME"
fi

## Setup uid requirements and workdir for temporaries
if [ -z "$HOME" ];then
    HOME=/tmp
fi
if [ -z "$ID" ];then
    ID=$(id -u)
fi
WORKDIR=$HOME/.docker
mkdir -p $WORKDIR
getent passwd $(id -u) > $WORKDIR/.$ID.passwd
getent group $(id -g) > $WORKDIR/.$ID.group
DOCKER_HOME=$WORKDIR/$NAME
mkdir -p $DOCKER_HOME

## Build out the Docker options
DOCKER_OPTIONS=""
DOCKER_OPTIONS+="-it "
DOCKER_OPTIONS+="--rm "

## USER ACCOUNT STUFF
DOCKER_OPTIONS+="--user $(id -u):$(id -g) "
DOCKER_OPTIONS+="$(id --groups | sed 's/\(\b\w\)/--group-add \1/g') "
DOCKER_OPTIONS+="-v $WORKDIR/.$ID.passwd:/etc/passwd:ro "
DOCKER_OPTIONS+="-v $WORKDIR/.$ID.group:/etc/group:ro "

## PROJECT
DOCKER_OPTIONS+="-v $MOUNT_DIR:/docs "
DOCKER_OPTIONS+="-v $DOCKER_HOME:$HOME "
DOCKER_OPTIONS+="--name $NAME "
DOCKER_OPTIONS+="--workdir=/docs/docs "
DOCKER_OPTIONS+="--entrypoint make "
DOCKER_OPTIONS+="--net=host "


## RUN
docker run $DOCKER_OPTIONS $IMAGE $1

## CLEANUP
rm -rf $DOCKER_HOME

