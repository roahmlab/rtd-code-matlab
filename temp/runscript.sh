#!/bin/bash

docker run --rm -it --gpus all --net=host --mount type=tmpfs,destination=/armour/buffer rtd-code/armour_cuda_planner:latest
