#!/bin/bash

# clear

docker build --load -t coverage_path_planner .

docker stop coverage_path_planner

docker rm coverage_path_planner

docker run -it \
--env-file ${HOME}/.cognicept/runtime.env \
--network=host \
--name=coverage_path_planner \
coverage_path_planner