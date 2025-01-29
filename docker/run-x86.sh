#!/bin/bash

# give permission to connect to X server
xhost +

docker run -it --rm \
--network=host \
--ipc=host --pid=host \
--env UID=$(id -u) \
--env GID=$(id -g) \
-e DISPLAY=$DISPLAY \
-v .:/ezbot \
ezbot:testing-x86
#vincida/ezbot:testing-x86
