#!/bin/bash

docker run -it --rm \
--network=host \
--ipc=host --pid=host \
--env UID=$(id -u) \
--env GID=$(id -g) \
-e DISPLAY=$DISPLAY \
-v .:/ezbot \
vincida/ezbot:testing-x86
