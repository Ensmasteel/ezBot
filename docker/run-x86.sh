#!/bin/bash

docker run -it --rm \
--network=host \
--ipc=host --pid=host \
--env UID=$(id -u) \
--env GID=$(id -g) \
-v .:/ezbot \
--workdir /ezbot \
ezbot-amd64