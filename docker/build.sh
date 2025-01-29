#!/bin/bash

#change working directory to the directory of the script
cd "$(dirname "$0")"

# build a multi-arch image
#docker build --platform linux/amd64 -t vincida/ezbot:testing-x86 -f ./dockerfile-x86 . --push
#docker build --platform linux/arm64 -t vincida/ezbot:testing-arm -f ./dockerfile-arm64 . --push

docker build --platform linux/amd64 -t ezbot:testing-x86 -f ./dockerfile-x86 .
#docker build --platform linux/arm64 -t ezbot:testing-arm -f ./dockerfile-arm64 .
