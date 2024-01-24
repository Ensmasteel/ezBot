#!/bin/bash

# build a multi-arch image
docker build --platform linux/amd64 -t vincida/ezbot:testing-x86 -f ./dockerfile-x86 . --push
docker build --platform linux/arm64 -t vincida/ezbot:testing-arm -f ./dockerfile-arm64 . --push

