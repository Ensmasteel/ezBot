#!/bin/bash

# change working directory to the directory of the script
cd "$(dirname "$0")"

# parse arguments
NO_CACHE=""

while [[ "$#" -gt 0 ]]; do
  case $1 in
    --no-cache) NO_CACHE="--no-cache"; shift ;;
    *) echo "Unknown parameter passed: $1"; exit 1 ;;
  esac
  shift
done

# build a multi-arch image
#docker build --platform linux/amd64 -t vincida/ezbot:testing-x86 -f ./dockerfile-x86 . --push
#docker build --platform linux/arm64 -t vincida/ezbot:testing-arm -f ./dockerfile-arm64 . --push

docker build --platform linux/amd64 -t ezbot:testing-x86 -f ./dockerfile-x86 $NO_CACHE .
#docker build --platform linux/arm64 -t ezbot:testing-arm -f ./dockerfile-arm64 $NO_CACHE .
