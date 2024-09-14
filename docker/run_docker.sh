#!/bin/bash

PARENT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )"/.. &> /dev/null && pwd )"
GITNAME="gitlab.lrz.de:5005/map_loc"
DOCKERFILE=""
TAG="latest"

# Function to display usage information
usage() {
    echo "Usage: $0 [--name <image_name>] [--tag <image_tag>] [--help]" 1>&2;
    echo "    --gitname <image_git_name>    Set the gitname of the Docker image (default: gitlab.lrz.de:5005/map_loc)"
    echo "    --tag <image_tag>             Set the tag of the Docker image (default: latest)"
    echo "    --help                        Display this help message"
    exit 1;
}


# Parse arguments
while [[ $# -gt 0 ]]; do
    key="$1"
    case $key in
        --gitname)
            GITNAME="$2"
            shift # past argument
            shift # past value
            ;;
        --tag)
            TAG="$2"
            shift # past argument
            shift # past value
            ;;
        --dockerfile)
            DOCKERFILE="$2"
            shift # past argument
            shift # past value
            ;;
        --help)
            usage
            ;;
        *)  # unknown option
            echo "Unknown option: $1"
            exit 1
            ;;
    esac
done

xhost +
docker run --rm -it \
    --network=host \
    -v /dev/shm:/dev/shm \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix/:/tmp/.X11-unix/ \
    --privileged \
    -v $HOME/Documents/datasets:/datasets \
    $GITNAME/$DOCKERFILE:$TAG \
    bash
xhost -