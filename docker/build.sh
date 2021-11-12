#!/bin/sh

if [ "$1" = "" ]; then
	echo "please specify docker image name."
	exit 1
else
	DOCKER_IMAGE_NAME=$1
fi

if [ "$2" = "" ]; then
	echo "please specify docker file path."
	exit 1
else
	DOCKER_FILE_PATH=$2
fi

DOCKER_BUILDKIT=1 docker build --ssh default -t $DOCKER_IMAGE_NAME -f $DOCKER_FILE_PATH .
