#!/bin/bash

if [ "$1" = "" ]; then
	echo "please specify image name"
	exit 1
else
	IMAGE_NAME=$1
fi

rocker --nvidia --x11 --network=host $IMAGE_NAME
