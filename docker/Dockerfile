FROM ubuntu:20.04

ENV DEBIAN_FRONTEND=noninteractive

SHELL ["/bin/bash", "-c"]

RUN apt update && apt install -y --no-install-recommends vim tmux git python3-pip ssh sudo

WORKDIR /root
RUN --mount=type=ssh \
    mkdir -m 0600 ~/.ssh && ssh-keyscan github.com >> ~/.ssh/known_hosts \
    && git clone git@github.com:choreonoid/choreonoid.git \
    &&  ./choreonoid/misc/script/install-requisites-ubuntu-20.04.sh

WORKDIR /root/choreonoid
RUN mkdir build && cd build && cmake .. && make -j20 && make install

ENV NVIDIA_VISIBLE_DEVICES ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics
