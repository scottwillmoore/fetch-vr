#!/usr/bin/env bash

docker image build \
    --build-arg LANGUAGE="en_US" \
    --build-arg ROS_DISTRIBUTION="melodic" \
    --build-arg ROS_PACKAGE="desktop" \
    --build-arg TIMEZONE="Australia/Melbourne" \
    --build-arg UBUNTU_DISTRIBUTION="bionic" \
    --build-arg UBUNTU_MIRROR="au" \
    --build-arg USERNAME="scott" \
    --progress plain \
    --tag fetch \
    .
