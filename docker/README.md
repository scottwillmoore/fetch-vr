# Docker

## Overview

This directory contains the resources required to build a Docker container on Ubuntu Bionic for ROS Melodic with the required Fetch dependencies installed. The primary purpose is to create a development environment which can be used from within Windows which is required for Unity development.

[Windows Subsystem for Linux](https://docs.microsoft.com/en-us/windows/wsl/) is a new feature for Windows that allows developers on Windows to run a Linux environment from within Windows, with very little overhead. In addition, the [Windows Subsystem for Linux GUI](https://github.com/microsoft/wslg) project integrates Linux GUI applications into the Windows desktop and enables them to be hardware accelerated. This theoretically allows ROS tools such as `gazebo`, `rqt` and `rviz` to be used from within Windows.

As ROS Melodic requires Ubuntu Bionic it is easiest to use a Docker container to make the environment reproducible across machines. In addition, due to the [recent release](https://devblogs.microsoft.com/directx/in-the-works-opencl-and-opengl-mapping-layers-to-directx/) of the [Mesa D3D12 driver](https://docs.mesa3d.org/drivers/d3d12.html) it is only supported on Ubuntu Focal and beyond. Therefore, an updated version of Mesa is required to be installed on Ubuntu Bionic in order to still benefit from hardware accleration.

In addition, it is possible to connect the WSL environment to a non-virtual network, and hence access and control the real robot from within Windows. This is currently only supported with an [external tool](https://github.com/dantmnf/WSLAttachSwitch), or with the [preview version of WSL](https://github.com/microsoft/WSL/issues/4150#issuecomment-1018524753).

## Getting Started

### 1. Install Windows 11

This is required by WSLg.

### 2. Install WSL

You can follow the [tutorial](https://docs.microsoft.com/en-us/windows/wsl/install) provided by Microsoft. You should install Ubuntu Focal and then make sure you update and upgrade your Ubuntu packages.

```bash
sudo apt-get update
sudo apt-get upgrade
```

### 3. Verify WSLg

You may need to consult the [GitHub repository](https://github.com/microsoft/wslg#pre-requisites) and insall the latest graphics drivers to be supported.

```bash
sudo apt-get update
sudo apt-get install mesa-utils

# Verify that the d3d12 driver is used.
glxinfo -B

# Run a simple OpenGL-enabled application.
glxgears
```

### 4. Install Docker in WSL

You can install [Docker Desktop for Windows](https://docs.docker.com/desktop/windows/install/), however this does not work well if you plan to connect your WSL environment with to non-virtual network. In addition, it now requires a paid subscription for enterprises of more than 250 employees.

Instead, I would install [Docker Engine for Ubuntu](https://docs.docker.com/engine/install/ubuntu/) directly in your WSL environment. You can then configure your WSL [boot settings](https://docs.microsoft.com/en-us/windows/wsl/wsl-config#boot-settings) to launch Docker with your WSL environment.

### 5. Build the Docker image

Finally, to build the Docker container you may use the `Makefile` provided in this directory. It provides sensible defaults for a WSL-enabled environment, but should be modified for a non-WSL environment.

```bash
sudo apt-get update
sudo apt-get install make

# Build the docker image.
make build

# Run the docker container.
make run
```

### 6. Enable network access

This step is optional, but if you wish to connect to a real robot you will need to enable access to a physical network from your WSL environment.

As discussed, this is currently only supported with an [external tool](https://github.com/dantmnf/WSLAttachSwitch), or with the [preview version of WSL](https://github.com/microsoft/WSL/issues/4150#issuecomment-1018524753). I believe this requires that [Hyper V](https://docs.microsoft.com/en-us/virtualization/hyper-v-on-windows/about/) is installed which requires a Pro edition of Windows.

## Addendum

In summary, this approach works, however there are still many issues with WSL and WSLg which can make the entire setup far more complicated than is required.

A major issue that currently exists whereby [meshes in `rviz` are not displayed when hardware acceleration is enabled](https://github.com/microsoft/wslg/issues/554). The workaround is to run `rviz` with a software renderer which is not ideal.
