# Fetch VR

## Overview

This repository contains all assets and code used to develop a VR experience on the [Oculus Quest 2](https://www.oculus.com/quest-2/) to control the [Fetch Mobile Manipulator](https://fetchrobotics.com/fetch-mobile-manipulator/). The goal is to explore various methods to control and visualise the robot and develop research that will quantify these results through user studies.

https://user-images.githubusercontent.com/9512557/155872489-d8b26750-9ad9-4818-9002-155b84904401.mp4

## Getting Started

### Install Git LFS

This repository uses [Git Large File Storage (LFS)](https://git-lfs.github.com/) to manage and store files such as audio, video and graphics. The aim is to reduce the impact of large files in repositories. To download this repository you must first ensure that Git LFS is installed. Instructions for Debian-based operating systems can be found below, otherwise comprehensive instructions can be found in the documentation.

```bash
sudo apt-get update
sudo apt-get install git-lfs
git lfs install
```

Most free Git providers limited storage and bandwidth for Git LFS. For example, GitHub provides [1 GB of storage and bandwidth per month](https://docs.github.com/en/repositories/working-with-files/managing-large-files/about-storage-and-bandwidth-usage) with a [$5/month upgrade for an addition 50 GB of storage and bandwidth](https://docs.github.com/en/billing/managing-billing-for-git-large-file-storage/upgrading-git-large-file-storage). GitLab offers [10 GB of storage with no bandwidth limit](https://docs.gitlab.com/ee/user/admin_area/settings/account_and_limit_settings.html#repository-size-limit), however only offer a [$60/year upgrade for an additional 10 GB](https://about.gitlab.com/pricing/licensing-faq/#can-i-buy-more-storage).

For now, as the repository is hosted on GitHub, it is important to only clone files in Git LFS when required. This will reduce the time to download the repository, but also help to reduce the bandwidth usage.

There may be methods to reduce cost through the use of a local Git LFS server which can push and pull files from cloud object storage such as [Amazon S3](https://aws.amazon.com/s3/). Some of these alternatives are [Giftless](https://github.com/datopian/giftless) and [Rudolfs](https://github.com/jasonwhite/rudolfs/), but a comprehensive list can be found at the [Git LFS repository](https://github.com/git-lfs/git-lfs/wiki/Implementations).

**Clone with LFS**

```bash
git clone --recurse-submodules git@github.com:scottwillmoore/fetch_vr
```

**Clone without LFS**

```bash
GIT_LFS_SKIP_SMUDGE=1 git clone --recurse-submodules git@github.com:scottwillmoore/fetch_vr
```

### Install ROS

The ROS packages require ROS Melodic to be installed. ROS Melodic requires Ubuntu Bionic and can be installed by following the instructions in [the ROS documentation](http://wiki.ros.org/melodic/Installation/Ubuntu). In addition, you must install the following ROS packages using APT on Ubuntu.

```
ros-melodic-desktop
ros-melodic-fetch-auto-dock-msgs
ros-melodic-fetch-driver-msgs
ros-melodic-fetch-ros
ros-melodic-fetch-simulation
```

### Install Unity Hub

It is best to install [Unity Hub](https://unity3d.com/get-unity/download) which can be used to download the correct version of Unity for the project. It can also be installed with the _Game Development for Unity_ workload from the [Visual Studio 2022 installer](https://visualstudio.microsoft.com/vs/).

### Create an Oculus account

It is best to [create an unmerged Oculus developer account](https://developer.oculus.com/sign-up/) with your Monash email address. This does not require it to be linked to a Facebook account. This link may not work if you are logged in with Facebook, so it you may need to open it in an incognito window.

### Install Oculus Developer Hub for Windows

To develop on the device I would highly recommend Windows. This is the only (actually) supported platform. I believe it was possible on a Mac, and is maybe possible on Linux, but it is not really supported.

The [Oculus Developer Hub for Windows](https://developer.oculus.com/downloads/package/oculus-developer-hub-win/) is not well known, but it allows you to setup an Oculus device without the Oculus app for you phone.

Install the Oculus Developer Hub for Windows. This will allow you to manage the device from your computer. It also allows you to enable developer mode without the app installed on your phone.

Login to your unmerged Oculus developer account and connect to the headset.

### Install Oculus Link for Windows

Install [Oculus Link for Windows](https://www.oculus.com/setup/). I believe this is required if you want to quick preview (press the play button) in Unity.

Login to your unmerged Oculus developer account and connect to the headset.

## Documentation

The project is divided into three directories: `docker`, `ros` and `unity`. Each directory contains additional documentation and insight that may be useful for other developers that work on similar projects.

- The `docker` directory contains an experimental [Docker](https://www.docker.com/) container that has been successfully used on Windows to develop with ROS. It is designed to run with the new [Windows Subsystem for Linux GUI](https://github.com/microsoft/wslg).

- The `ros` directory contains all the scripts that are required to run in the Docker development environment through simulation or on the real robot.

- The `unity` folder contains the Unity project which connects to the ROS master using the [ROS TCP Connector](https://github.com/Unity-Technologies/ROS-TCP-Connector) package. It interacts with the VR headset and has been tested with an Oculus Quest 2.

## License

All assets and code are licensed under the [MIT License](./LICENSE).
