# Fetch VR

## Getting Started

### Prerequisites

#### Install Git LFS

This repository uses [Git Large File Storage (LFS)](https://git-lfs.github.com/) to manage and store files such as audio, video and graphics. The aim is to reduce the impact of large files in repositories. To download this repository you must first ensure that Git LFS is installed. Instructions for Debian-based operating systems can be found below, otherwise comprehensive instructions can be found in the [documentation](https://git-lfs.github.com/).

```bash
sudo apt-get update
sudo apt-get install git-lfs
git lfs install
```

#### Install ROS Melodic

The ROS packages require ROS Melodic to be installed. ROS Melodic requires Ubuntu Bionic and can be installed by following the instructions in [the ROS documentation](http://wiki.ros.org/melodic/Installation/Ubuntu). In addition, you must install the following ROS packages on Ubuntu.

```
ros-melodic-desktop
ros-melodic-fetch-auto-dock-msgs
ros-melodic-fetch-driver-msgs
ros-melodic-fetch-ros
ros-melodic-fetch-simulation
ros-melodic-vision-msgs
```

### Downloads

Most free Git providers limited storage and bandwidth for Git LFS. For example, GitHub provides [1 GB of storage and bandwidth per month](https://docs.github.com/en/repositories/working-with-files/managing-large-files/about-storage-and-bandwidth-usage) with a [$5/month upgrade for an addition 50 GB of storage and bandwidth](https://docs.github.com/en/billing/managing-billing-for-git-large-file-storage/upgrading-git-large-file-storage). GitLab offers [10 GB of storage with no bandwidth limit](https://docs.gitlab.com/ee/user/admin_area/settings/account_and_limit_settings.html#repository-size-limit), however only offer a [$60/year upgrade for an additional 10 GB](https://about.gitlab.com/pricing/licensing-faq/#can-i-buy-more-storage).

For now, as the repository is hosted on GitHub, it is important to only clone files in Git LFS when required. This will reduce the time to download the repository, but also help to reduce the bandwidth usage.

There may be methods to reduce cost through the use of a local Git LFS server which can push and pull files from cloud object storage such as [Amazon S3](https://aws.amazon.com/s3/). Some of these alternatives are [Giftless](https://github.com/datopian/giftless) and [Rudolfs](https://github.com/jasonwhite/rudolfs/), but a comprehensive list can be found at the [Git LFS repository](https://github.com/git-lfs/git-lfs/wiki/Implementations).

#### Clone with LFS files

```bash
git clone --recurse-submodules git@github.com:scottwillmoore/research
```

#### Clone without LFS files

```bash
GIT_LFS_SKIP_SMUDGE=1 git clone --recurse-submodules git@github.com:scottwillmoore/research
```
