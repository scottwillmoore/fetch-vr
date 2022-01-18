# syntax=docker/dockerfile:labs

ARG UBUNTU_DISTRIBUTION

FROM ubuntu:${UBUNTU_DISTRIBUTION}

# Use Bash as the default shell

SHELL ["/bin/bash", "-c"]

# Allow downloaded packages to be cached

RUN <<EOF
mv /etc/apt/apt.conf.d/docker-clean /etc/apt
echo 'Binary::apt::APT::Keep-Downloaded-Packages "true";' > /etc/apt/apt.conf.d/docker-keep-downloaded-packages
EOF

# Update the default Ubuntu mirror

ARG UBUNTU_MIRROR

RUN <<EOF
sed --in-place --regexp-extended "s/(\/\/)(archive\.ubuntu)/\1${UBUNTU_MIRROR}.\2/" /etc/apt/sources.list
EOF

# Unminimize the Ubuntu image

RUN --mount=type=cache,target=/var/cache/apt --mount=type=cache,target=/var/lib/apt <<EOF
yes | unminimize
EOF

# Update the default locale

ARG LANGUAGE

RUN --mount=type=cache,target=/var/cache/apt --mount=type=cache,target=/var/lib/apt <<EOF
apt-get update
apt-get install --assume-yes --no-install-recommends locales
EOF

RUN <<EOF
sed --in-place --regexp-extended "/${LANGUAGE}.UTF-8/s/^# //g" /etc/locale.gen
dpkg-reconfigure --frontend noninteractive locales
update-locale LANG=${LANGUAGE}.UTF-8
EOF
	
ENV LANG="${LANGUAGE}.UTF-8"

ENV LC_ALL="${LANGUAGE}.UTF-8"

# Update the default timezone

ARG TIMEZONE

RUN --mount=type=cache,target=/var/cache/apt --mount=type=cache,target=/var/lib/apt <<EOF
apt-get update
apt-get install --assume-yes --no-install-recommends tzdata
EOF

RUN <<EOF
ln --force --symbolic /usr/share/zoneinfo/${TIMEZONE} /etc/localtime
dpkg-reconfigure --frontend noninteractive tzdata
EOF
	
ENV TZ="${TIMEZONE}"

# Install latest Mesa for Windows Subsystem for Linux

RUN --mount=type=cache,target=/var/cache/apt --mount=type=cache,target=/var/lib/apt <<EOF
apt-get update
apt-get install --assume-yes --no-install-recommends software-properties-common
add-apt-repository ppa:kisak/turtle --yes
apt-get update
apt-get install --assume-yes --no-install-recommends mesa-utils
EOF

# Install ROS packages

RUN --mount=type=cache,target=/var/cache/apt --mount=type=cache,target=/var/lib/apt <<EOF
apt-get update
apt-get install --assume-yes --no-install-recommends curl gnupg2
EOF

ARG ROS_DISTRIBUTION

ARG ROS_PACKAGE

RUN --mount=type=cache,target=/var/cache/apt --mount=type=cache,target=/var/lib/apt <<EOF
curl --location --show-error --silent https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | gpg --dearmor > /etc/apt/trusted.gpg.d/ros-keyring.gpg
echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros.list
apt-get update
apt-get install --assume-yes --no-install-recommends ros-${ROS_DISTRIBUTION}-${ROS_PACKAGE}
EOF

# Install ROS development packages

RUN --mount=type=cache,target=/var/cache/apt --mount=type=cache,target=/var/lib/apt <<EOF
apt-get update
apt-get install --assume-yes --no-install-recommends build-essential python-rosdep python-rosinstall python-rosinstall-generator python-wstool
rosdep init
rosdep update
EOF

# Install Fetch packages

RUN --mount=type=cache,target=/var/cache/apt --mount=type=cache,target=/var/lib/apt <<EOF
apt-get update
apt-get install --assume-yes --no-install-recommends ros-${ROS_DISTRIBUTION}-fetch-gazebo-demo ros-melodic-joint-state-publisher-gui
EOF

# Install developer packages

RUN --mount=type=cache,target=/var/cache/apt --mount=type=cache,target=/var/lib/apt <<EOF
apt-get update
apt-get install --assume-yes --no-install-recommends bash-completion curl less git git-lfs sudo tmux unzip vim wget zip
bash -c "$(curl --location --show-error --silent https://starship.rs/install.sh)" -- --yes
EOF

# Disallow downloaded packages to be cached

RUN <<EOF
mv /etc/apt/docker-clean /etc/apt/apt.conf.d
rm --force --recursive /etc/apt/apt.conf.d/docker-keep-downloaded-packages
EOF

# Create a user account

ARG USERNAME

RUN <<EOF
groupadd ${USERNAME}
useradd --create-home --gid ${USERNAME} --shell /bin/bash --skel /dev/null ${USERNAME}
echo "$USERNAME ALL=(ALL:ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME}
chmod 440 /etc/sudoers.d/${USERNAME}
EOF

WORKDIR /home/${USERNAME}

USER ${USERNAME}

ADD ./files/ ./

RUN <<EOF
sed --in-place --regexp-extended "s/ROS_DISTRIBUTION/${ROS_DISTRIBUTION}/g" ./.bashrc
EOF

# Setup a ROS workspace

# NOTE: A better way to do this may be to setup a development container...
# https://code.visualstudio.com/docs/remote/containers

# RUN <<EOF
# source /opt/ros/${ROS_DISTRIBUTION}/setup.bash
# mkdir --parents ./workspace/src
# pushd ./workspace/src
# git lfs install
# GIT_LFS_SKIP_SMUDGE=1 git clone --recurse-submodules git@github.com:scottwillmoore/research ./workspace/src/
# popd
# catkin_make
# EOF
