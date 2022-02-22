# syntax=docker/dockerfile:labs

ARG UBUNTU_DISTRIBUTION

FROM ubuntu:${UBUNTU_DISTRIBUTION}

RUN <<EOF
mv /etc/apt/apt.conf.d/docker-clean /etc/apt
echo 'Binary::apt::APT::Keep-Downloaded-Packages "true";' > /etc/apt/apt.conf.d/docker-keep-downloaded-packages
EOF

ARG UBUNTU_MIRROR

RUN <<EOF
sed --in-place --regexp-extended "s/(\/\/)(archive\.ubuntu)/\1${UBUNTU_MIRROR}.\2/" /etc/apt/sources.list
EOF

RUN --mount=type=cache,target=/var/cache/apt --mount=type=cache,target=/var/lib/apt <<EOF
yes | unminimize
EOF

ARG LANGUAGE

RUN --mount=type=cache,target=/var/cache/apt --mount=type=cache,target=/var/lib/apt <<EOF
apt-get update
apt-get install --assume-yes --no-install-recommends locales
sed --in-place --regexp-extended "/${LANGUAGE}.UTF-8/s/^# //g" /etc/locale.gen
dpkg-reconfigure --frontend noninteractive locales
update-locale LANG=${LANGUAGE}.UTF-8
EOF

ENV LANG="${LANGUAGE}.UTF-8"

ENV LC_ALL="${LANGUAGE}.UTF-8"

ARG TIMEZONE

RUN --mount=type=cache,target=/var/cache/apt --mount=type=cache,target=/var/lib/apt <<EOF
apt-get update
apt-get install --assume-yes --no-install-recommends tzdata
ln --force --symbolic /usr/share/zoneinfo/${TIMEZONE} /etc/localtime
dpkg-reconfigure --frontend noninteractive tzdata
EOF

ENV TZ="${TIMEZONE}"

RUN --mount=type=cache,target=/var/cache/apt --mount=type=cache,target=/var/lib/apt <<EOF
apt-get update
apt-get install --assume-yes --no-install-recommends \
	software-properties-common
add-apt-repository ppa:kisak/turtle --yes
apt-get update
apt-get install --assume-yes --no-install-recommends \
	mesa-utils \
	mesa-utils-extra
EOF

RUN --mount=type=cache,target=/var/cache/apt --mount=type=cache,target=/var/lib/apt <<EOF
apt-get update
apt-get install --assume-yes --no-install-recommends \
	bash-completion \
	build-essential \
	curl \
	curl \
	git \
	gnupg \
	gnupg2 \
	less \
	sudo \
	tmux \
	unzip \
	vim \
	wget \
	zip
EOF

ARG ROS_DISTRIBUTION

ARG ROS_PACKAGE

RUN --mount=type=cache,target=/var/cache/apt --mount=type=cache,target=/var/lib/apt <<EOF
curl --location --show-error --silent https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | gpg --dearmor > /etc/apt/trusted.gpg.d/ros-keyring.gpg
echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros.list
apt-get update
apt-get install --assume-yes --no-install-recommends \
	python-rosdep \
	python-rosinstall \
	python-rosinstall-generator \
	python-wstool \
	ros-${ROS_DISTRIBUTION}-${ROS_PACKAGE}
rosdep init
rosdep update
EOF

RUN --mount=type=cache,target=/var/cache/apt --mount=type=cache,target=/var/lib/apt <<EOF
apt-get update
apt-get install --assume-yes --no-install-recommends \
	ros-melodic-fetch-auto-dock-msgs \
	ros-melodic-fetch-driver-msgs \
	ros-melodic-fetch-gazebo-demo \
	ros-melodic-fetch-tools \
	ros-melodic-joint-state-publisher-gui
EOF

ADD ./files/ /root/

RUN <<EOF
sed --in-place --regexp-extended "s/ROS_DISTRIBUTION/${ROS_DISTRIBUTION}/g" /root/.bashrc
EOF

COPY ros_entrypoint.sh /

ENTRYPOINT ["/ros_entrypoint.sh"]

CMD ["bash"]

RUN <<EOF
mv /etc/apt/docker-clean /etc/apt/apt.conf.d
rm --force --recursive /etc/apt/apt.conf.d/docker-keep-downloaded-packages
EOF
