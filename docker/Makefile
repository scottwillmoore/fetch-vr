DOCKER_TAG := scottwillmoore/ros

build:
	docker image build \
		--build-arg LANGUAGE="en_US" \
		--build-arg ROS_DISTRIBUTION="melodic" \
		--build-arg ROS_PACKAGE="desktop" \
		--build-arg TIMEZONE="Australia/Melbourne" \
		--build-arg UBUNTU_DISTRIBUTION="bionic" \
		--build-arg UBUNTU_MIRROR="au" \
		--tag ${DOCKER_TAG} \
		.

run:
	docker container run \
		--device /dev/dxg \
		--env DISPLAY="${DISPLAY}" \
		--env LD_LIBRARY_PATH="/usr/lib/wsl/lib" \
		--env PULSE_SERVER="${PULSE_SERVER}" \
		--env WAYLAND_DISPLAY="${WAYLAND_DISPLAY}" \
		--env XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR}" \
		--gpus all \
		--interactive \
		--mount source=/mnt/wslg,target=/mnt/wslg,type=bind \
		--mount source=/tmp/.X11-unix,target=/tmp/.X11-unix,type=bind \
		--mount source=/usr/lib/wsl,target=/usr/lib/wsl,type=bind \
		--network host \
		--rm \
		--tty \
		${DOCKER_TAG}
