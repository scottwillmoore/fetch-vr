#!/usr/bin/env bash

docker container run \
	--device /dev/dxg \
	--env DISPLAY="$DISPLAY" \
	--env LD_LIBRARY_PATH="/usr/lib/wsl/lib" \
	--env PULSE_SERVER="$PULSE_SERVER" \
	--env WAYLAND_DISPLAY="$WAYLAND_DISPLAY" \
	--env XDG_RUNTIME_DIR="$XDG_RUNTIME_DIR" \
	--gpus all \
	--interactive \
	--rm \
	--tty \
	--user scott \
	--volume /mnt/wslg:/mnt/wslg \
	--volume /tmp/.X11-unix:/tmp/.X11-unix \
	--volume /usr/lib/wsl:/usr/lib/wsl \
	--workdir /home/scott \
	fetch
