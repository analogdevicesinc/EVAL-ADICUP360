#!/bin/bash

TOP_DIR=${TOP_DIR:-'./'}

command_exists() {
	local cmd=$1
	[ -n "$cmd" ] || return 1
	type "$cmd" >/dev/null 2>&1
}

ensure_command_exists() {
	local cmd="$1"
	local package="$2"
	[ -n "$cmd" ] || return 1
	[ -n "$package" ] || package="$cmd"
	! command_exists "$cmd" || return 0
	# go through known package managers
	for pacman in apt-get brew yum ; do
		command_exists $pacman || continue
		$pacman install -y $package || {
			# Try an update if install doesn't work the first time
			$pacman -y update && \
				$pacman install -y $package
		}
		return $?
	done
	return 1
}

ensure_command_exists wget
ensure_command_exists sudo

# Get the common stuff from no-OS
[ -f ${TOP_DIR}/build/lib.sh ] || {
	mkdir -p ${TOP_DIR}/build
	wget https://raw.githubusercontent.com/analogdevicesinc/no-OS/master/ci/lib.sh \
		-O ${TOP_DIR}/build/lib.sh
}

. ${TOP_DIR}/build/lib.sh

# Call function from ${TOP_DIR}/build/lib.sh to download common scripts
download_common_scripts
