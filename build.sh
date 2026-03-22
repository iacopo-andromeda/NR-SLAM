#!/usr/bin/env zsh

set -e

BUILD_DIR="build"
PARALLEL=8
BUILD_TYPE="Release"

ensure_ros_setup() {
	local profile_path="${HOME}/.andromeda_profile"

	if ! command -v ros_setup >/dev/null 2>&1 && [ -f "${profile_path}" ]; then
		# `ros_setup` is defined in the user's Andromeda shell profile, but VS Code
		# tasks run in a fresh non-interactive shell that does not load shell
		# functions automatically.
		source "${profile_path}"
	fi

	if command -v ros_setup >/dev/null 2>&1; then
		echo "Initializing ROS environment with ros_setup..."
		if ! ros_setup; then
			echo "Warning: ros_setup failed. Continuing build without ROS setup."
		fi
	fi
}

show_help() {
	cat <<EOF
Usage: ./build.sh [--help] [--all] [--clean] [--dbg] [--target <name>]

Options:
  --help            Show this help and available targets.
  --all             Build all targets (default when no action is provided).
  --clean           Clean build outputs.
	--dbg             Configure and build with debug symbols (CMAKE_BUILD_TYPE=Debug).
  --target <name>   Build a specific target.

Examples:
  ./build.sh --all
	./build.sh --dbg --all
  ./build.sh --target nr_slam_mapping
  ./build.sh --clean
EOF
}

ACTION=""
TARGET=""

while [ "$#" -gt 0 ]; do
	case "$1" in
		--help)
			ACTION="help"
			shift
			;;
		--all)
			ACTION="all"
			shift
			;;
		--clean)
			ACTION="clean"
			shift
			;;
		--dbg)
			BUILD_TYPE="Debug"
			shift
			;;
		--target)
			if [ -z "$2" ]; then
				echo "Missing value for --target"
				exit 1
			fi
			ACTION="target"
			TARGET="$2"
			shift 2
			;;
		*)
			echo "Unknown option: $1"
			show_help
			exit 1
			;;
	esac
done

if [ -z "${ACTION}" ]; then
	ACTION="all"
fi

# Initialize ROS environment when available (needed for ROS-dependent targets
# such as andromeda/cpp_bag_reader).
ensure_ros_setup

mkdir -p "${BUILD_DIR}"
cd "${BUILD_DIR}"

cmake -DCMAKE_BUILD_TYPE="${BUILD_TYPE}" ..

case "${ACTION}" in
	help)
		show_help
		cmake --build . --target help
		;;
	all)
		cmake --build . --parallel ${PARALLEL}
		;;
	clean)
		cmake --build . --target clean
		;;
	target)
		cmake --build . --target "${TARGET}" --parallel ${PARALLEL}
		;;
	*)
		echo "Unknown action: ${ACTION}"
		exit 1
		;;
esac