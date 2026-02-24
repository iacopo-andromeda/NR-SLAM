set -e

BUILD_DIR="build"
PARALLEL=8

show_help() {
	cat <<EOF
Usage: ./build.sh [--help] [--all] [--clean] [--target <name>]

Options:
  --help            Show this help and available targets.
  --all             Build all targets (default when no action is provided).
  --clean           Clean build outputs.
  --target <name>   Build a specific target.

Examples:
  ./build.sh --all
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

mkdir -p "${BUILD_DIR}"
cd "${BUILD_DIR}"

cmake ..

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