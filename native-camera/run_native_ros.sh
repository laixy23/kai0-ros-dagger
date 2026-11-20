#!/usr/bin/env bash
# Select the isolated official ROS1 camera overlay; run only the requested command.
set -euo pipefail
if [[ $# -eq 0 ]]; then
  echo "Usage: $0 command [arguments...]" >&2
  exit 2
fi
DAGGER_ROOT="${DAGGER_ROOT:-/home/rie/dev/xingyu.lai/kai0-official-dagger}"
export DAGGER_ROOT
exec bash "${DAGGER_ROOT}/ros-launch/run_ros.sh" bash -c '
  set -eo pipefail
  source "${DAGGER_ROOT}/native-camera/ros_ws/devel/setup.bash"
  export LD_LIBRARY_PATH="${DAGGER_ROOT}/native-camera/sdk-2.56.2/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
  exec "$@"
' native-camera "$@"
