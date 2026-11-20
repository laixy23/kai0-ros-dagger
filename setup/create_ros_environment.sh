#!/usr/bin/env bash
set -euo pipefail
ROOT=${DAGGER_ROOT:-/home/rie/dev/xingyu.lai/kai0-official-dagger}
if [[ -x "$ROOT/.ros-env/bin/python" ]]; then
    echo "Existing ROS environment: $ROOT/.ros-env"
    exit 0
fi
"$ROOT/tools/micromamba" create --yes --root-prefix "$ROOT/.mamba" \
    --prefix "$ROOT/.ros-env" --override-channels \
    --channel robostack-noetic --channel conda-forge --strict-channel-priority \
    'python=3.11' 'numpy=1.26.4' 'ros-distro-mutex=0.6' \
    'libopencv=4.10.*=headless*' 'py-opencv=4.10.*=headless*' \
    'ros-noetic-cv-bridge=1.16.2=np126py311ha538b22_23' \
    ros-noetic-ros-base ros-noetic-tf ros-noetic-control-msgs ros-noetic-std-srvs \
    ros-noetic-realsense2-camera ros-noetic-realsense2-description \
    catkin_tools 'cmake<4' 'empy<4' make cxx-compiler pip \
    'h5py=3.12.1' 'av=14.2.0' python-can typing_extensions msgpack-python pillow
cp "$ROOT/setup/ros-env-pins.txt" "$ROOT/.ros-env/conda-meta/pinned"
