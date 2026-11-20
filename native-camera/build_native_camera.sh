#!/usr/bin/env bash
# Build only. Never enumerates/opens USB, launches ROS nodes, or modifies the OS.
set -euo pipefail
DAGGER_ROOT="${DAGGER_ROOT:-/home/rie/dev/xingyu.lai/kai0-official-dagger}"
NATIVE_ROOT="${DAGGER_ROOT}/native-camera"
SDK_SOURCE="${NATIVE_ROOT}/sources/librealsense-2.56.2"
SDK_PREFIX="${NATIVE_ROOT}/sdk-2.56.2"
ROS_SOURCE="${NATIVE_ROOT}/ros_ws/src/realsense-ros"
ROS_ENV="${DAGGER_ROOT}/.ros-env"
RUN_ROS="${DAGGER_ROOT}/ros-launch/run_ros.sh"
mkdir -p "${NATIVE_ROOT}/sources" "${NATIVE_ROOT}/logs" "${NATIVE_ROOT}/receipts" "${NATIVE_ROOT}/ros_ws/src"
if [[ ! -d "${SDK_SOURCE}/.git" ]]; then
  git clone --depth 1 --branch v2.56.2 https://github.com/realsenseai/librealsense.git "${SDK_SOURCE}"
fi
if [[ ! -d "${ROS_SOURCE}/.git" ]]; then
  git clone --depth 1 --branch 2.3.2 https://github.com/realsenseai/realsense-ros.git "${ROS_SOURCE}"
fi
git -C "${SDK_SOURCE}" rev-parse HEAD > "${NATIVE_ROOT}/receipts/sdk-source-commit.txt"
git -C "${ROS_SOURCE}" rev-parse HEAD > "${NATIVE_ROOT}/receipts/ros-source-commit.txt"
if [[ ! -f "${NATIVE_ROOT}/build-deps/include/libudev.h" ]]; then
  MAMBA_ROOT_PREFIX="${DAGGER_ROOT}/.mamba" "${DAGGER_ROOT}/tools/micromamba" create \
    -p "${NATIVE_ROOT}/build-deps" --override-channels -c conda-forge \
    --strict-channel-priority libudev=262.2 -y
fi
SDK_SOURCE="${SDK_SOURCE}" ROS_SOURCE="${ROS_SOURCE}" python3 - <<'PY'
import os
from pathlib import Path
ros = Path(os.environ['ROS_SOURCE']) / 'realsense2_camera/CMakeLists.txt'
text = ros.read_text()
ros.write_text(text.replace('-std=c++11', '-std=c++14'))
sdk = Path(os.environ['SDK_SOURCE']) / 'CMake/install_config.cmake'
text = sdk.read_text()
sdk.write_text(text.replace('install(CODE "execute_process(COMMAND ldconfig)")',
                           '# Project-local installation uses RPATH; never update the system linker cache.'))
PY

bash "${RUN_ROS}" cmake -S "${SDK_SOURCE}" -B "${NATIVE_ROOT}/sdk-build" \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="${SDK_PREFIX}" \
  -DCMAKE_INSTALL_LIBDIR=lib -DCMAKE_INSTALL_RPATH="${SDK_PREFIX}/lib;${ROS_ENV}/lib" \
  -DFORCE_RSUSB_BACKEND=OFF -DBUILD_WITH_DDS=OFF -DBUILD_WITH_CUDA=OFF \
  -DBUILD_EXAMPLES=OFF -DBUILD_GRAPHICAL_EXAMPLES=OFF -DBUILD_GLSL_EXTENSIONS=OFF \
  -DBUILD_TOOLS=OFF -DBUILD_UNIT_TESTS=OFF -DBUILD_PYTHON_BINDINGS=OFF \
  -DBUILD_RS2_ALL=OFF -DIMPORT_DEPTH_CAM_FW=OFF -DCHECK_FOR_UPDATES=OFF \
  -DUDEV_INCLUDE_DIRS="${NATIVE_ROOT}/build-deps/include" -DUDEV_LIBRARIES="${ROS_ENV}/lib/libudev.so.1" \
  2>&1 | tee "${NATIVE_ROOT}/logs/sdk-configure.log"
CPLUS_INCLUDE_PATH="${NATIVE_ROOT}/build-deps/include" C_INCLUDE_PATH="${NATIVE_ROOT}/build-deps/include" \
  LIBRARY_PATH="${NATIVE_ROOT}/build-deps/lib" \
  bash "${RUN_ROS}" cmake --build "${NATIVE_ROOT}/sdk-build" --parallel 8 \
  2>&1 | tee "${NATIVE_ROOT}/logs/sdk-build.log"
bash "${RUN_ROS}" cmake --install "${NATIVE_ROOT}/sdk-build" \
  2>&1 | tee "${NATIVE_ROOT}/logs/sdk-install.log"

bash "${RUN_ROS}" catkin_make -C "${NATIVE_ROOT}/ros_ws" -j8 \
  -DCMAKE_BUILD_TYPE=Release -Drealsense2_DIR="${SDK_PREFIX}/lib/cmake/realsense2" \
  -DOpenCV_DIR="${ROS_ENV}/lib/cmake/opencv4" -DPYTHON_EXECUTABLE="${ROS_ENV}/bin/python" \
  -DCMAKE_PREFIX_PATH="${SDK_PREFIX};${DAGGER_ROOT}/ros_ws/devel;${ROS_ENV}" \
  -DCMAKE_BUILD_RPATH="${SDK_PREFIX}/lib;${ROS_ENV}/lib" \
  -DCMAKE_INSTALL_RPATH="${SDK_PREFIX}/lib;${ROS_ENV}/lib" \
  2>&1 | tee "${NATIVE_ROOT}/logs/ros-build.log"
git -C "${ROS_SOURCE}" diff > "${NATIVE_ROOT}/receipts/ros-compatibility.patch"
git -C "${SDK_SOURCE}" diff > "${NATIVE_ROOT}/receipts/sdk-compatibility.patch"
ldd "${NATIVE_ROOT}/ros_ws/devel/lib/librealsense2_camera.so" \
  > "${NATIVE_ROOT}/receipts/ros-driver-ldd.txt"
ldd "${SDK_PREFIX}/lib/librealsense2.so" > "${NATIVE_ROOT}/receipts/sdk-ldd.txt"
if grep -q 'not found' "${NATIVE_ROOT}/receipts/ros-driver-ldd.txt" "${NATIVE_ROOT}/receipts/sdk-ldd.txt"; then
  echo "Unresolved dynamic dependencies; inspect native-camera/receipts/*ldd.txt" >&2
  exit 1
fi
python3 "${NATIVE_ROOT}/write_build_receipt.py"
echo "BUILD COMPLETE: isolated native V4L2 SDK and official ROS1 overlay; no camera opened."
