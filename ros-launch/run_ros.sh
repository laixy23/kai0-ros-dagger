#!/usr/bin/env bash
# Enter the isolated RoboStack ROS1 environment, then run the exact command.
set -eo pipefail

if [[ $# -eq 0 ]]; then
  echo "Usage: $0 command [arguments...]" >&2
  echo "Examples: $0 roscore | $0 roslaunch /absolute/path/to/cameras_wj_dev.launch" >&2
  exit 2
fi

export DAGGER_ROOT="${DAGGER_ROOT:-/home/rie/dev/xingyu.lai/kai0-official-dagger}"
export ROS_ENV_PREFIX="${ROS_ENV_PREFIX:-${DAGGER_ROOT}/.ros-env}"
export MAMBA_ROOT_PREFIX="${MAMBA_ROOT_PREFIX:-${DAGGER_ROOT}/.mamba}"
MICROMAMBA_BIN="${MICROMAMBA_BIN:-${DAGGER_ROOT}/tools/micromamba}"
export ROS_WORKSPACE_SETUP="${ROS_WORKSPACE_SETUP:-${DAGGER_ROOT}/ros_ws/devel/setup.bash}"
KAI0_SOURCE_ROOT="${KAI0_SOURCE_ROOT:-/home/rie/dev/kai0-task-a}"

if [[ ! -x "${MICROMAMBA_BIN}" || ! -x "${ROS_ENV_PREFIX}/bin/python" ]]; then
  echo "[ERR] micromamba or ROS Python is unavailable under ${DAGGER_ROOT}" >&2
  exit 1
fi
if [[ "${DAGGER_SKIP_WORKSPACE:-0}" != 1 && ! -f "${ROS_WORKSPACE_SETUP}" ]]; then
  echo "[ERR] build/source workspace is missing: ${ROS_WORKSPACE_SETUP}" >&2
  echo "Use DAGGER_SKIP_WORKSPACE=1 only to initialize/build the workspace." >&2
  exit 1
fi

# The policy service keeps its own JAX/CUDA venv. Never source it in this shell.
# Discard inherited venv, system-ROS and Python/C++ library search paths before
# activating the ROS environment, whose NumPy/cv2/cv_bridge ABIs must agree.
export PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
unset PYTHONHOME PYTHONPATH VIRTUAL_ENV __PYVENV_LAUNCHER__ LD_LIBRARY_PATH
unset CONDA_PREFIX CONDA_DEFAULT_ENV CONDA_PROMPT_MODIFIER CONDA_SHLVL CONDA_EXE
unset _CE_CONDA _CE_M AMENT_PREFIX_PATH CMAKE_PREFIX_PATH ROS_PACKAGE_PATH ROS_ROOT ROS_DISTRO
export PYTHONNOUSERSITE=1
eval "$("${MICROMAMBA_BIN}" shell hook --shell bash)"
micromamba activate "${ROS_ENV_PREFIX}"

if [[ "${DAGGER_SKIP_WORKSPACE:-0}" != 1 ]]; then
  # This is the isolated catkin overlay, never the model venv or /opt/ros.
  source "${ROS_WORKSPACE_SETUP}"
fi
export PYTHONPATH="${KAI0_SOURCE_ROOT}/packages/openpi-client/src${PYTHONPATH:+:${PYTHONPATH}}"
export ROS_MASTER_URI="${ROS_MASTER_URI:-http://127.0.0.1:11311}"
export NO_PROXY="127.0.0.1,localhost${NO_PROXY:+,${NO_PROXY}}"
export no_proxy="127.0.0.1,localhost${no_proxy:+,${no_proxy}}"
export TZ=Asia/Shanghai
export ROS_LOG_DIR="${DAGGER_ROOT}/logs/ros"
mkdir -p -- "${ROS_LOG_DIR}"

for argument in "$@"; do
  case "${argument}" in
    *arms_official_wj_dev.launch|*start_ms_piper_new.launch)
      echo "[MOTION] Arm initialization may enable motors and close follower grippers; managed masters require prepared enabled CAN1." >&2
      echo "[STOP] Ctrl+C stops the process; it does not disable the motors. Keep the on-site emergency stop available." >&2
      break
      ;;
  esac
done

# A stale catkin overlay must not reintroduce the model venv's site-packages.
"${ROS_ENV_PREFIX}/bin/python" - <<'PY'
from pathlib import Path
import os
import sys

prefix = Path(os.environ['ROS_ENV_PREFIX']).resolve()
workspace = Path(os.environ['ROS_WORKSPACE_SETUP']).resolve().parent
if Path(sys.prefix).resolve() != prefix:
    raise SystemExit('[ERR] ROS environment Python prefix does not match ROS_ENV_PREFIX')
for entry in sys.path:
    path = Path(entry or '.').resolve()
    if ({'site-packages', 'dist-packages'}.intersection(path.parts) and
            not any(root == path or root in path.parents for root in (prefix, workspace))):
        raise SystemExit('[ERR] foreign site-packages leaked into ROS Python: ' + str(path))
PY
exec "$@"
