#!/usr/bin/env bash
set -euo pipefail
ROOT=${DAGGER_ROOT:-/home/rie/dev/xingyu.lai/kai0-official-dagger}
UPSTREAM=${KAI0_SOURCE_ROOT:-/home/rie/dev/kai0-task-a}
unset PYTHONPATH PYTHONHOME VIRTUAL_ENV CONDA_PREFIX LD_LIBRARY_PATH
export PYTHONNOUSERSITE=1
export MAMBA_ROOT_PREFIX="$ROOT/.mamba"
set +u
eval "$("$ROOT/tools/micromamba" shell hook --shell bash --root-prefix "$ROOT/.mamba")"
micromamba activate "$ROOT/.ros-env"
set -u

python -m pip install --index-url https://download.pytorch.org/whl/cpu 'torch==2.7.1'
python -m pip install 'python-can==4.6.1' 'dm-env==1.6' 'dm-tree==0.1.9' 'websockets==15.0.1'
python -m pip install --no-index --find-links "$ROOT/firmware-compat/artifacts" \
    --no-deps --require-hashes -r "$ROOT/firmware-compat/requirements-sdk-v3.txt"
mkdir -p "$ROOT/ros_ws/src"
for package in piper piper_msgs; do
    if [[ -e "$ROOT/ros_ws/src/$package" ]]; then
        diff -qr --exclude=piper_start_ms_node_new.py \
            "$UPSTREAM/train_deploy_alignment/dagger/agilex/src/$package" "$ROOT/ros_ws/src/$package"
    else
        cp -a "$UPSTREAM/train_deploy_alignment/dagger/agilex/src/$package" "$ROOT/ros_ws/src/"
    fi
done
export KAI0_SETUP_ROOT="$ROOT" KAI0_SETUP_UPSTREAM="$UPSTREAM"
python - <<'PY'
from pathlib import Path
import os
root = Path(os.environ['KAI0_SETUP_ROOT'])
upstream = Path(os.environ['KAI0_SETUP_UPSTREAM'])
relative = Path('piper/scripts/piper_start_ms_node_new.py')
destination = root / 'ros_ws/src' / relative
original = upstream / 'train_deploy_alignment/dagger/agilex/src' / relative
adapted = root / 'firmware-compat/piper_start_ms_node_new.py'
if destination.read_bytes() not in (original.read_bytes(), adapted.read_bytes()):
    raise SystemExit('Deployment driver has additional changes; preserve and review them before rebuilding')
destination.write_bytes(adapted.read_bytes())
PY
chmod u+x "$ROOT/ros_ws/src/piper/scripts/"*.py
cd "$ROOT/ros_ws"
catkin_make -DPYTHON_EXECUTABLE="$CONDA_PREFIX/bin/python" -DCMAKE_BUILD_TYPE=Release -j4
set +u
source devel/setup.bash
set -u
export PYTHONPATH="$UPSTREAM/packages/openpi-client/src:${PYTHONPATH:-}"
python - <<'PY'
import json
import sys
import importlib.metadata as metadata
import rospy, cv2, cv_bridge, torch, av, h5py, dm_env
import piper_sdk
from piper_sdk import C_PiperInterface_V3
from piper_msgs.msg import PosCmd, PiperStatusMsg
from openpi_client import websocket_client_policy
print(json.dumps({'python': sys.executable, 'ros': rospy.__file__, 'cv_bridge': cv_bridge.__file__,
                  'versions': {name: metadata.version(name) for name in
                               ['numpy', 'torch', 'piper_sdk', 'python-can', 'websockets', 'h5py', 'av']}}, indent=2))
PY
micromamba list --explicit > "$ROOT/setup/ros-env-explicit.txt"
python -m pip freeze > "$ROOT/setup/ros-pip-freeze.txt"
