#!/usr/bin/env bash
# This starts the isolated ROS compatibility copy, which moves the followers.
set -euo pipefail

if [[ "${1:-}" != "--confirm-motion" ]]; then
  echo "Usage: $0 --confirm-motion [additional collector arguments]" >&2
  echo "The collector moves both followers to the official initial pose, then runs policy actions." >&2
  exit 2
fi
shift

DAGGER_ROOT="${DAGGER_ROOT:-/home/rie/dev/xingyu.lai/kai0-official-dagger}"
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
POLICY_HOST="${POLICY_HOST:-127.0.0.1}"
POLICY_PORT="${POLICY_PORT:-8014}"
ROS_ENV_PREFIX="${ROS_ENV_PREFIX:-${DAGGER_ROOT}/.ros-env}"
DATASET_NAME="${DATASET_NAME:-task_a_official_mixed1_$(TZ=Asia/Shanghai date +%Y%m%d_%H%M%S)}"
DATASET_DIR="${DATASET_DIR:-${DAGGER_ROOT}/data}"
COLLECTOR_DIR="${COLLECTOR_DIR:-${DAGGER_ROOT}/collector}"

if [[ ! -f "${COLLECTOR_DIR}/agilex_openpi_dagger_collect.py" ]]; then
  echo "[ERR] compatibility collector not found: ${COLLECTOR_DIR}" >&2
  exit 1
fi
mkdir -p -- "${DATASET_DIR}"
if [[ ! -d "${DATASET_DIR}" || ! -w "${DATASET_DIR}" ]]; then
  echo "[ERR] correction data directory is not writable: ${DATASET_DIR}" >&2
  exit 1
fi
if [[ ! -t 0 ]]; then
  echo "[ERR] collector d/Space/s/r controls require an interactive SSH or local terminal." >&2
  exit 1
fi
export DAGGER_SKIP_WORKSPACE=0
echo "[MOTION] Collector requires prepared enabled arms; follower-driver initialization closes grippers." >&2
echo "[MOTION] The collector immediately moves followers to the initial pose, then executes the policy." >&2
echo "[STOP] Ctrl+C stops the process; it does not disable the motors. Keep the on-site emergency stop available." >&2
cd "${COLLECTOR_DIR}"
exec "${SCRIPT_DIR}/run_ros.sh" "${ROS_ENV_PREFIX}/bin/python" \
  "${COLLECTOR_DIR}/agilex_openpi_dagger_collect.py" \
  --host "${POLICY_HOST}" --port "${POLICY_PORT}" --ctrl_type joint \
  --use_temporal_smoothing --chunk_size 50 --publish_rate 30 --inference_rate 3 \
  --dataset_dir "${DATASET_DIR}" --dataset_name "${DATASET_NAME}" "$@"
