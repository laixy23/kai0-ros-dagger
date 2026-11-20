#!/usr/bin/env bash
# Official ROS1 Kai0 DAgger: one new session per invocation.
set -euo pipefail

DAGGER_ROOT="${DAGGER_ROOT:-$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)}"
export DAGGER_ROOT
ROS_RUN="${DAGGER_ROOT}/ros-launch/run_ros.sh"
COLLECTOR_RUN="${DAGGER_ROOT}/ros-launch/run_official_collector.sh"
ROS_PYTHON="${DAGGER_ROOT}/.ros-env/bin/python"
OPERATOR_DIR="${DAGGER_ROOT}/operator"

usage() {
  cat <<'TEXT'
用法：bash run_dagger.sh [--check | --stop | --help] [官方采集器参数...]

默认行为：恢复本轮驱动，检查设备，双从臂平滑回官方初始姿态，然后开始 mixed_1 推理。
官方初始姿态（每臂）：[0, 0.32, -0.36, 0, 0.24, 0, 0.07]
每次运行默认创建新的数据目录，已有记录不删除、不覆盖。

运行中的按键（不需要回车）：
  d       暂停推理，到位后并行保持3秒和准备跟随；两项都就绪才放开并自动录制
  空格    可选：保存后在当前示教状态下重新开始录制一段
  s       保存本段纠正数据
  任意键  保存后的确认；w 删除刚保存的一段
  r       结束接管，恢复推理
  Ctrl+C  停止当前一轮和四臂 ROS 转发，不恢复推理

接管中也可以直接 Ctrl+C，不用先按 r。需要保留纠正数据时先按 s 并等待保存完成。
Ctrl+C 不会自动保存尚未保存的纠正段。
退出不会自动失能、归位，也不等同硬件急停；电机可能继续完成最后一个目标。
重新运行本命令会归位并开始新一轮。
四臂启动恢复不主动失能或复位：未使能/部分使能且反馈正常时按实测当前位置直接使能，从臂随后按原流程平滑归位。
已正常使能的通道保持原准备流程；故障或无法取得可信反馈时仍报告失败，不回退旧的卸力恢复。
主臂旧 FA 模式缺反馈时，仅对已有记录支持免失能切换的固件尝试恢复反馈，再验证六轴仍使能。
主臂驱动使用已经准备好的 CAN 持位状态，不重复切换角色；首次新版本仍需低位支撑验证。
保存确认期间 r 只作确认，需要再按一次 r 才恢复推理。

环境变量：
  DATASET_NAME=cloth_round_01  指定数据集名称；默认每次新建
  DATASET_DIR=/path/to/data   修改保存目录
  POLICY_HOST=127.0.0.1      策略服务器，默认本机
  POLICY_PORT=8014           官方 mixed_1 服务端口

--check 只检查，不归位、不启动采集、不使能机械臂。
--stop 用于已退出采集器后的清理：仅停止本项目四臂 ROS 驱动，不启动或恢复推理。
启动前须完成 CAN 配置，并运行模型、ROS master 和相机；四臂驱动由本入口管理。
脚本不会自动结束其他采集器或清除独立的故障隔离。
TEXT
}

case "${1:-}" in
  -h|--help) usage; exit 0 ;;
  --stop)
    if [[ ! -x "${ROS_PYTHON}" || ! -f "${OPERATOR_DIR}/stop_arm_drivers.py" ]]; then
      echo "[错误] 缺少四臂停止工具，请完整更新 operator 目录。" >&2
      exit 2
    fi
    exec "${ROS_PYTHON}" "${OPERATOR_DIR}/stop_arm_drivers.py" --human
    ;;
  --check) CHECK_ONLY=1; shift ;;
  *) CHECK_ONLY=0 ;;
esac

if [[ ! -x "${ROS_PYTHON}" || ! -f "${ROS_RUN}" || ! -f "${COLLECTOR_RUN}" ]]; then
  echo "[错误] 请在 wj-dev 的 kai0-official-dagger 目录运行此脚本。" >&2
  exit 2
fi
if [[ ! -f "${OPERATOR_DIR}/prepare_keyboard_session.py" || ! -f "${OPERATOR_DIR}/run_keyboard_session.py" || ! -f "${OPERATOR_DIR}/stop_arm_drivers.py" ]]; then
  echo "[错误] 缺少键盘会话管理文件，请完整更新 operator 目录。" >&2
  exit 2
fi
DAGGER_MANAGED_DRIVERS=0
DAGGER_CHECK_FILE=""
cleanup_startup() {
  local result=$?
  trap - EXIT INT TERM HUP
  if [[ -n "${DAGGER_CHECK_FILE}" ]]; then rm -f -- "${DAGGER_CHECK_FILE}"; fi
  if [[ "${DAGGER_MANAGED_DRIVERS}" == 1 ]]; then
    if ! "${ROS_PYTHON}" "${OPERATOR_DIR}/stop_arm_drivers.py"; then
      echo "[错误] 四臂驱动清理未完成，请检查服务状态；需要立即制止运动时使用现场急停。" >&2
      if [[ "${result}" == 0 ]]; then result=1; fi
    fi
  fi
  exit "${result}"
}
trap cleanup_startup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
trap 'exit 129' HUP
if [[ "${CHECK_ONLY}" == 0 ]]; then
  if [[ ! -t 0 || ! -t 1 ]]; then
    echo "[错误] 需要交互终端；远程运行请使用 ssh -t。" >&2
    exit 2
  fi
  command -v flock >/dev/null || { echo "[错误] 缺少 flock。" >&2; exit 2; }
  mkdir -p "${DAGGER_ROOT}/_runtime"
  exec 9>"${DAGGER_ROOT}/_runtime/keyboard-dagger.lock"
  flock -n 9 || { echo "[错误] 另一个 Bash DAgger 会话正在运行。" >&2; exit 2; }
fi

echo "[检查] 旧采集会话和右主隔离状态……"
"${ROS_PYTHON}" - "${DAGGER_ROOT}" <<'PY'
from pathlib import Path
import json
import re
import sys

root = Path(sys.argv[1])
owners, stopped = [], []
for directory in Path('/proc').iterdir():
    if not directory.name.isdecimal():
        continue
    try:
        argv = [part.decode(errors='replace') for part in (directory / 'cmdline').read_bytes().split(b'\0') if part]
        state = next(line.split()[1] for line in (directory / 'status').read_text().splitlines() if line.startswith('State:'))
        if not argv or state in ('Z', 'X', 'x'):
            continue
        program = Path(argv[0]).name
        if re.fullmatch(r'python(?:\d+(?:\.\d+)*)?', program):
            program = None
            index = 1
            while index < len(argv):
                value = argv[index]
                if value in ('-c', '-m'):
                    break
                if value == '--':
                    program = Path(argv[index + 1]).name if index + 1 < len(argv) else None
                    break
                if value in ('-W', '-X'):
                    index += 2
                    continue
                if not value.startswith('-'):
                    program = Path(value).name
                    break
                index += 1
        if program in ('agilex_openpi_dagger_collect.py', 'run_first_live_cycle.py'):
            owners.append(directory.name)
        if (program == 'piper_start_ms_node_new.py' and state in ('T', 't')
                and any(value.startswith('__name:=piper_master_right') for value in argv)):
            stopped.append(directory.name)
    except (OSError, StopIteration, IndexError):
        continue
if owners:
    print('[阻塞] 已有采集器运行，PID=' + ','.join(owners) + '。先退出旧会话，避免重复控制。', file=sys.stderr)
if stopped:
    print('[阻塞] 右主 ROS 进程仍被暂停隔离，PID=' + ','.join(stopped) + '。需先完成恢复。', file=sys.stderr)
if owners or stopped:
    raise SystemExit(2)
PY

for DAGGER_SERVICE in kai0-official-mixed1 kai0-official-roscore kai0-official-cameras; do
  if ! systemctl --user is-active --quiet "${DAGGER_SERVICE}.service"; then
    echo "[阻塞] 服务未运行：${DAGGER_SERVICE}.service" >&2
    exit 2
  fi
done
if [[ "${CHECK_ONLY}" == 1 ]]; then
  if ! systemctl --user is-active --quiet kai0-official-arms.service; then
    echo "[阻塞] 四臂驱动尚未运行；--check 不启动服务。运行本脚本默认入口可准备新一轮。" >&2
    exit 2
  fi
else
  echo "[准备] 停止上轮四臂转发并准备本轮驱动……"
  # A signal sent only to this Bash PID can be deferred until its foreground
  # helper returns. Record it until ownership is known, then clean the new
  # drivers before exiting. Terminal Ctrl+C also reaches the helper itself.
  DAGGER_PREPARE_EXIT=0
  trap 'DAGGER_PREPARE_EXIT=130' INT
  trap 'DAGGER_PREPARE_EXIT=143' TERM
  trap 'DAGGER_PREPARE_EXIT=129' HUP
  if bash "${ROS_RUN}" "${ROS_PYTHON}" "${OPERATOR_DIR}/prepare_keyboard_session.py" --root "${DAGGER_ROOT}" --apply; then
    DAGGER_MANAGED_DRIVERS=1
  else
    # The helper cleans its own claimed drivers on failure; it may also have
    # rejected ownership, in which case this wrapper must not stop that owner.
    DAGGER_PREPARE_RESULT=$?
    exit "${DAGGER_PREPARE_RESULT}"
  fi
  trap 'exit 130' INT
  trap 'exit 143' TERM
  trap 'exit 129' HUP
  if [[ "${DAGGER_PREPARE_EXIT}" != 0 ]]; then exit "${DAGGER_PREPARE_EXIT}"; fi
fi

echo "[检查] 四臂真实反馈、夹爪单位和控制模式……"
DAGGER_CHECK_FILE="$(mktemp "${TMPDIR:-/tmp}/kai0-dagger-ready.XXXXXX")"
if ! bash "${ROS_RUN}" "${ROS_PYTHON}" "${DAGGER_ROOT}/setup/preflight_v3_feedback.py" >"${DAGGER_CHECK_FILE}"; then
  cat "${DAGGER_CHECK_FILE}" >&2
  echo "[阻塞] 四臂反馈检查未通过；主臂仍在旧接管角色时也可能无普通反馈。" >&2
  exit 2
fi
"${ROS_PYTHON}" - "${DAGGER_CHECK_FILE}" <<'PY'
import json
import sys
report = json.load(open(sys.argv[1]))
if not report.get('passed'):
    raise SystemExit('[阻塞] 四臂反馈尚未就绪。')
expected = {'can_left_slave', 'can_right_slave', 'can_left_mas', 'can_right_mas'}
if set(report.get('channels', {})) != expected:
    raise SystemExit('[阻塞] 缺少四臂反馈。')
for name, row in report['channels'].items():
    if row.get('ctrl_mode') != 1:
        raise SystemExit('[阻塞] ' + name + ' 仍在示教或其他模式；请先恢复本轮起始状态。')
    if not all(row.get('driver_enabled', [])) or len(row.get('driver_enabled', [])) != 6:
        raise SystemExit('[阻塞] ' + name + ' 六个关节驱动尚未全部使能。')
print('[通过] 四臂反馈正常。')
PY

echo "[检查] 三路相机图像……"
if ! bash "${ROS_RUN}" "${ROS_PYTHON}" "${DAGGER_ROOT}/ros-launch/probe_official_topics.py" \
    --cameras-only --samples 3 --timeout 10 >"${DAGGER_CHECK_FILE}"; then
  cat "${DAGGER_CHECK_FILE}" >&2
  exit 2
fi
rm -f -- "${DAGGER_CHECK_FILE}"
DAGGER_CHECK_FILE=""
if [[ "${CHECK_ONLY}" == 1 ]]; then
  echo "[通过] 仅检查结束，未归位、未启动推理。"
  exit 0
fi

export DATASET_NAME="${DATASET_NAME:-cloth_$(TZ=Asia/Shanghai date +%Y%m%d_%H%M%S)_$$}"
export DATASET_DIR="${DATASET_DIR:-${DAGGER_ROOT}/data}"
echo "[新一轮] 数据集：${DATASET_NAME}"
echo "[开始] 双从臂将回官方初始姿态，然后开始推理。请保持工作区清空、急停可用。"
echo "[键盘] d 对齐、保持3秒后自动示教录制 | s 保存 | r 恢复 | Ctrl+C 停止本轮（无需 r）"
# The official collector performs the initial interpolation on every new process.
exec "${ROS_PYTHON}" "${OPERATOR_DIR}/run_keyboard_session.py" --owns-arms -- bash "${COLLECTOR_RUN}" --confirm-motion "$@"
