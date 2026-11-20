#!/usr/bin/env bash
# Compatibility entry for the now-default master gripper handoff.
set -euo pipefail

if [[ "${1:-}" == "--help" || "${1:-}" == "-h" ]]; then
  cat <<'TEXT'
用法：bash run_dagger_gripper_handoff_test.sh [--pedal] [原入口参数...]

本脚本保留旧命令兼容；正式 run_dagger.sh 和 run_dagger_pedal.sh 已默认使用同一双主夹爪交接逻辑。
默认使用键盘入口；--pedal 使用踏板入口及现有单踩/双踩映射。
按 d/左单踩：沿用原对齐及3秒保持/跟随准备；左右各用本侧已有反馈宽度发送一次夹爪释放指令，再请求 FA。
六关节和从臂沿用原流程，不失能整臂、不设置夹爪零点。
按 r/右单踩：沿用原恢复推理，不额外补发夹爪恢复指令；下一次对齐仍由原流程使能夹爪。
按 s/中单踩：暂停从臂跟随并保存；空格/保存后左单踩重开录制时恢复跟随。
Ctrl+C/右双踩：沿用原停止和保存剩余 inference；退出不补发硬件使能/归位指令。

正常启动会执行原入口的从臂归位和策略动作。现场需有人扶稳主臂、夹爪无物，急停可用。
没有新增夹爪状态等待、超时或恢复分支；现有检查与失败处理保留。
该固件释放夹爪后能否持续发出可用示教数据、能否消除突合，仍需现场验证。
--check/--stop 透传原入口，分别只检查/清理；--help 不启动任何入口。
TEXT
  exit 0
fi

DAGGER_ROOT="${DAGGER_ROOT:-$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)}"
export DAGGER_ROOT
DAGGER_GRIPPER_ENTRY="${DAGGER_ROOT}/run_dagger.sh"
if [[ "${1:-}" == "--pedal" ]]; then
  DAGGER_GRIPPER_ENTRY="${DAGGER_ROOT}/run_dagger_pedal.sh"
  shift
fi
export DAGGER_TEST_MASTER_GRIPPER_RELEASE=both
echo "[兼容入口] 正式版已默认启用左右主夹爪 FC→FA 交接；继续使用相同流程。"
exec bash "${DAGGER_GRIPPER_ENTRY}" "$@"
