#!/usr/bin/env bash
# USB pedal controls around the unchanged original DAgger Bash flow.
set -euo pipefail
DAGGER_ROOT="${DAGGER_ROOT:-$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)}"
export DAGGER_ROOT
DAGGER_PEDAL_CONFIG="${DAGGER_PEDAL_CONFIG:-${DAGGER_ROOT}/_runtime/foot-pedal.json}"
DAGGER_PEDAL_PYTHON="${DAGGER_ROOT}/.ros-env/bin/python"
if [[ ! -x "${DAGGER_PEDAL_PYTHON}" ]]; then DAGGER_PEDAL_PYTHON=python3; fi

case "${1:-}" in
  --help|-h)
    cat <<'TEXT'
用法：bash run_dagger_pedal.sh [原 run_dagger.sh 参数...]

原来的启动、归位、模型和数据流程保持相同，键盘操作也保留。
左单踩：进入示教 d；保存后在已确认示教状态下再单踩，恢复跟随并重新开始一段。
中单踩：暂停跟随并保存 s；保存确认期间只确认保留，不恢复跟随。
中双踩：仅保存确认期间删除刚保存的一段 w，不先执行保存/确认。
右单踩：恢复推理 r；保存确认期间只确认，需要再次单踩才恢复。
右双踩：Ctrl+C 停止本轮并保存剩余 inference，不先触发 r、不恢复推理。
每次踩下再松开算一次；双踩第二次须在第一次松开后0.5秒内踩下。
中、右单踩会等待0.5秒排除双踩；按住时长不决定动作。左单踩松开即触发。

--calibrate  按提示左/中/右各踩3次，不要求长按，不启动机器人。
--test-pedal 只打印踏板动作，不启动机器人；Ctrl+C结束测试。
--help      显示帮助，不启动机器人。
--check/--stop 由原来的 run_dagger.sh 执行。
配置：DAGGER_PEDAL_CONFIG，默认 <root>/_runtime/foot-pedal.json。
TEXT
    exit 0 ;;
  --calibrate)
    shift
    exec "${DAGGER_PEDAL_PYTHON}" "${DAGGER_ROOT}/operator/foot_pedal_device.py" --calibrate "${DAGGER_PEDAL_CONFIG}" "$@" ;;
  --test-pedal)
    shift
    exec "${DAGGER_PEDAL_PYTHON}" "${DAGGER_ROOT}/operator/foot_pedal_device.py" --test "${DAGGER_PEDAL_CONFIG}" "$@" ;;
  --check|--stop)
    exec bash "${DAGGER_ROOT}/run_dagger.sh" "$@" ;;
esac

exec "${DAGGER_PEDAL_PYTHON}" "${DAGGER_ROOT}/operator/run_pedal_session.py" \
  --config "${DAGGER_PEDAL_CONFIG}" -- bash "${DAGGER_ROOT}/run_dagger.sh" "$@"
