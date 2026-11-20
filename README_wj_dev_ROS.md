# 官方 AgileX DAgger：wj-dev 本机 ROS1 操作说明

本文只介绍 **官方 ROS1 DAgger** 在 `wj-dev` 上的使用方法。
原版 `README.md` 保留不动；本文使用本机已有环境、服务和部署路径。

## 1. 本机目录与运行方式

以下 Linux 命令均在 `wj-dev` 上、以 `rie` 用户运行，不需要切换成 root。
从本地连接时使用带终端的 SSH：

```bash
ssh -t wj-dev
```

本机固定路径：

- 官方 ROS DAgger 部署：`/home/rie/dev/xingyu.lai/kai0-official-dagger`
- 官方 Kai0 源码：`/home/rie/dev/kai0-task-a`
- 模型 checkpoint：`/home/rie/dev/kai0-task-a/checkpoints/Task_A/mixed_1`
- 模型地址：`ws://127.0.0.1:8014`
- ROS 环境：`/home/rie/dev/xingyu.lai/kai0-official-dagger/.ros-env`
- ROS 工作空间：`/home/rie/dev/xingyu.lai/kai0-official-dagger/ros_ws`
- 数据目录：`/home/rie/dev/xingyu.lai/kai0-official-dagger/data`

所有 ROS 命令通过部署目录中的 `ros-launch/run_ros.sh` 执行。
它会进入本机已安装的 ROS1 Noetic 环境并加载工作空间，无需另行激活 conda，
也不要在同一终端手工混用模型 Python 环境和其他 ROS 环境。
相机命令使用 `native-camera/run_native_ros.sh`，加载本机相机驱动覆盖层。

## 2. 启动前准备四路 CAN

先退出旧采集会话，再配置 CAN。四臂需供电，CAN 线接牢。
配置脚本按 USB-CAN **序列号**识别设备，换 USB 口后重新执行，不需要固定 USB bus 地址。

```bash
sudo /usr/bin/python3 /home/rie/dev/xingyu.lai/kai0-official-dagger/setup/activate_can_by_serial.py --apply
ip -brief link show type can
```

应出现以下四个名称，全部为 `UP`、1 Mbps：

- 左从：`can_left_slave`，序列号 `0027002C5547571120343930`
- 右从：`can_right_slave`，序列号 `004B002C4648570220393831`
- 左主：`can_left_mas`，序列号 `002E002C5547571120343930`
- 右主：`can_right_mas`，序列号 `001B00285547570420303135`

查看详细状态：

```bash
ip -details -statistics link show type can
```

`UP` 表示电脑端 CAN 接口已配置，并不等于机械臂已经持续反馈。
接口存在、能回复固件查询、持续提供关节反馈和已经使能，是不同的检查项。

## 3. 准备官方 mixed_1 模型：整段复制执行

ROS 入口明确要求 **`kai0-official-mixed1.service`** 已运行，并不会自动启动它。
如果另一个 mixed_1 服务占着 8014，官方服务即使完成预热也会因端口冲突退出；
此时单看旧日志中的 `GPU warmup passed` 或 `ready.json` 都不能判断模型已就绪。

先退出使用旧模型的采集任务，再把下面**整个代码块**复制到 `wj-dev` 终端执行。
它只准备模型：停止本机两个已核对的冲突模型单元，检查端口归属，启动或复用官方单元，
最多等待 300 秒确认当前进程、8014 监听以及本次预热记录；不会启动机械臂或相机。

```bash
/usr/bin/python3 - <<'PY_MODEL'
from pathlib import Path
import datetime
import json
import re
import subprocess
import time

unit = "kai0-official-mixed1.service"
root = Path("/home/rie/dev/kai0-task-a")

def state(name):
    result = subprocess.run(
        ["systemctl", "--user", "show", name,
         "--property=LoadState,ActiveState,MainPID,WorkingDirectory,ExecStart,ExecMainStartTimestampMonotonic"],
        capture_output=True, text=True, timeout=5)
    values = dict(line.split("=", 1) for line in result.stdout.splitlines() if "=" in line)
    if not values.get("LoadState"):
        raise SystemExit("无法读取服务状态：" + name)
    return values

def listener():
    result = subprocess.run(["ss", "-H", "-ltnp", "sport", "=", ":8014"],
                            check=True, capture_output=True, text=True, timeout=5)
    return result.stdout.strip(), set(re.findall(r"pid=(\d+),", result.stdout))

current = state(unit)
if current.get("WorkingDirectory") != str(root) or "deploy/serve.py" not in current.get("ExecStart", ""):
    raise SystemExit("官方模型服务配置不符，未切换任何服务。")
known = {
    "galaxea-toast-100-original.service":
        ("/home/rie/dev/inference-origin", "/scripts/run_server.sh"),
    "inference-dagger-mixed1.service":
        ("/home/rie/dev/xingyu.lai/inference-dagger", "/server/serve_kai0_mixed1.py"),
}
for name, (directory, entry) in known.items():
    previous = state(name)
    if previous.get("ActiveState") in ("active", "activating", "reloading", "deactivating"):
        if previous.get("WorkingDirectory") != directory or directory + entry not in previous.get("ExecStart", ""):
            raise SystemExit("同名模型服务身份不符，未停止它：" + name)
        print("[准备] 停止冲突模型：" + name, flush=True)
        subprocess.run(["systemctl", "--user", "stop", name], check=True, timeout=45)
current = state(unit)
listing, pids = listener()
if listing and (current.get("ActiveState") != "active" or pids != {current.get("MainPID")}):
    raise SystemExit("8014 仍被其他或无法确认的进程占用，已停止准备：\n" + listing)
subprocess.run(["systemctl", "--user", "start", unit], check=True, timeout=45)
print("[等待] 官方 mixed_1 加载及预热，最多 300 秒……", flush=True)
deadline = time.monotonic() + 300
while time.monotonic() < deadline:
    current = state(unit)
    if current.get("ActiveState") not in ("active", "activating"):
        raise SystemExit("官方模型启动失败，请查看下方排错命令的日志。")
    listing, pids = listener()
    try:
        ready = json.loads((root / "deploy/ready.json").read_text())
        ready_at = datetime.datetime.fromisoformat(ready["ready_at"]).timestamp()
        started_at = time.time() - (time.monotonic() - int(current["ExecMainStartTimestampMonotonic"]) / 1_000_000)
        valid = (current.get("ActiveState") == "active" and current.get("MainPID") not in (None, "0")
                 and pids == {current["MainPID"]} and ready_at >= started_at
                 and ready.get("model") == "Kai0" and ready.get("action_horizon") == 50
                 and ready.get("checkpoint") == "OpenDriveLab-org/Kai0/Task_A/mixed_1")
    except (OSError, ValueError, KeyError):
        valid = False
    if valid:
        print("[模型就绪] " + unit + "，PID=" + current["MainPID"] + "，端口 8014。")
        break
    time.sleep(1)
else:
    raise SystemExit("模型准备超时，未启动任何机械臂。")
PY_MODEL
```

只有看到 `[模型就绪] kai0-official-mixed1.service ...` 才继续第 4 节。
本机官方服务使用 `mixed_1`、`pi05_flatten_fold_normal` 配置和 checkpoint 根目录归一化文件。
ROS 采集器任务文本为 `fold the cloth`，无需追加 `Advantage: positive`。

准备失败时，用以下命令排错；它们不能代替上面整段模型准备流程：

```bash
systemctl --user show kai0-official-mixed1.service \
  --property=ActiveState,MainPID,ExecMainStartTimestamp
ss -ltnp 'sport = :8014'
tail -n 60 /home/rie/dev/kai0-task-a/deploy/official-mixed1-baseline.log
journalctl --user -u kai0-official-mixed1.service -n 40 --no-pager
```

## 4. 启动 ROS master 和三路相机

这两个服务都需要有正确的 systemd 用户服务名称，不能仅在另一个终端手工运行同名程序。
下面命令会复用已经运行的服务；服务不存在时，用本机包装器创建临时服务。

### 4.1 ROS master

```bash
if ! systemctl --user is-active --quiet kai0-official-roscore.service; then
  systemd-run --user --unit=kai0-official-roscore --collect \
    --property=WorkingDirectory=/home/rie/dev/xingyu.lai/kai0-official-dagger \
    --setenv=ROS_IP=127.0.0.1 \
    /bin/bash /home/rie/dev/xingyu.lai/kai0-official-dagger/ros-launch/run_ros.sh roscore
fi
```

等待服务启动，再检查 ROS master 可访问：

```bash
bash /home/rie/dev/xingyu.lai/kai0-official-dagger/ros-launch/run_ros.sh rostopic list
```

如果临时服务创建提示同名单元已存在，先用 `systemctl --user status` 和日志核对它；
不要同时再启动一个 roscore。

### 4.2 相机身份

相机 launch 文件：
`/home/rie/dev/xingyu.lai/kai0-official-dagger/ros-launch/cameras_wj_dev.launch`。
默认使用三台 RealSense，彩色图像为 640×480、30 Hz：

- 头部：`263022074037` → `/camera_f/color/image_raw`
- 左腕：`263022070737` → `/camera_l/color/image_raw`
- 右腕：`263022072753` → `/camera_r/color/image_raw`

2026-11-11 已通过实际画面和现场确认：`263022074037` 就是头部相机。
部署目录的 `cameras_wj_dev.launch` 已将 `front_serial` 默认值持久化为该序列号；
左右腕序列号保持不变。正常启动和重建相机服务会直接使用这个映射。
原头部序列号 `263022074810` 已停用，旧日志中的缺设备错误见第 8.1 节。
2026-11-11 15:59 复查：三路在 8 秒内各收到 240 帧，均为 640×480、约 30 FPS，
时间戳新鲜，ROS 中的三路序列号参数与上述映射一致。此结果仅说明相机检查通过。
如果以后更换相机，仍需先确认实际位置，再更新对应序列号。

### 4.3 相机服务

确认三台相机身份正确且未被其他进程占用后：

```bash
if ! systemctl --user is-active --quiet kai0-official-cameras.service; then
  systemd-run --user --unit=kai0-official-cameras --collect \
    --property=WorkingDirectory=/home/rie/dev/xingyu.lai/kai0-official-dagger \
    --setenv=ROS_IP=127.0.0.1 \
    /bin/bash /home/rie/dev/xingyu.lai/kai0-official-dagger/native-camera/run_native_ros.sh \
    roslaunch /home/rie/dev/xingyu.lai/kai0-official-dagger/ros-launch/cameras_wj_dev.launch
fi
```

只检查图像话题，不启动机械臂：

```bash
bash /home/rie/dev/xingyu.lai/kai0-official-dagger/ros-launch/run_ros.sh \
  python /home/rie/dev/xingyu.lai/kai0-official-dagger/ros-launch/probe_official_topics.py \
  --cameras-only --samples 3 --timeout 10
```

服务 `active` 或 USB 能枚举到相机，均不能替代这一步实际图像检查。
排错日志：

```bash
journalctl --user -u kai0-official-cameras.service -n 80 --no-pager
```

## 5. 启动一轮官方 ROS DAgger

准备好四臂：放低并稳妥支撑，夹爪无物，人员撤出运动范围，急停可用。
官方驱动启动可能使能机械臂并闭合从臂夹爪；采集器随后会移动从臂并开始推理。

先完成第 2–4 节；下面的采集入口不会代你启动模型、ROS master 或相机。
在交互终端运行：

```bash
bash /home/rie/dev/xingyu.lai/kai0-official-dagger/run_dagger.sh
```

这个入口依次：

1. 检查旧采集器、右主隔离状态以及模型、ROS master、相机三个服务。
2. 停止上一轮托管的四臂转发，读取实际 CAN 反馈。
3. 四臂使能准备：已处于 CAN 控制且六轴使能时直接通过。**完整反馈显示未使能或部分使能时，正常启动会自动按实测当前位置使能**：先预装新鲜实际六关节位置，待机/示教状态沿既有当前位置持位流程恢复 CAN 控制，再直接发送 `EnableArm(7)`，重发同一位置目标并由真实六电机反馈确认。不会先失能、复位、归零或额外操作夹爪；不会只改软件使能标志。有可信实际反馈的旧示教/待机主臂继续按实测位置固定持位恢复 CAN 控制。主臂在旧 FA 模式下缺普通实际反馈时，仍沿用已有固件支持的单次 FC 取回反馈流程（>=1.8.9；当前记录为 S-V1.9），不在没有实际姿态时盲目使能。故障或反馈不可信仍按原逻辑报告；旧卸力恢复工具不会自动调用。已有恢复漂移 1° 拒绝门继续保持删除，首次目标写入前取新鲜实际姿态，随后固定同一持位目标，不反复追踪改目标。`--check` 和 `--stop` 不会触发这项自动使能。
4. 启动本轮四臂 ROS 驱动。主臂节点只被动验证准备好的 CAN 持位状态，不重复切换角色；随后检查四臂反馈及三路实际图像。从臂驱动初始化仍会使能并闭合夹爪。
5. 双从臂回官方初始姿态，然后立即执行模型动作，进入键盘交互。

每只从臂的初始姿态为：

```text
[0, 0.32, -0.36, 0, 0.24, 0, 0.07]
```

前六项是关节弧度，最后一项是夹爪开度（米）。每次重新运行都会重新归位，
创建新一轮数据目录。入口本身不会启动前三个依赖服务，也不会跳过反馈、图像或隔离错误。
如果从臂缺少原始反馈，需先排查通信；脚本不能靠“已确认支撑”绕过缺失反馈。

### `--check` 和 `--stop` 的区别

```bash
# 检查已经运行的官方服务、四臂反馈和图像；不归位、不启动采集
bash /home/rie/dev/xingyu.lai/kai0-official-dagger/run_dagger.sh --check

# 先 Ctrl+C 退出旧采集器，再清理残余四臂 ROS 转发
bash /home/rie/dev/xingyu.lai/kai0-official-dagger/run_dagger.sh --stop
```

`--check` 要求四臂服务已运行，因此不是首次启动前自动准备全部依赖的命令。
首次启动按第 2–5 节操作，由正常入口准备四臂驱动。
`--stop` 跳过模型和相机依赖检查，只清理四臂驱动；它不会开始新一轮，
也不会关闭模型、ROS master 或相机。参数写作 `--stop`，中间没有空格。

## 6. 键盘操作

按键在运行采集器的同一终端输入，不需要回车。

- **`d`**：暂停新的策略输出，进入接管流程后让原推理采样线程额外记录 **1 秒**实际反馈，再停止并保存推理阶段数据。这一秒仍归 `inference_hdf5`，不继续执行新策略动作、不写重复填充帧，也不把后续对齐等待并入示教段。按当前名义 30 Hz 约增加 30 次采样，实际数量取决于有效输入；采样已结束或用户退出时不会硬凑时长。双主臂先移动到官方中间姿态，
  再对齐当前从臂姿态；最终对齐检查通过后继续保持位置控制，连续验证双主臂实际关节
  实际到位验证通过后，**立即并行开始两项工作**：持续按最终目标保持位置并验证至少 **3 秒**；同时验证双从臂原位探针、启动主臂 FC 实际姿态转发并等待双侧回执。准备阶段只发送 `prepare_master`，不会放开主臂。两项条件都满足后才发送 `release_master`；到位后的等待约为 **max(3 秒，跟随准备耗时)**，不会先等 3 秒再开始预热。重复同一最终六关节目标不会取消已完成的准备，释放前还会重新检查数据鲜度与六关节目标。夹爪位置差或夹爪单独改变目标不阻止对齐、准备或进入示教；夹爪示教控制仍保留，未固定或禁用。
  `release_master` 最终发送 FA，**FA 本身就是进入零力拖动的释放点**，不再在它之后等待 6 秒或重复使能。原生转发线程已提前准备好，FA 的四个原生数据帧收齐即转发，消息保留原 CAN 源时间，不用旧 SDK 聚合缓存或全零回退。收到 FA 数据的从臂回执以及双主模式确认后，提示就绪并自动开始录制纠正段。
  每次从 inference 进入 DAgger 时，纠正段还会包含**原来正式开录时点之前约 1 秒**的真实三路图像和从臂关节数据。切换期间独立预采，正式开录时只把此前 1 秒接到本段开头，不把整个对齐或 FA 等待都写入，也不增加固定等待。按相机原始 ROS 时间选取这一秒；接续时不重复写入同一源帧，观测与下一帧实际动作的配对保持原样。数据不足时只保留真实可得数据，不补帧；机械臂切换顺序、3 秒保持和既有控制流程不变。
  左右主臂等待 FA 首组完整有效数据、双侧跟随回执以及采集器的示教模式确认均**没有固定等待期限**；已删除额外添加的 2 秒首组、2.5 秒双侧回执和进入 FA 的外层 15 秒超时。数据一到立即转发，真实回执和模式确认满足后自动开始录制；等待期间可用 Ctrl+C 停止。FC 预热缺流检查仍为 250 毫秒，各帧鲜度及完整性检查保留；不再因额外的首次姿态差 2° 或 FC 转发姿态差 1° 拒绝。FA 已经释放主臂，等待期间是否已跟随取决于原生数据和从臂实际执行情况。
  **无需再按空格。** 这 3 秒从最终位置验证通过后开始计算，不包含归位运动时间；
  普通短暂未就绪会继续保持，整个并行准备最长等待 15 秒；真实故障、失能、位置偏离、取消或超时会阻止释放。真实跟随还存在控制器帧周期、ROS 与执行器延迟，不能承诺物理零延迟。保持期间反馈丢失、失能、故障或偏离目标导致验证失败时，
  不进入示教、不开始录制。
  按键后主臂会使能并移动，手先离开运动范围；中间姿态不代表已规划避障。
  使能在位置保持和拖动示教期间都保留；“可以拖动”表示控制方式改变，并不表示电机失能。就绪检查针对进入示教阶段，不替代示教过程中持续的硬件安全保护。
  FA 首组或回执未到时继续等待，不因固定时长关闭转发或切回 FC。**FA 已收到首组后，不论接管确认前后**，没有新原生数据都进入可恢复等待；超过 250 毫秒仅显示等待，不因此立即关闭转发、切模式或重发旧目标。新鲜完整数据恢复后继续跟随，不再附加恢复目标差 2° 的等待门。真正无效数据、接收器错误或手动停止仍沿用已有停止流程；交接故障若需要切 FC，则读取新的实际位置持位，不强拉回旧的对齐位置。
  [官方维护者说明](https://github.com/agilexrobotics/piper_sdk/issues/112#issuecomment-4515586181)主臂不运动时可能静默，因此不能将所有静默都判成连接故障。采集器不再要求接管后的主臂控制包持续小于 250 毫秒；本轮双方已见过控制数据且模式确认有效即可。记录仅需要相机和从臂数据同步，不再等待未用于记录内容的主臂队列。静止不自动结束本段；模式明确异常或状态确认过期时，暂停追加记录并保留已有内容，等待期间不自动恢复推理。
  最终保持和跟随预热确认结束后会自动进入可拖动状态，请在现场准备好支撑主臂，避免松手悬空。首次需现场低位验证日志顺序和实际延迟。
- **空格（可选）**：第一次录制不需要空格；已保存并结束一段后，重新开始录制时恢复主臂到从臂的跟随，不重复附加上一段的提前采样。
- **`s`**：先暂停双主臂向从臂转发目标并结束采集，再保存本段纠正数据。从臂六关节和夹爪保持最后接受的位置目标，主臂再动不会继续更新从臂目标；保存确认、删除和等待重开期间都保持暂停。没有数据时也结束采集并暂停跟随，不生成空文件。主臂仍处于示教模式；不额外发送使能、失能、归位或切模式指令，也不增加确认、超时或位置限制。已下发或仍在途的目标可能完成，暂停转发不等于取消已下发运动。
- **保存后约 10 秒内 `w`**：删除刚保存的一段；其他键或超时保留。
  确认窗口内第一次 `r` 只确认数据，需要再按一次 `r` 才恢复推理。
- **`r`**：结束接管，等待双主模式确认及转发停止后，恢复新的模型推理段。未确认成功时显示具体侧别和原因，推理保持暂停，当前会话和已有数据保留；可按 `s` 保存，再按 `r` 重试，或 Ctrl+C 结束。交接期间重复 `r` 不叠加请求；不再因一次交接超时直接结束整轮。
- **Ctrl+C**：直接结束本轮，先停止四臂 ROS 转发，再停止采样并自动保存当前尚未保存的 **inference** 段，不需要先按 `r`。沿用本轮目录和下一个 episode 编号，同时导出对应视频；退出时不额外等待尾采样。首次退出会等待保存完成并显示进度，再次 Ctrl+C 可强制退出，未完成的文件可能不完整。
  人工纠正段仍需按 `s` 保存；退出不会把未保存的纠正自动当成完整示教，也不会重复保存已经按 `d` 写出的 inference。没有有效 action 的空段不创建文件。

典型顺序：推理 → `d` → 等待对齐 → 并行保持3秒及准备跟随 → 两项就绪后自动示教并录制 → 操作主臂纠正 → `s` → 确认保留/删除 → `r`。
退出不主动失能，也不自动归位；停止进程不能代替现场急停。
如果通过 ROS 话题显式请求主臂失能，会立即取消当前切换，并在该驱动进程内锁存取消；旧的排队使能或示教指令不能重新使能。此时先结束本轮，再通过正常入口准备新一轮。

## 7. 数据保存和重新开始

### USB 三踏板入口

键盘入口 `run_dagger.sh` 保持原样。踏板使用独立入口，启动、归位、模型、接管和数据流程仍调用原入口：

```bash
bash /home/rie/dev/xingyu.lai/kai0-official-dagger/run_dagger_pedal.sh
```

- 左单踩：相当于 `d`；保存并确认后，已处于示教模式时再次单踩可恢复跟随并重新录一段，相当于空格。
- 中单踩：相当于 `s`，先暂停跟随再保存；保存确认期间只确认保留，不恢复跟随。
- 中双踩：仅在保存确认期间相当于 `w`，删除刚保存的一段；窗口外不删除，也不先执行单踩的保存/确认。
- 右单踩：相当于 `r`；保存确认期间只确认，需要再次单踩才恢复推理。
- 右双踩：按 Ctrl+C 流程停止本轮并保存尚未保存的 inference，不先触发 `r`，不会恢复推理。准备阶段也能使用。

每次踩下再松开算一次；双踩的第二次须在第一次松开后的 **0.5 秒内踩下**。中、右单踩等待该窗口结束才触发，双踩不会先执行单踩动作；左单踩松开即触发。按住时长不决定动作。若同时踩了多块，普通操作作废，右双踩停止仍有效。键盘操作始终保留。首次操作前，程序会等采集器控制线程就绪，准备期间的普通踩踏不会排队到之后触发。

踏板按序列号 `864032454065` 识别，并独占这只 USB 设备的输入接口，避免默认按键传到桌面或 SSH 终端。三个 `/dev/input/event*` 是其 HID 接口，不对应三块踏板；左右中绑定来自实际校准，不假设默认键码。程序不修改踏板固件键位，不向整个系统注入按键。

首次只读校准与测试（不启动机器人）：

```bash
bash /home/rie/dev/xingyu.lai/kai0-official-dagger/run_dagger_pedal.sh --calibrate --timeout 600
bash /home/rie/dev/xingyu.lai/kai0-official-dagger/run_dagger_pedal.sh --test-pedal
```

校准按左、中、右顺序，每块踩下并松开 **3 次**，看到该块完成 `3/3` 后再踩下一块，不要求长按。只有三块均确认完成才保存 `_runtime/foot-pedal.json`；换 USB 口不依赖旧的 event 编号。测试模式只打印动作；按键盘 Ctrl+C 结束测试。

Linux 输入设备读取权限需由管理员设置一次，只授权这只踏板，不把用户加入可读取所有键盘的 input 组：

```bash
sudo python3 /home/rie/dev/xingyu.lai/kai0-official-dagger/operator/setup_foot_pedal_access.py --apply
```

单踩／双踩按内核事件时间和完整按下松开次数区分，不依赖设备上报的按住时长；输入事件批量到达时也不先执行单踩操作。踏板失联时停止接收踏板动作，键盘仍可操作；退出进程会释放踏板独占。

### 正式版左右主夹爪 FC → FA 交接

正式键盘版和踏板版已默认使用此前夹爪验证版的双主臂交接逻辑；此前取消的“保存时主臂固定”没有加入。

```bash
# 键盘版；正常启动仍沿用原入口的从臂归位及策略运行
bash /home/rie/dev/xingyu.lai/kai0-official-dagger/run_dagger.sh

# 踏板版；使用现有单踩／双踩映射
bash /home/rie/dev/xingyu.lai/kai0-official-dagger/run_dagger_pedal.sh
```

进入 `d` 时沿用对齐和 3 秒保持／跟随准备；在原有模式锁内、最后一条对齐指令之后，左右主臂各使用本侧已有原生 FC 反馈宽度，在原 FA 转发阶段建立后各发送一次夹爪失能指令（WIDTH、力度 0、不设置零点），随后发送原 FA 角色请求。六关节和从臂的控制方式沿用原流程。

正式化只改变默认选择，没有增加夹爪状态确认、独立故障拦截、第二次跟随检查或恢复分支。已有的数据有效性、手动停止、跟随准备和 FA 完整数据检查继续使用；已删除的 FA 2 秒、2.5 秒及外层 15 秒时限继续保持删除。历史环境变量 `DAGGER_TEST_MASTER_GRIPPER_RELEASE` 未设置时默认 `both`；显式 `right` 兼容只用右侧，显式空值或其它值仍沿用普通请求，不新增配置报错。

旧 `run_dagger_gripper_handoff_test.sh` 及其 `--pedal` 命令保留为兼容入口，仍执行同一版逻辑。

日志区分 FC `0x2A8` 的实际反馈与 FA `0x159` 的示教命令，记录最后对齐宽度、实际宽度、力度、状态码和首组 FA 夹爪数据。FA 命令值不能替代真实夹爪位置，软件测试通过也不能证明已消除物理突合。

按 `r`、交接失败或退出时均沿用现有处理，不额外发送夹爪恢复指令。下一次对齐仍会由原关节控制回调发送夹爪使能位置指令。`Ctrl+C`／右双踩保持原退出与 inference 保存流程。

本次按要求将该逻辑设为正式默认，没有执行机械臂操作。两侧物理释放并非严格同步；是否消除夹爪突合仍以现场表现为准。交接与退出沿用现有停止策略。

默认根目录：
`/home/rie/dev/xingyu.lai/kai0-official-dagger/data`。

每次默认创建带时间戳的新数据集，不覆盖之前的记录：

```text
data/
  <dataset_name>_inference_hdf5/aloha_mobile_dummy/episode_*.hdf5
  <dataset_name>_dagger_hdf5/aloha_mobile_dummy/episode_*.hdf5
```

两类 episode 编号独立。视频导出保持开启，保存函数依赖此设置；
导出的视频位于对应数据集的
`aloha_mobile_dummy/video/<camera_name>/episode_<i>.mp4`。

需要指定数据集名称和位置时，在 `wj-dev` 运行：

```bash
DATASET_NAME="cloth_ros_$(TZ=Asia/Shanghai date +%Y%m%d_%H%M%S)_$$" \
DATASET_DIR=/home/rie/dev/xingyu.lai/kai0-official-dagger/data \
bash /home/rie/dev/xingyu.lai/kai0-official-dagger/run_dagger.sh
```

显式指定的 `DATASET_NAME` 也应每轮唯一。复用同名数据集时，默认 `--episode_idx 0`
可能覆盖已有 HDF5 和视频；建议使用上面的时间戳名称。只有核对过已有编号后，
才考虑另行指定不冲突的 `--episode_idx`。

Ctrl+C 后再次运行正常入口，即归位并开始新一轮。确认无旧采集器后仍有残余转发，
可先运行上述 `--stop`。两轮采集不能同时控制同一套机械臂。

## 8. 常见阻塞及应对顺序

- **`服务未运行：kai0-official-mixed1.service`**：完成第 3 节，检查正确服务名、
  8014 端口归属及本次预热；其他名字的模型服务 active 不会通过官方入口检查。
- **相机服务不存在**：完成第 4 节的 `systemd-run` 创建；重启后临时服务可能需要重建。
- **已有采集器、右主暂停隔离**：先处理旧会话或隔离状态，不清空锁文件、
  不用 `kill -CONT` 恢复未知状态的旧驱动。
- **CAN 为 UP 但缺反馈**：只能说明接口配置完成。继续核对序列号、供电、CAN 链路和反馈角色；
  不以启动整个机械臂驱动代替被动检查。恢复通信角色可能涉及失能，需现场支撑。
- **四臂仍在旧示教模式或尚未全部使能**：正常启动时，有完整正常反馈的未使能/部分使能通道会按实测当前位置直接使能；已使能通道仍沿用原恢复流程。主臂缺反馈时按上述固件门槛，仅切 FC 后等待真实反馈，不盲目使能。若实际使能反馈始终未到，入口会报告失败；不自动调用旧失能、复位恢复工具。
  [官方维护者确认](https://github.com/agilexrobotics/pyAgxArm/discussions/53)较新固件支持免失能切换。程序已取消自动恢复中的主动失能，但旧 FA 切 FC 到首次真实反馈之间无法独立观察位置，不能保证固件切模零瞬间位移或卸力；首次仍需低位支撑验收。
- **FA 原生数据检查失败**：夹爪 WIDTH 原始读数按 SDK 保留有符号值，异常上限为绝对值 150 mm；发给从臂的目标另限制到 0～80 mm。原始值不冒充限幅后的执行目标。首次 FA 与静默恢复不再附加 2° 目标差拒绝。缺帧日志会列出各 ID 的计数、源时间、年龄、原始值、组跨度与过滤原因；夹爪仍需有效 WIDTH 数据，完整性与新鲜度检查保留。

### 8.1 `CalledProcessError(... probe_official_topics.py ...)`

该异常只表示子检查进程返回非零，并没有告诉你具体缺少哪个话题。
本机准备程序将检查输出捕获在子进程中，终端有时只显示这一行异常。
这条启动检查命令要求下面 7 个话题在 15 秒内各收到至少 3 条消息：

- 头部图像：`/camera_f/color/image_raw`
- 左腕图像：`/camera_l/color/image_raw`
- 右腕图像：`/camera_r/color/image_raw`
- 左右从臂：`/puppet/joint_left`、`/puppet/joint_right`
- 左右主臂：`/puppet_master/joint_left`、`/puppet_master/joint_right`

出现这个异常时，按以下顺序排查，不要直接反复启动四臂驱动。

**第一步：确认失败阶段和清理结果。**

```bash
/usr/bin/python3 -m json.tool \
  /home/rie/dev/xingyu.lai/kai0-official-dagger/_runtime/keyboard-prepare.json
```

关注 `drivers_started`、`error` 和 `failure_cleanup.passed`。
例如，`drivers_started: true` 表示本轮已经启动过驱动；
`failure_cleanup.passed: true` 表示检查失败后已停止本轮驱动。
因此此时四臂服务为 `inactive` 或 `not-found` 可能是正常清理结果，不能据此反推最初失败原因。
停止转发不代表电机已经失能。

**第二步：先单独检查相机，直接查看详细 JSON。**

```bash
bash /home/rie/dev/xingyu.lai/kai0-official-dagger/ros-launch/run_ros.sh \
  /home/rie/dev/xingyu.lai/kai0-official-dagger/.ros-env/bin/python \
  /home/rie/dev/xingyu.lai/kai0-official-dagger/ros-launch/probe_official_topics.py \
  --cameras-only --samples 3 --timeout 10
```

这是只订阅检查，不启动相机或机械臂，也不发布控制命令。
查看每个话题的 `received`，三路都至少为 3、且 `passed: true` 才说明本次收图检查通过。

2026-11-11 修复前实测：5 秒内头部图像 `received: 0`，左右腕各 `received: 150`。
同时相机日志反复出现：

```text
The requested device with serial number 263022074810 is NOT found. Will Try again.
```

这说明相机服务主进程虽然为 `active`，头部相机却没有成功打开。
随后已通过预览和现场确认 `263022074037` 为头部相机，并更新了默认配置。
上面的错误是旧配置造成的；排查时要区分相机服务本次启动日志与历史日志。
查看设备和服务参数的只读命令：

```bash
journalctl --user -u kai0-official-cameras.service -n 100 --no-pager
systemctl --user show kai0-official-cameras.service --property=ActiveState,ExecStart
bash /home/rie/dev/xingyu.lai/kai0-official-dagger/ros-launch/run_ros.sh \
  rosparam get /camera_f
```

如果不是头部相机缺图，按 JSON 中实际缺失的那一路处理；
若没有正常 JSON，而是 Python 导入错误或无法联系 ROS master，则先修复对应环境或 ROS master。
不要只延长超时、删掉话题检查或使用旧图像绕过缺图问题。

**第三步：确认替换相机身份后，仅重建相机服务。**

先在采集终端 Ctrl+C，确认采集器已经退出。需要清理剩余四臂转发时：

```bash
bash /home/rie/dev/xingyu.lai/kai0-official-dagger/run_dagger.sh --stop
```

本机 `263022074037` 的头部身份已经确认。下列命令用于需要重建相机服务的场景，
执行前仍需确保它未被其他进程占用；若以后换相机，使用现场重新确认的序列号。

```bash
systemctl --user stop kai0-official-cameras.service
systemd-run --user --unit=kai0-official-cameras --collect \
  --property=WorkingDirectory=/home/rie/dev/xingyu.lai/kai0-official-dagger \
  --setenv=ROS_IP=127.0.0.1 \
  /bin/bash /home/rie/dev/xingyu.lai/kai0-official-dagger/native-camera/run_native_ros.sh \
  roslaunch /home/rie/dev/xingyu.lai/kai0-official-dagger/ros-launch/cameras_wj_dev.launch \
  front_serial:=263022074037
```

这只重建相机服务，不启动四臂驱动，不归位。
本机 launch 默认值已经更新为 `263022074037`，所以不带覆盖参数的正常启动也会使用它。
上面 `front_serial:=...` 是与默认值一致的显式覆盖，不会再次改写文件。
如果 `systemd-run` 提示同名单元仍存在，先检查它的状态和 `ExecStart`，
不要重启整个机器人或再并行启动一组相机节点。

接着重新执行第二步的 `--cameras-only` 检查。三路通过后，才回到第 5 节运行正常入口。
这一步不保证其他设备也已通过，仍需让入口完成四臂和完整话题检查。

**第四步：相机通过但完整检查仍失败时，再定位四臂话题。**

仅在四臂驱动本来已经正常运行时，执行下面的只订阅检查：

```bash
bash /home/rie/dev/xingyu.lai/kai0-official-dagger/ros-launch/run_ros.sh \
  /home/rie/dev/xingyu.lai/kai0-official-dagger/.ros-env/bin/python \
  /home/rie/dev/xingyu.lai/kai0-official-dagger/ros-launch/probe_official_topics.py \
  --samples 3 --timeout 15
```

如果启动失败后驱动已经被清理，此时四臂话题收不到消息是预期结果；
不要仅为重现探针而手动使能四臂。可以先只读查看上一次驱动日志：

```bash
journalctl --user -u kai0-official-arms.service -n 100 --no-pager
```

### 8.2 不同阻塞对应的处理重点

- **缺原始 CAN 反馈**：停在通信阶段，先确认设备身份和收发，再考虑模式恢复。
- **ROS 图像或关节话题缺消息**：停在话题阶段，按探针 JSON 找到缺失项，再查对应服务日志。
- **探针失败后四臂服务消失**：先核对 `failure_cleanup`，不要把清理结果当作原始故障。
- **相机服务 active，但头部 0 帧**：先处理相机序列号、占用或相机节点错误；不反复启动四臂。

模型准备成功不代表四臂和三路图像也已经就绪。相机映射已按现场确认更新；
图像检查不会启动、归位或使能四臂。重建相机服务仍应在采集会话退出后进行。

官方源码版本：`OpenDriveLab/kai0`，commit `9d93078c757840f50e75248c5c5a94ab7b41e13a`。
原始通用说明保留在 `/home/rie/dev/kai0-task-a/train_deploy_alignment/dagger/agilex/README.md`；
本机部署补丁及数据结构说明见
`/home/rie/dev/xingyu.lai/kai0-official-dagger/collector/README.txt`。
