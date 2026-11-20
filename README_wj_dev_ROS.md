# AgileX PiPER 双臂 DAgger 实机指南

这套 ROS 1 系统使用两只主臂示教、两只从臂操作衣物，采集器按 ROS 时间戳配对头部、左右腕相机的彩色图像与从臂反馈。下文以 Linux 控制机上的参考部署为例。已有工作环境的操作者可以直接从[启动前检查](#启动前检查)开始。只想阅读代码或做离线测试，可以从[项目首页](README.md)开始。

## 准备运行环境

从本仓库根目录执行以下命令。将第二行改为本机的 Kai0 源码目录，ROS 客户端需要其中的 `packages/openpi-client/src`。参考部署使用 RoboStack ROS 1 Noetic、Python 3.11、NumPy 1.26 和 PiPER SDK 0.7.0a1。[Kai0 源码基线](https://github.com/OpenDriveLab/kai0/tree/9d93078c757840f50e75248c5c5a94ab7b41e13a) · [ROS 依赖版本](setup/ros-env-pins.txt)

```bash
export DAGGER_ROOT="$(pwd -P)"
export KAI0_SOURCE_ROOT="$HOME/src/kai0"
test -d "$KAI0_SOURCE_ROOT/packages/openpi-client/src/openpi_client"
```

新控制机需要先准备可执行的 Linux x86-64 版 `tools/micromamba`，参考部署使用 2.10.0。下面的命令建立隔离的 ROS 环境，并直接编译仓库内已经适配的 `ros_ws/src/piper` 和 `piper_msgs`。历史初始化脚本会把已有驱动与上游版本做比对，重建本版本时应使用这里的源码。[micromamba 版本与校验值](tools/micromamba-install.json) · [PiPER SDK 版本](firmware-compat/requirements-sdk-v3.txt)

```bash
bash setup/create_ros_environment.sh

ros_build() {
  DAGGER_SKIP_WORKSPACE=1 bash "$DAGGER_ROOT/ros-launch/run_ros.sh" "$@"
}
ros_build python -m pip install --index-url https://download.pytorch.org/whl/cpu 'torch==2.7.1'
ros_build python -m pip install 'python-can==4.6.1' 'dm-env==1.6' 'dm-tree==0.1.9' 'websockets==15.0.1'
ros_build python -m pip install --no-index --no-deps --require-hashes \
  --find-links "$DAGGER_ROOT/firmware-compat/artifacts" \
  -r "$DAGGER_ROOT/firmware-compat/requirements-sdk-v3.txt"
(cd "$DAGGER_ROOT/ros_ws/src" && ros_build catkin_init_workspace)
ros_build catkin_make -C "$DAGGER_ROOT/ros_ws" \
  -DPYTHON_EXECUTABLE="$DAGGER_ROOT/.ros-env/bin/python" -DCMAKE_BUILD_TYPE=Release
bash native-camera/build_native_camera.sh
```

相机脚本会另外构建 librealsense 2.56.2 和 realsense-ros 2.3.2。日常运行通过 `ros-launch/run_ros.sh` 加载 ROS 工作空间，相机通过 `native-camera/run_native_ros.sh` 再加载相机驱动。[相机构建说明](native-camera/README.txt)

## 配置硬件和策略服务

| 项目 | 需要与现场匹配的配置 |
| --- | --- |
| 四路 CAN | 将四个 USB-CAN 适配器的序列号分别映射到左主 `can_left_mas`、右主 `can_right_mas`、左从 `can_left_slave`、右从 `can_right_slave`，速率均为 1 Mbps。映射定义在 [CAN 配置脚本](setup/activate_can_by_serial.py) 的 `ADAPTERS`。 |
| 三台相机 | 头部、左腕、右腕分别对应启动参数 `front_serial`、`left_serial`、`right_serial`。默认使用 640 × 480、30 Hz 彩色图像。[相机 launch 文件](ros-launch/cameras_wj_dev.launch) |
| 策略服务 | 在独立的 Kai0 环境部署 `Task_A/mixed_1` 权重和 OpenPI WebSocket 服务。采集器使用任务文本 `fold the cloth`，默认连接 `127.0.0.1:8014`。可通过 `POLICY_HOST` 和 `POLICY_PORT` 指定地址。控制机上的入口还会检查名为 `kai0-official-mixed1.service` 的用户服务。[上游策略服务启动示例](https://github.com/OpenDriveLab/kai0/blob/9d93078c757840f50e75248c5c5a94ab7b41e13a/train_deploy_alignment/dagger/agilex/README.md#on-the-inference-machine-gpu-server) |
| USB 三踏板（可选） | 输入程序会核对 USB 厂商、产品和序列号。更换踏板或 Linux 用户组时，先核对[设备识别](operator/foot_pedal_device.py)和 [udev 规则](operator/setup_foot_pedal_access.py)，再校准左、中、右三块踏板的绑定。键盘入口可独立使用。 |

在采集会话退出后先查看 CAN 配置计划，核对四个物理位置再应用。最后用 `ip` 检查接口和速率，机械臂实际反馈在启动检查中另行验证。

```bash
python3 setup/activate_can_by_serial.py --plan
sudo python3 setup/activate_can_by_serial.py --apply
ip -details link show type can
```

### ROS master 与相机

参考部署由同一用户的 systemd 管理服务。先让用户服务管理器获得前面设置的路径，以便四臂驱动和相机进程也能找到工作空间。若相应服务已在运行，直接进入下一节。以下命令在本机回环地址上启动尚未建立的 ROS master 和相机服务。

```bash
systemctl --user import-environment DAGGER_ROOT KAI0_SOURCE_ROOT
systemd-run --user --unit=kai0-official-roscore --collect \
  --property="WorkingDirectory=$DAGGER_ROOT" --setenv=ROS_IP=127.0.0.1 \
  /bin/bash "$DAGGER_ROOT/ros-launch/run_ros.sh" roscore
```

先在当前终端设置 `FRONT_SERIAL`、`LEFT_SERIAL`、`RIGHT_SERIAL` 为三台相机实际的序列号，再启动相机服务。

```bash
systemd-run --user --unit=kai0-official-cameras --collect \
  --property="WorkingDirectory=$DAGGER_ROOT" --setenv=ROS_IP=127.0.0.1 \
  /bin/bash "$DAGGER_ROOT/native-camera/run_native_ros.sh" \
  roslaunch "$DAGGER_ROOT/ros-launch/cameras_wj_dev.launch" \
  "front_serial:=${FRONT_SERIAL:?请设置头部相机序列号}" \
  "left_serial:=${LEFT_SERIAL:?请设置左腕相机序列号}" \
  "right_serial:=${RIGHT_SERIAL:?请设置右腕相机序列号}"
```

## 启动前检查

先确认模型、ROS master 和相机三个用户服务均为 `active`。查询模型服务本次启动的日志，核对加载的 checkpoint 和预热结果。本机部署还可用 `ss -ltnp 'sport = :8014'` 核对默认端口的监听进程。然后订阅相机话题。探针会输出各路的 `received`、分辨率和时间戳。三路都在 10 秒内收到至少 3 帧时，结果中会出现 `"passed": true`。

```bash
systemctl --user show --property=Id,ActiveState \
  kai0-official-mixed1.service kai0-official-roscore.service kai0-official-cameras.service
bash ros-launch/run_ros.sh python ros-launch/probe_official_topics.py --cameras-only --samples 3 --timeout 10
```

**四臂驱动已经运行时**，可以用 `bash run_dagger.sh --check` 检查四臂反馈和图像。这条命令只做检查。首次启动由正常采集入口准备四臂驱动。

使用踏板时，先在同一终端校准并测试。校准按左、中、右顺序各踩下并松开 3 次，配置写入指定的文件。测试模式会打印识别到的动作。

```bash
export DAGGER_PEDAL_CONFIG="$DAGGER_ROOT/_runtime/foot-pedal-local.json"
bash run_dagger_pedal.sh --calibrate
bash run_dagger_pedal.sh --test-pedal
```

## 开始、保存和结束一轮

在本机交互终端或带 TTY 的 SSH 终端中选择一个入口。**正常启动会准备并使能机械臂，从臂夹爪可能闭合。随后两只从臂移到初始姿态并立即执行策略。** 启动前清空运动范围、确认夹爪内无物，并使现场急停可用。

| 操作方式 | 启动一轮 |
| --- | --- |
| 键盘 | `bash run_dagger.sh` |
| 键盘与三踏板 | `bash run_dagger_pedal.sh` |

1. **接管。** 按 `d` 或左单踩。策略暂停发出新动作，推理记录再采约 1 秒真实反馈。双主臂对齐当前从臂位置后，一边稳定保持至少 3 秒，一边准备跟随。等两侧确认进入示教，第一段纠正就自动开录。它会带上就绪前最多 1 秒可用的相机和从臂采样。对齐与等待可能留下未录制的时间，分析时应按段读取。
2. **保存。** 按 `s` 或中单踩，先暂停主臂向从臂发送新目标，再保存当前纠正段。保存后的约 10 秒内，按 `w` 或中双踩可删除刚保存的一段。其他普通按键、单踩或超时会保留它。在确认窗口里按 `r` 或右单踩，第一次仅确认保留。
3. **继续。** 确认结束后，用空格或左单踩开始另一段纠正。用 `r` 或右单踩把控制权交回策略。恢复推理前，采集器会等待双主臂模式确认、示教转发停止，并重新获取策略动作。
4. **退出。** 要保留当前人工纠正，先保存，再按 `Ctrl+C` 或右双踩。正常退出会导出剩余的有效推理段，并停止本轮四臂驱动。软件退出后电机可能仍然使能。若采集器已退出而驱动仍在运行，可执行 `bash run_dagger.sh --stop` 清理。

双踩按第二次踩下计算，须在第一次松开后 0.5 秒内发生。中、右单踩会等待该窗口结束再执行。双主夹爪的交接由正式键盘和踏板入口共同使用。[交接与录制实现说明](collector/README.txt)

## 数据文件

默认输出根目录为 `$DAGGER_ROOT/data`。设置 `DATASET_DIR` 可更改位置，`DATASET_NAME` 可指定本轮名称。默认名称包含时间戳和进程号。两类片段独立编号，下例为同一轮的第一份推理记录和第一份纠正记录。

```text
data/
├── <name>_inference_hdf5/aloha_mobile_dummy/
│   ├── episode_0.hdf5
│   └── video/…
└── <name>_dagger_hdf5/aloha_mobile_dummy/
    ├── episode_0.hdf5
    └── video/
        ├── cam_high/episode_0.mp4
        ├── cam_left_wrist/episode_0.mp4
        └── cam_right_wrist/episode_0.mp4
```

| HDF5 字段 | 形状 | 含义 |
| --- | --- | --- |
| `/observations/qpos`、`/observations/qvel`、`/observations/effort` | `(T, 14)` | 来自左右从臂的 ROS 关节反馈，按先左后右排列。 |
| `/action` | `(T, 14)` | 与当前观测配对的下一次有效采样的实测位置。每侧的七个量依次为六个关节位置和夹爪开度。 |
| `/base_action` | `(T, 2)` | 底盘线速度和角速度。固定双臂配置写入零值。 |

`T` 是写出的观测与动作对数。彩色图像导出到同编号的三个 MP4。复用数据集名称时，先核对两个目录各自已有的编号。[HDF5 与视频写入实现](collector/collect_data.py)

## 常见问题

| 现象 | 从哪里查起 |
| --- | --- |
| 四个 CAN 接口都已 `UP`，启动仍报缺少关节反馈 | 核对脚本的序列号映射、对应臂的供电和 CAN 回包。接口状态与机械臂反馈分别检查。 |
| 相机服务在运行，某一路 `received` 为 0 | 按头部、左腕、右腕的实际位置核对相机启动参数。通过 `journalctl --user -u kai0-official-cameras.service -n 60` 查看本次启动日志。 |
| `--check` 提示四臂驱动未运行 | 它只检查已有的 `kai0-official-arms.service`。如果刚发生启动失败，查看 `_runtime/keyboard-prepare.json` 中的 `error`、`drivers_started` 和 `failure_cleanup`，定位驱动被清理前的错误。 |
| 已保存，右踩一次仍处于示教 | 查看终端是否刚确认保留文件。确认窗口结束后，再右单踩或按 `r` 恢复推理。 |
