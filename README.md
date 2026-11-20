# Kai0 ROS1 DAgger for AgileX PiPER

A ROS 1 collector for recording policy rollouts and human corrections during bimanual cloth folding.

双臂折衣时，如果策略漏抓了衣角，操作者可以暂停推理，用两只主臂带动从臂完成纠正，再把控制权交回策略。这个 ROS 1 采集器把策略执行和人工纠正分别存成 episode，支持在同一轮任务里多次接管。

实现基于 OpenDriveLab 的 Kai0 AgileX DAgger 采集器。参考平台使用两只主臂、两只从臂、头部和左右腕三台 RealSense 相机，以及独立的 Kai0 WebSocket 策略服务。键盘和可选的 USB 三踏板共用同一套采集逻辑。[实机部署与操作指南](README_wj_dev_ROS.md) · [Kai0 上游实现](https://github.com/OpenDriveLab/kai0/tree/9d93078c757840f50e75248c5c5a94ab7b41e13a/train_deploy_alignment/dagger/agilex)

## 离线查看和测试

在安装了 Git、Bash 和 Python 3 的电脑上，可以先查看两个入口的帮助。最后一条命令用模拟输入验证接管、保存和恢复推理的踏板指令。

```bash
git clone https://github.com/laixy23/kai0-ros-dagger.git
cd kai0-ros-dagger
bash run_dagger.sh --help
bash run_dagger_pedal.sh --help
python3 -m unittest discover -s collector -p 'test_pedal_commands.py'
```

## 一轮采集怎么操作

| 目的 | 键盘 | 三踏板 |
| --- | --- | --- |
| 暂停策略，让主臂对齐从臂并开始录制纠正 | `d` | 左单踩 |
| 暂停跟随，保存当前纠正段 | `s` | 中单踩 |
| 保存并确认后，继续录一段人工纠正 | 空格 | 左单踩 |
| 把控制权交回策略 | `r` | 右单踩 |
| 结束本轮 | `Ctrl+C` | 右双踩 |

**接管就绪后会自动开录。** 保存后有约 10 秒的确认窗口。在窗口里按 `r` 或右单踩只会确认保留，再操作一次才恢复推理。需要保留手上的纠正段时，先按 `s` 保存，再退出。[完整按键、踏板和启动步骤](README_wj_dev_ROS.md)

## 录到了什么

每轮默认在 `data/` 下生成两组目录。策略段放在 `<名称>_inference_hdf5/`，纠正段放在 `<名称>_dagger_hdf5/`。两组各自从 `episode_0` 编号。同一段的 HDF5 和三路 MP4 可按目录和编号配对。[示例目录与字段](README_wj_dev_ROS.md#数据文件)

读取训练数据时，`/observations/qpos[t]` 是当前有效采样的双从臂状态，`/action[t]` 是下一次有效采样的双从臂实测位置。二者都是 14 维，先左后右，每侧为六个关节和一个夹爪开度。[采集与配对逻辑](collector/agilex_openpi_dagger_collect.py) · [HDF5 与视频写入](collector/collect_data.py)

接管逻辑从采集器的 `enter_dagger_mode` 进入，主臂模式切换由 PiPER 驱动处理。三踏板的输入映射在操作端。[PiPER 驱动](ros_ws/src/piper/scripts/piper_start_ms_node_new.py) · [踏板输入](operator/foot_pedal_device.py)

本实现的来源与改动见 [采集器说明](collector/README.txt)、[上游版本清单](collector/upstream-manifest.json) 和 [Kai0 上游许可证](licenses/Kai0-LICENSE.txt)。
