# Kai0 ROS1 DAgger

本仓库保存 2026 年 11 月 13 日部署在 `wj-dev` 的双臂折衣 DAgger 代码。模型推理与人工示教之间可以多次切换，每段分别保存 HDF5 和头部、左右腕三路视频。键盘和 USB 三踏板共用一个采集器。[实机操作手册](README_wj_dev_ROS.md)

| 入口 | 用途 |
| --- | --- |
| `run_dagger.sh` | 键盘操作。按 `d` 接管，主臂对齐且跟随就绪后自动录制。 |
| `run_dagger_pedal.sh` | 踏板操作。左踩接管，中踩暂停跟随并保存，右踩恢复推理。保存确认窗口里第一次右踩确认保留。 |

部署目录为 `/home/rie/dev/xingyu.lai/kai0-official-dagger`。运行依赖 Linux、RoboStack ROS1 Noetic、已构建的catkin 工作空间和独立的 Kai0 模型服务。环境使用的 micromamba 版本与哈希记录在 [安装记录](tools/micromamba-install.json)。部署的 Piper 驱动位于 `ros_ws/src/piper`，相机驱动构建脚本与补丁位于 `native-camera`。[环境和设备准备](setup/README_OFFICIAL_ROS.txt)

采集器基于 OpenDriveLab/Kai0 提交 `9d93078c757840f50e75248c5c5a94ab7b41e13a`。部署版主采集器的 SHA-256 为 `1f559d77ddcb8af2a00d1895d391dab0421f91efe2e833131bfb4cbca6c2668c`。[采集器改动](collector/README.txt) · [源码校验清单](collector/upstream-manifest.json) · [11 月 13 日部署记录](_runtime/startup-enable-current.json) · [Kai0 上游许可证](licenses/Kai0-LICENSE.txt)
