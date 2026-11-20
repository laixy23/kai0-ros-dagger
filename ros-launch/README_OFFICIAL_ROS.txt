Kai0 ROS1 DAgger launch artifacts for wj-dev
===========================================

Deployment root:
  /home/rie/dev/xingyu.lai/kai0-official-dagger
These launch files belong in <root>/ros-launch. They run the isolated collector
compatibility copy in <root>/collector; they do not overwrite the original repo:
  /home/rie/dev/kai0-task-a
Official source reviewed: OpenDriveLab/kai0 commit
  9d93078c757840f50e75248c5c5a94ab7b41e13a.

The compatibility copy preserves the official model/control/data format, while
fixing user-owned storage, SSH TTY keyboard handling, takeover/resume lifecycle,
initial capture-thread shutdown, and RGB/BGR boundaries. Review its provenance/diff files
for the precise edits. The official Piper arm driver is not changed here.
Resume keeps policy output paused until both master nodes acknowledge slave mode
and their command topics are quiet. Queued actions and older in-flight model
results are discarded before requesting fresh actions after an intervention.
Policy output and the initial interpolation require fresh camera headers within
0.5s and follower feedback within0.25s. Expiration stops the collector's new
commands; it does not disable physical motors.

Isolated ROS environment
-------------------------
run_ros.sh always activates this environment using the micromamba shell hook:
  <root>/tools/micromamba
  <root>/.ros-env
and sources this isolated catkin overlay:
  <root>/ros_ws/devel/setup.bash
It does not source system ROS or the model server's Python venv. It clears
inherited PYTHONPATH, PYTHONHOME, venv, ROS and native-library search paths before
activation, disables user site-packages, and rejects foreign site-packages
reintroduced by a stale catkin overlay. The ROS environment and its own catkin
overlay are allowed. The official client source is added explicitly:
  /home/rie/dev/kai0-task-a/packages/openpi-client/src

Use the wrapper for every ROS command. In separate terminals:
  cd /home/rie/dev/xingyu.lai/kai0-official-dagger/ros-launch
  ./run_ros.sh roscore
  ../native-camera/run_native_ros.sh roslaunch ./cameras_wj_dev.launch
  ./run_ros.sh python ./probe_official_topics.py --cameras-only

Environment overrides, when needed, are DAGGER_ROOT, ROS_ENV_PREFIX,
MICROMAMBA_BIN, MAMBA_ROOT_PREFIX, ROS_WORKSPACE_SETUP and KAI0_SOURCE_ROOT.
Keep ROS_ENV_PREFIX pointed at the ROS environment, not the model server venv.
ROS_MASTER_URI defaults to http://127.0.0.1:11311; ROS_IP/ROS_HOSTNAME are retained
if explicitly set. UTC+8 timestamps use TZ=Asia/Shanghai. ROS node logs use:
  <root>/logs/ros

Build/dependency requirements
------------------------------
The ROS environment must provide ROS1 Noetic with matching Python/NumPy/cv2 and
cv_bridge ABIs. This setup targets RoboStack Python3.11 and NumPy1.26, not the
model server's JAX/CUDA environment. ROS package families include:
  catkin, actionlib, actionlib_msgs, control_msgs, roscpp, rospy, sensor_msgs,
  std_msgs, trajectory_msgs, geometry_msgs, message_generation, message_runtime,
  nav_msgs, std_srvs, tf, nodelet, realsense2_camera.
Python imports include torch (CPU works for the client), numpy<2, cv2, h5py,
dm_env, av, piper_sdk, websockets, typing_extensions and official openpi_client.

The isolated ros_ws/src contains copies of official piper and piper_msgs. To
initialize/build it before its devel overlay exists:
  cd /home/rie/dev/xingyu.lai/kai0-official-dagger/ros_ws
  DAGGER_SKIP_WORKSPACE=1 ../ros-launch/run_ros.sh catkin_init_workspace src
  DAGGER_SKIP_WORKSPACE=1 ../ros-launch/run_ros.sh catkin_make
Do not use DAGGER_SKIP_WORKSPACE for arm/collector execution: the generated
piper_msgs must be discoverable. Python node scripts in the official repo are
tracked without executable mode; the deployment copy needs executable scripts:
  chmod +x src/piper/scripts/*.py

piper_msgs generates PiperStatusMsg.msg and PosCmd.msg. The package CMake files
do not install Python nodes; a sourced devel/source workspace is required.

Camera-only checks
------------------
Use the separately built official camera overlay through run_native_ros.sh.
It combines ROS driver2.3.2 with SDK2.56.2 using native V4L2 and udev, while
preserving the original ROS environment. The original SDK2.50 camera runtime
developed blocked kernel USB threads during this bring-up. See native-camera/
receipts/build-receipt.json for exact sources, compatibility patches and hashes.
Compilation/linkage checks have passed; kernel7.0 streaming still needs testing
after the blocked kernel USB state is cleared.

The three RealSense D435 cameras publish 640x480 color at 30 Hz:
  camera_f: 263022074810 -> /camera_f/color/image_raw
  camera_l: 263022070737 -> /camera_l/color/image_raw
  camera_r: 263022072753 -> /camera_r/color/image_raw
Depth/infrared/confidence/pointcloud are disabled. initial_reset=false avoids an
unnecessary USB reset. Stop other camera consumers before starting ROS drivers.
This camera launch starts no Piper nodes or motion command publishers.

The include arguments match IntelRealSense/realsense-ros 2.3.2 rs_camera.launch.
That release explicitly sets the serial_no ROS parameter type to string.
Its color publisher declares rgb8 and has no color_format launch argument.
The compatibility collector requests bgr8 at RGB image bridge boundaries, then
retains the official BGR-to-RGB model conversion; depth stays passthrough.
Video export converts BGR arrays to RGB for PyAV's rgb24 input. The probe reports
the actual topic encoding so it can be checked before policy execution.
probe_official_topics.py only subscribes. Its full mode checks three cameras and
four arm feedback topics; --cameras-only excludes arm topics.

Arm launch IS NOT PASSIVE
-------------------------
After on-site clearance:
  ./run_ros.sh roslaunch ./arms_official_wj_dev.launch
  ./run_ros.sh python ./probe_official_topics.py

This includes the official piper/start_ms_piper_new.launch and preserves its
defaults. It requires four already mapped/UP 1 Mbps CAN interfaces:
  can_left_slave  serial 0027002C5547571120343930
  can_right_slave serial 004B002C4648570220393831
  can_left_mas    serial 002E002C5547571120343930
  can_right_mas   serial 001B00285547570420303135

Launching this enables hardware. Official mode=0 master initialization calls
EnableArm(7) regardless of the master auto_enable=false parameter. Therefore
auto_enable_slave=false DOES NOT make the combined official arm launch read-only.
Follower initialization closes the grippers. Ctrl+C stops the process but does
not disable motors; stopping a terminal is not an arm-disable operation.
Wait for on-site clearance and reachable emergency stop before launching it.
The feedback probe is passive; starting the official arm nodes is not passive.
Followers left in teach mode (CAN ctrl_mode=2) need separate supported mode
preparation before this launch. The official follower node does not exit teach.

Collector startup IS MOVEMENT-AFFECTING
---------------------------------------
Use the official Task_A/mixed_1 model service at:
  ws://127.0.0.1:8014
  /home/rie/dev/kai0-task-a/checkpoints/Task_A/mixed_1
The global language prompt remains "fold the cloth". There is no prompt override
flag. Do not append Advantage: positive for this baseline.

After cameras, four-arm feedback and on-site clearance have been confirmed, run
in an interactive terminal (SSH -t is sufficient; no DISPLAY/X server needed):
  ./run_official_collector.sh --confirm-motion

The collector still moves both followers to the official initial pose:
  [0, 0.32, -0.36, 0, 0.24, 0, 0.07]
then executes model actions immediately. It does not start paused. Pressing d
later enables/moves the masters through the same fixed intermediate pose, matches
the followers, then enters drag teaching. The intermediate pose is named "safe"
in the source; it is not a scene collision-avoidance plan.

All combined collector controls come from this same terminal:
  d       pause inference and align masters for takeover
  Space   start recording correction frames after takeover is ready
  s       save the correction episode
  w       delete the last save during its confirmation prompt
  other   accept the save during its confirmation prompt
  r       resume policy after confirmation is complete
The isolated combined path does not use pynput. The sibling collect_data.py's
standalone listener may still import it lazily if that separate path is invoked.

Storage and optional parameters
--------------------------------
Default data root:
  /home/rie/dev/xingyu.lai/kai0-official-dagger/data
The compatibility copy consistently honors --dataset_dir. Example outputs:
  <data>/<dataset_name>_inference_hdf5/aloha_mobile_dummy/episode_*.hdf5
  <data>/<dataset_name>_dagger_hdf5/aloha_mobile_dummy/episode_*.hdf5
The launcher creates its data root and chooses a timestamped dataset name.
Override the location/name with DATASET_DIR and DATASET_NAME, for example:
  DATASET_NAME=task_a_mixed1_trial01 ./run_official_collector.sh --confirm-motion

POLICY_HOST and POLICY_PORT default to 127.0.0.1 and 8014. COLLECTOR_DIR defaults
to <root>/collector. Extra official CLI arguments are passed after defaults:
  ./run_official_collector.sh --confirm-motion --episode_idx 0 --max_timesteps 900
  ./run_official_collector.sh --confirm-motion --publish_rate 30 --inference_rate 3
--max_timesteps bounds initial inference capture; it is not a whole-session
timeout. --max_publish_step bounds an inner loop; the outer loop repeats. Use
supervised terminal execution and inspect the actual output files after saving.
Video export remains enabled: official save_data expects it to be enabled.

Authoritative references
-------------------------
https://github.com/OpenDriveLab/kai0/tree/9d93078c757840f50e75248c5c5a94ab7b41e13a/train_deploy_alignment/dagger/agilex
https://github.com/IntelRealSense/realsense-ros/blob/2.3.2/realsense2_camera/launch/rs_camera.launch
https://github.com/IntelRealSense/realsense-ros/blob/2.3.2/realsense2_camera/launch/includes/nodelet.launch.xml
https://github.com/IntelRealSense/realsense-ros/blob/2.3.2/realsense2_camera/src/base_realsense_node.cpp
