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
for the precise edits. The isolated Piper driver copy adds follower-readiness
interlocks; the original repository's driver remains unchanged.
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

The combined launch still performs follower initialization, including enable
and gripper closure. Managed master nodes instead require an already enabled
CAN1 holding state and only validate it. The generic official master path with
the readiness interlock disabled retains its legacy EnableArm initialization.
Ctrl+C stops the process but does
not disable motors; stopping a terminal is not an arm-disable operation.
Wait for on-site clearance and reachable emergency stop before launching it.
The feedback probe is passive; starting the official arm nodes is not passive.
The Bash entry prepares already-enabled followers left in teach mode (CAN
ctrl_mode=2) using their fresh measured positions as holding targets. Its
automatic follower path never requests DisableArm, ResetPiper or gripper motion.
It verifies all six drivers stay enabled while returning to CAN joint control;
pose drift is diagnostic only. Normal startup now also handles valid feedback
showing disabled or partially enabled arms: preload a fresh actual six-joint
target, restore CAN1 at that same target if needed, send EnableArm(7), reseed the
same target, and verify the actual six motor-enable bits with the existing
feedback criteria. The new setup/enable_arm_current_pose.py helper never sends
DisableArm, ResetPiper, a zero/initial-pose target or a gripper command. Already
enabled CAN1 arms keep their passive fast path. --check and --stop do not enable.
Faulted or stale-feedback arms retain the existing failure behavior; the old
disable/re-enable tool is not an automatic fallback.
The collector then interpolates to the same official initial pose. Master-arm
startup recovery avoids DisableArm, ResetPiper and gripper commands. The existing
already-enabled master helper still sends no EnableArm; the new startup branch
enables only known disabled/partial-enabled arms at their measured pose.
Fresh enabled CAN1 masters pass without any write. Enabled masters
with actual feedback in standby/teaching use one fixed measured-pose target to
return to CAN1. For a silent old FA role, only a single FC role request is sent
before native actual feedback is recovered. No pose/teach/CAN1 command is allowed
until all raw components are fresh, fault-free, WIDTH and all six drivers enabled.
The FA bootstrap requires an identifiable cached firmware version >=1.8.9;
the current firmware receipt reports S-V1.9. This is cached identification, not
a new hardware query. Unknown/older firmware with missing feedback blocks startup.
The managed master driver then only verifies the prepared enabled CAN1 state;
it does not repeat role changes or initialization control writes.

Fault, stale or missing feedback retains the existing failure behavior. A
disabled motor with complete valid feedback is now enabled at its actual pose
during normal startup, then the existing strict final verification still
requires all six actual motors enabled. Additional1-degree drift limits have been removed from master
and follower startup and FC restoration. Drift is diagnostic only. Refresh the
native measured anchor once before the first holding-target write, then keep that
single target; do not continuously chase observed drift.
There is no fallback to the legacy supported torque-off master recovery helper.
The original independent repair helper remains available only for explicit manual
use. Initialisation of the follower driver still enables followers and closes
their grippers; these are separate from this recovery helper.

The vendor describes no-disable mode switching for firmware1.8.9 and later:
  https://github.com/agilexrobotics/pyAgxArm/discussions/53
Firmware behavior during an enabled mode transition still needs on-site validation.
In the silent-FA bootstrap, position cannot be independently checked before the
first actual feedback after FC; no software test proves zero transient movement
or torque drop during that interval. First use must be tested low and supported.

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
the followers. At verified arrival, holding and preparation start concurrently:
one path keeps the same final targets and verifies three continuous seconds of
fresh enabled actual alignment; the other performs fixed measured-pose follower
probes, requests prepare_master, starts native FC actual feedback forwarding and
waits for both follower acknowledgements. Preparation cannot release FA. Both
conditions must be currently satisfied before release_master, so waiting is
max(3 seconds, preparation time), within a15-second overall preparation deadline.
Transient unready states continue holding; faults, cancellation and timeout block
release. The driver rechecks current per-component freshness and the same six-
joint target before consuming a live preparation. Gripper position differences
or gripper-only goal changes do not block alignment or preparation; normal
gripper following resumes with the complete valid FA stream. FA itself is the
zero-force release point; there are no managed-path
post-FA EnableArm, extra teach command or fixed six-second waits.

An independent forwarding loop is armed before FA. It immediately forwards each
complete fresh hardware-native155/156/157/159 group, without waiting for the mode
callback to finish. It rejects local TX loopback using is_rx, old epochs, stale or
partial groups and unknown/ANGLE gripper modes. ROS headers keep
the earliest component's CAN timestamp; SDK aggregate caches and zero fallbacks
are not used. FA readiness requires both the first native publication and the
corresponding follower command-source acknowledgement. Recording still waits for
both fresh master mode acknowledgements and command observations from this
request; it does not require continuously advancing commands while idle.

Both masters wait for the first complete valid FA group and both follower source
acknowledgements without a fixed deadline. The added2s first-group and2.5s pair-ACK
deadlines are removed, as is the collector's15s FA entry timeout. A valid group
forwards immediately. Existing shutdown/manual cancellation can end the wait.
FC priming retains its0.25s missing-data failure. FA silence after the first group
waits in the same epoch before or after acknowledgement;0.25s triggers a notice
only. FA has already released the master, so waiting does not imply following or
position holding before native data arrives. FC preparation and r retain their
existing bounded waits.

WIDTH input and execution limits are separate: retain signed raw values within
the SDK's absolute150000um abnormal-data bound, and separately clamp the outgoing
command to0..80000um as the original follower driver does. No additional
first-FA/gap geometric target-difference gate remains. Per-ID counts, kernel timestamps, ages,
raw bytes, missing/stale/nonadvancing components, group span and rejected-frame
reasons are included in timeout/failure diagnostics. Frame freshness and validity
limits remain unchanged; FA first-group and acknowledgement wait deadlines are removed.

If entry fails after FA, forwarding closes; FC is requested without disabling,
then fresh actual pose and six enables are verified before holding the newly
measured current pose. A previously saved alignment pose is not forced back onto
a master that may already have moved. After successful entry, missing data waits
passively in the same FA phase rather than permanently closing forwarding at0.25s.
The SDK maintainer notes that an unmoving master may go silent:
  https://github.com/agilexrobotics/piper_sdk/issues/112#issuecomment-4515586181
Only new complete fresh hardware-RX groups can resume publication. The additional
first-FA and gap-resume2-degree target-difference gates have been removed, as has
the1-degree actual-priming pose gate. Do not replay old commands, change modes or
assign fresh timestamps to caches while waiting. Existing mode-status text reports
waiting for input. Invalid data, receiver errors and cancellation still close the
gate without an old-pose pullback. This is
an entry interlock, not a continuous emergency-stop controller. Physical following
still has controller-frame, ROS and actuator latency and needs on-site validation.
An explicit master-disable topic request cancels a pending transition immediately
and latches cancellation for that driver process. Old queued enable/teach requests
cannot re-enable it; stop the round and use the supported new-round entry to recover.
The intermediate pose is named "safe"
in the source; it is not a scene collision-avoidance plan.

An unsuccessful explicit r request leaves policy paused and the session/data
intact; report the missing side/mode or continued publishing and allow a later
manual r retry. Do not terminate the entire round on a handoff timeout. Repeated
r keys while a handoff is pending/in progress do not queue extra transitions.
Teaching preparation does not require fresh camera inputs or a ROS node-name
prefix: transport, real follower feedback and executed command acknowledgements
prove the path. Policy output retains its existing camera/input validation.

All combined collector controls come from this same terminal:
  d       pause inference, align masters, hold3s, enter teaching and auto-record
  Space   resume following and restart recording after saving a correction segment
  s       pause following, stop collection and save the correction episode
  w       delete the last save during its confirmation prompt
  other   accept the save during its confirmation prompt
  r       resume policy after confirmation is complete
The isolated combined path does not use pynput. The sibling collect_data.py's
standalone listener may still import it lazily if that separate path is invoked.

Each fresh inference-to-DAgger entry prepends up to one second of actual camera
and follower samples before the original FA-ready recording point. A data-only
worker uses non-destructive message snapshots during entry; it stops and joins
before the main recorder starts, then the normal add_frame path imports only
source timestamps in [ready-1s, ready]. Live recording skips already imported
source frames. No waiting, motion command, ACK/timeout/pose check or padding is
added. The worker and pending data are cleared on failed/interrupted entry;
post-save recording restarts do not reuse this prefix. Inference still samples
its separate one-second tail before saving.

Save pauses both masters' native FA target publication through the existing
config topics before export and its confirmation window. The followers hold
their last accepted joint/gripper targets; no new hold/mode/enable/disable CAN
command is added. In-flight targets may finish. Empty saves also stop capture
and pause following. The gate only affects FA publication: FC priming and policy
commands remain unaffected. Actual recording restart resumes fresh native input;
confirmation/deletion alone does not resume. No new ACK, timeout or pose check
is introduced.

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
