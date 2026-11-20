Synthetic ROS1 integration validation
=====================================

Run validate_ros_dagger.py in the ROS environment after building piper_msgs and
installing the official openpi_client. Source the same ROS/catkin setup as the
collector. No robot, CAN or camera nodes are required or launched by this script.

Example on wj-dev after copying this validation directory into the project:

  bash ros-launch/run_ros.sh .ros-env/bin/python validation/validate_ros_dagger.py \
    --collector collector/agilex_openpi_dagger_collect.py --model-port 8014

The script refuses to reuse occupied loopback ports 11321 (ROS master) or 11322
(transparent policy relay). It overrides ROS_MASTER_URI for every child, starts
its own master and three fake camera/four fake arm feedback publishers, and only
subscribes to command topics. It never starts hardware drivers or opens CAN/USB.
It runs the actual collector in a PTY, forwards its requests to the real official
mixed_1 service on 127.0.0.1:8014, and counts/validates real model responses.

Two keyboard cycles run d -> Space -> s -> confirmation -> r. Checks cover master
alignment/control topic output, successful inference after each r, two fresh
inference recordings, two correction recordings, finite Nx14 state/actions, Nx2
base actions, and three decodable videos per episode. Fake images use rgb8, as
the official camera driver does. Distinct per-camera/per-phase RGB colors check
the actual model CHW request after preprocessing and decoded MP4 frames, within
10 levels for JPEG/video loss. Each inference phase needs a correct model request;
each video needs the expected median color and at least half its frames correct,
allowing a first cached frame around resume. The deployed temporal
smoothing path is the default; --blocking-inference checks the other path.

The fake master nodes apply mode commands asynchronously after 0.4 seconds and
publish the official Twist acknowledgement schema on master_control_status_left
and master_control_status_right. Config master enables a distinct ROS source
publishing /master/joint_left and /master/joint_right, reproducing the real node's
competing publisher behavior. Config slave exits teaching and stops that source.
Before every resumed model call the checks require fresh FC252/teach0 statuses,
the slave command for each side, a quiet master-command interval, and resumed
policy commands without any remaining fake master commands. Every observed ROS
command records its callerid, so source ownership is checked independently.

All logs, datasets and a machine-readable report live under validation/runs/.
Synthetic HDF5 root attrs include validation_only=True, training_allowed=False,
fake_ros_inputs=True and hardware_io=False, including on failed runs. A
DO_NOT_TRAIN.txt marker is written before any data. Never move these files into
real data directories or use them as demonstrations. HDF5 does not contain RGB
image datasets because the official writer stores RGB as separate MP4 videos.

This is an integration check of the official collector and real policy service.
It does not validate mechanical alignment, hardware feedback, camera serials,
real scene perception, actual takeover motion or collision safety.
