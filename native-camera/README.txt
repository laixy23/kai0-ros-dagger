Isolated native V4L2 RealSense build
===================================

Purpose: preserve the official ROS1 camera driver while replacing the old
RSUSB-backed SDK used by its RoboStack binary. This is a build and activation
artifact; no build step enumerates cameras, opens USB, or launches hardware nodes.

Official sources:
  https://github.com/realsenseai/librealsense/tree/v2.56.2
  https://github.com/realsenseai/realsense-ros/tree/2.3.2

SDK 2.56.2 introduced Ubuntu24.04/kernel6.8 support. The current host's kernel7.0
is newer and is not an officially tested combination for this version. Native
V4L2 is the official recommended backend for multicamera use. RGB-only uses
standard UVC streams, but operation on this kernel still needs a real stream test
after the kernel USB deadlock is cleared. A successful build alone is insufficient.

Independent project locations:
  native-camera/sources/librealsense-2.56.2
  native-camera/sdk-build
  native-camera/sdk-2.56.2
  native-camera/ros_ws
  native-camera/logs
  native-camera/receipts

build_native_camera.sh only reuses .ros-env's compiler/headers/libraries. It does
not modify .ros-env packages, /usr, system drivers, udev rules, firmware or kernel.
RSUSB, DDS, CUDA, graphical examples, tools, tests and Python bindings are disabled.
Any required source compatibility changes must be recorded as patches in receipts.
Completed build compatibility changes are two C++11-to-C++14 flags in the ROS
driver (the installed Eigen headers require C++14), and removal of the SDK's
system ldconfig install hook. No sensor/stream implementation changes were needed.
Matching libudev262.2 development headers and its development linker name reside
in native-camera/build-deps; runtime resolves the matching existing .ros-env
libudev.so.1. The final compiled SDK uses udev event monitoring, not its fallback
polling device watcher. Source commits, patches, hashes and ldd evidence are in
receipts/build-receipt.json. camera_stream_tested remains false until hardware
recovery and a deliberate camera test by the owner.

When the build is complete and hardware is ready, the owner can deliberately run:
  bash native-camera/run_native_ros.sh roslaunch /absolute/path/to/cameras_wj_dev.launch

This selects the native-camera ROS overlay and its SDK library path, leaving the
RoboStack camera package installed as a rollback. It must not be launched while
the old camera processes still own blocked kernel USB handles.

Source-level findings about the old stack:
  * ROS2.3.2 installs notification callbacks for every sensor, even disabled depth.
    SDK2.50's notification registration enables ERROR_POLLING_ENABLED.
  * ROS2.3.2 selects DEPTH as its base stream for D435 even in RGB-only mode, and
    its 1Hz temperature diagnostics read that depth sensor.
  * In SDK2.50 and2.56.2, opening a UVC sensor starts time_diff_keeper unconditionally;
    global_time_enabled=false changes timestamp conversion but does not stop the
    background get_device_time_ms polling. It is not a sufficient deadlock fix.

Primary references:
  https://github.com/realsenseai/realsense-ros/issues/3285
  https://github.com/realsenseai/librealsense/issues/14925
  https://github.com/realsenseai/librealsense/releases/tag/v2.50.0
  https://github.com/realsenseai/realsense-ros/blob/2.3.2/realsense2_camera/src/base_realsense_node.cpp
  https://github.com/realsenseai/librealsense/blob/v2.50.0/src/sensor.cpp
  https://github.com/realsenseai/librealsense/blob/v2.50.0/src/global_timestamp_reader.cpp
