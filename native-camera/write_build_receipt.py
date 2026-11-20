#!/usr/bin/env python3
"""Inspect build artifacts only; never import the SDK or touch camera devices."""
import hashlib
import json
import os
from pathlib import Path
import platform
import subprocess


def capture(*command):
    return subprocess.check_output(command, text=True).strip()


def describe(path):
    return {"path": str(path), "bytes": path.stat().st_size,
            "sha256": hashlib.sha256(path.read_bytes()).hexdigest()}


root = Path(os.environ.get("DAGGER_ROOT", "/home/rie/dev/xingyu.lai/kai0-official-dagger")) / "native-camera"
sdk = root / "sdk-2.56.2"
driver = root / "ros_ws/devel/lib/librealsense2_camera.so"
cache = (root / "sdk-build/CMakeCache.txt").read_text()
compiler_flags = (root / "sdk-build/CMakeFiles/realsense2.dir/flags.make").read_text()
assert "-DRS2_USE_V4L2_BACKEND" in compiler_flags, "Wrong compiled SDK backend"
assert "-DUSING_UDEV" in compiler_flags, "SDK fell back to polling device watcher"
required_flags = ("FORCE_RSUSB_BACKEND", "BUILD_WITH_DDS", "BUILD_WITH_CUDA", "BUILD_EXAMPLES",
                  "BUILD_GRAPHICAL_EXAMPLES", "BUILD_GLSL_EXTENSIONS", "BUILD_TOOLS", "BUILD_UNIT_TESTS")
for flag in required_flags:
    assert f"{flag}:BOOL=OFF" in cache, f"Unexpected SDK build flag {flag}"
driver_ldd = capture("ldd", str(driver))
sdk_ldd = capture("ldd", str(sdk / "lib/librealsense2.so"))
assert "not found" not in driver_ldd + sdk_ldd, "Missing runtime libraries"
assert str(sdk / "lib/librealsense2.so.2.56") in driver_ldd, "ROS driver linked another SDK"
assert "libopencv_core.so.410" in driver_ldd, "ROS driver did not link OpenCV4.10"
sdk_source = root / "sources/librealsense-2.56.2"
ros_source = root / "ros_ws/src/realsense-ros"
receipt = {
    "build_only": True, "hardware_io": False, "camera_stream_tested": False,
    "kernel": platform.release(), "backend": "RS2_USE_V4L2_BACKEND",
    "device_watcher": "udev", "private_udev_headers": str(root / "build-deps/include"),
    "sdk": {"url": "https://github.com/realsenseai/librealsense", "tag": "v2.56.2",
            "commit": capture("git", "-C", str(sdk_source), "rev-parse", "HEAD"),
            "flags": {flag: False for flag in required_flags}},
    "ros": {"url": "https://github.com/realsenseai/realsense-ros", "tag": "2.3.2",
            "commit": capture("git", "-C", str(ros_source), "rev-parse", "HEAD")},
    "artifacts": [describe(driver), describe((sdk / "lib/librealsense2.so").resolve())],
    "patches": [describe(root / "receipts/sdk-compatibility.patch"),
                describe(root / "receipts/ros-compatibility.patch")],
    "sdk_source_changes": capture("git", "-C", str(sdk_source), "diff", "--stat"),
    "ros_source_changes": capture("git", "-C", str(ros_source), "diff", "--stat"),
    "ros_driver_ldd": driver_ldd.splitlines(), "sdk_ldd": sdk_ldd.splitlines(),
}
receipts = root / "receipts"
receipts.mkdir(exist_ok=True)
(receipts / "build-receipt.json").write_text(json.dumps(receipt, indent=2) + "\n")
print(json.dumps({"receipt": str(receipts / "build-receipt.json"),
                  "backend": receipt["backend"], "hardware_io": False,
                  "camera_stream_tested": False}, indent=2))
