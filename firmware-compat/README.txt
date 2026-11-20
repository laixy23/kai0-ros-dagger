Piper S-V1.9 firmware compatibility staging (not installed remotely)
=================================================================

All four arm controllers report S-V1.9 firmware. Official Piper SDK documentation
requires the V3 interface for firmware >=1.8-8. SDK 0.6.2 lacks that interface.
This directory stages the published official SDK 0.7.0a1 prerelease and minimal
class-selection changes. The original Kai0 repo and existing ROS environment
are unchanged by staging these files. The model environment is not involved.

Pinned official source and artifacts
------------------------------------
Repository: https://github.com/agilexrobotics/piper_sdk
Commit/tag: b847e5393a7ce6ff8508d713eca7bdd3a42421b4 / 0.7.0a1
The tag and HEAD were independently checked with git ls-remote.
The 195 Python package files in the published wheel match the pinned repository
source exactly. The vendor/piper_sdk directory contains that repository source.

SDK wheel:
  artifacts/piper_sdk-0.7.0a1-py3-none-any.whl
  SHA256 599677152cc8bed8e0dd11cec89f46e0e7e95b2b2be002ffe74d1459e17e4d51
Rollback wheel:
  artifacts/piper_sdk-0.6.2-py3-none-any.whl
  SHA256 d4aa1088fd6716f2f8c81a269dc5fb88b4c875a859e1c06e95bf2e9e8a6493c6

Only the isolated deployment driver copy changes two lines: explicit import and
construction of C_PiperInterface_V3. All existing MOVE P/J, master alignment,
keyboard, gripper width commands, topic names and SDK DTO field accesses remain
the official ones. The source node is Kai0 commit 9d93078; compatibility.patch
records exact deployment-relative changes. A normalized AST comparison verifies
that the driver changes only its SDK class selection.

prepare_follower_mode.py is a staged copy of the previous independent helper.
It now pins SDK 0.7.0a1, constructs V3, and rejects gripper feedback with byte7
other than 0 (WIDTH). Its supported-arm disable/settle/mode/hold/enable sequence
is unchanged. test_prepare_follower_mode.py exercises only fake interfaces and
raw byte fixtures; it never imports a real SDK interface or opens CAN.

V3 gripper units must be checked
--------------------------------
V3 preserves legacy GetArmGripperMsgs and GetArmGripperCtrl DTO fields, including
grippers_angle. In ANGLE mode those raw values are angles, not micrometers.
The official Kai0 node divides by 1e6, so all four actual 0x2A8 messages must have
byte7 == 0 (WIDTH) before running the node or using the state as policy input.

preflight_v3_feedback.py passively checks all four CAN interfaces, requires all
11 component frames fresh within 0.2s, rejects arm/driver faults and ANGLE mode,
and sends no frames. It never imports/constructs the Piper SDK. It does not
automatically change gripper mode. A successful preflight does not enable arms.

check_sdk_contract.py is the separate offline import/signature/DTO contract
tool. It imports SDK and constructs only DTO/parser classes, never an interface
instance, CAN bus, or socket. It rejects socket creation through an audit hook.
Run it after installing the SDK, before any driver launch.

The offline tool also checks exact inherited SDK configuration encodings using
fake owner state and a memory callback only:
  DisableArm(7)                    -> one 0x471 frame, 0701000000000000
  MasterSlaveConfig(0xFC,0,0,0)    -> one 0x470 frame, fc00000000000000
Neither SDK method calls EnableArm, JointCtrl or GripperCtrl. This proves SDK
behavior only; it does not establish firmware torque side effects. The official
V3 piper_set_slave.py demo says an arm previously in leader mode must restart
AFTER setting follower mode. These configuration writes are not performed by
the contract tool or installation plan.

Reviewable installation plan (root executes only after reviewing artifacts)
-------------------------------------------------------------------------
From /home/rie/dev/xingyu.lai/kai0-official-dagger, after placing this complete
directory at ./firmware-compat:

  ./ros-launch/run_ros.sh python -m pip freeze > setup/ros-pip-before-sdk-v3.txt
  ./ros-launch/run_ros.sh python -m pip install --no-index --find-links ./firmware-compat/artifacts --no-deps --require-hashes --force-reinstall -r ./firmware-compat/requirements-sdk-v3.txt
  ./ros-launch/run_ros.sh python ./firmware-compat/check_sdk_contract.py --node ./ros_ws/src/piper/scripts/piper_start_ms_node_new.py --output ./setup/sdk-v3-contract.json

Then, after checking the staged hashes/provenance and backing up deployment
copies, copy only these three runtime files:

  firmware-compat/piper_start_ms_node_new.py -> ros_ws/src/piper/scripts/piper_start_ms_node_new.py
  firmware-compat/prepare_follower_mode.py   -> setup/prepare_follower_mode.py
  firmware-compat/preflight_v3_feedback.py  -> setup/preflight_v3_feedback.py

Keep the driver executable. No catkin message or build-file changes are needed.
Do not replace files in /home/rie/dev/kai0-task-a or the original local checkout.
Once the physical controller feedback is restored, the next check is passive:

  ./ros-launch/run_ros.sh python ./setup/preflight_v3_feedback.py --output ./setup/v3-width-feedback.json

This plan upgrades only piper_sdk in .ros-env, with --no-deps and a pinned wheel
hash. Existing python-can/typing_extensions and the model environment remain as
they are. SDK 0.6.2 rollback is available using the same pip command with
requirements-sdk-rollback.txt, together with restoration of the saved V2 driver
and helper copies. Do not combine a restored V2 SDK with a V3-only driver import.
Exact pre-change node/helper copies are included in rollback/ for that purpose.

Physical limits remain
----------------------
Offline tests cannot establish arm/CAN health or target acceptance by firmware.
If periodic joint/status feedback is absent, do not enable or switch modes.
The independent helper still needs supported arms, no competing control writer,
and fresh fault-free feedback. SDK mode acknowledgment does not prove acceptance
of targets sent while disabled; the helper reseeds immediately after enable as
well. Ctrl+C does not disable motors.

Primary references
------------------
https://github.com/agilexrobotics/piper_sdk/tree/b847e5393a7ce6ff8508d713eca7bdd3a42421b4
https://github.com/agilexrobotics/piper_sdk/blob/b847e5393a7ce6ff8508d713eca7bdd3a42421b4/README.MD
https://github.com/agilexrobotics/piper_sdk/blob/b847e5393a7ce6ff8508d713eca7bdd3a42421b4/assets/V3/INTERFACE_V3.MD
https://pypi.org/project/piper-sdk/0.7.0a1/
