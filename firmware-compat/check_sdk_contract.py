#!/usr/bin/env python3
"""Offline V3 SDK/node contract audit. No interface instance or hardware access.

Only DTOs, a parser, and fake adapter state are constructed. A process audit hook
rejects socket creation before importing the SDK. The ROS node is parsed as AST.
"""
from __future__ import annotations

import argparse
import ast
import hashlib
import importlib.metadata
import inspect
import json
from pathlib import Path
import struct
import sys
import threading
import time
from types import SimpleNamespace


PINNED_COMMIT = "b847e5393a7ce6ff8508d713eca7bdd3a42421b4"
EXPECTED_VERSION = "0.7.0a1"
EXPECTED_PYTHON_FILES = 195
EXPECTED_SOURCE_SHA256 = "141220166c26525b24b93c05f08c31455e99eda7ca01f608288703104e939e10"
HERE = Path(__file__).resolve().parent
DEFAULT_NODE = HERE / "piper_start_ms_node_new.py"
WRAPPERS = {
    "GetArmStatus": "ArmStatus",
    "GetArmEndPoseMsgs": "ArmEndPose",
    "GetArmJointMsgs": "ArmJoint",
    "GetArmJointCtrl": "ArmJointCtrl",
    "GetArmHighSpdInfoMsgs": "ArmMotorDriverInfoHighSpd",
    "GetArmLowSpdInfoMsgs": "ArmMotorDriverInfoLowSpd",
    "GetArmGripperMsgs": "ArmGripper",
    "GetArmGripperCtrl": "ArmGripperCtrl",
}


def require(condition, message):
    if not condition:
        raise AssertionError(message)


def sdk_call(node):
    return (isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute)
            and isinstance(node.func.value, ast.Attribute)
            and isinstance(node.func.value.value, ast.Name)
            and node.func.value.value.id == "self" and node.func.value.attr == "piper")


def dto_path(node):
    parts = []
    while isinstance(node, ast.Attribute):
        parts.append(node.attr)
        node = node.value
    if sdk_call(node):
        return node.func.attr, ".".join(reversed(parts))
    return None


def node_contract(source):
    calls, paths = {}, {}
    for node in ast.walk(ast.parse(source)):
        if sdk_call(node):
            calls.setdefault(node.func.attr, []).append(node)
        elif isinstance(node, ast.Attribute):
            result = dto_path(node)
            if result:
                paths.setdefault(result[0], set()).add(result[1])
    leaves = {method: sorted(path for path in group
                            if not any(other.startswith(path + ".") for other in group))
              for method, group in paths.items()}
    return calls, leaves


def source_fingerprint(root):
    rows = [(path.relative_to(root).as_posix(), hashlib.sha256(path.read_bytes()).hexdigest())
            for path in sorted(root.rglob("*.py"))]
    encoded = "".join(name + "\0" + sha + "\n" for name, sha in rows).encode()
    return len(rows), hashlib.sha256(encoded).hexdigest()


def check_protocol(sdk, interface, parser, Message, messages):
    # Actual V3 codec, pure can.Message DTOs; no transport object or sender.
    # For the two config-only SDK wrappers, provide only an in-memory send
    # callback. No interface, can.Bus, socket, or real transport is constructed.
    config_frames = []
    success = object()
    def record_frame(identifier, data):
        config_frames.append((identifier, bytes(data)))
        return success
    fake_config = SimpleNamespace(
        ArmMsgType=messages.ArmMsgType,
        tx_msg=messages.PiperMessage(),
        _parser=parser,
        _arm_can=SimpleNamespace(SendCanMessage=record_frame,
                                 CAN_STATUS=SimpleNamespace(SEND_MESSAGE_SUCCESS=success)),
        logger=SimpleNamespace(error=lambda *args: None),
    )
    config_checks = {}
    for name, args, identifier, expected in (
            ("DisableArm", (7,), 0x471, bytes([7, 1, 0, 0, 0, 0, 0, 0])),
            ("MasterSlaveConfig", (0xFC, 0, 0, 0), 0x470, bytes([0xFC, 0, 0, 0, 0, 0, 0, 0]))):
        before = len(config_frames)
        getattr(interface, name)(fake_config, *args)
        require(len(config_frames) == before + 1, name + " must request exactly one frame")
        require(config_frames[-1] == (identifier, expected), name + " wire contract changed")
        config_checks[name] = {"can_id": f"0x{identifier:03X}", "payload_hex": expected.hex(),
                               "send_callbacks": 1, "automatic_enable_joint_or_gripper_calls": 0}

    modes = {}
    for mode in (0, 1):
        message, frame = messages.PiperMessage(), Message()
        message.type_ = messages.ArmMsgType.PiperMsgMotionCtrl_2
        message.arm_motion_ctrl_2 = messages.ArmMsgMotionCtrl_2(1, mode, 10, 0, 0, 0)
        require(parser.EncodeMessage(message, frame), "V3 MOVE P/J encoding failed")
        require(frame.arbitration_id == 0x151 and bytes(frame.data) == bytes([1, mode, 10, 0, 0, 0, 0, 0]),
                "MOVE P/J normal-mode wire contract changed")
        modes["MOVE_P" if mode == 0 else "MOVE_J"] = bytes(frame.data).hex()

    dispatches = []
    fake_sender = SimpleNamespace(GripperCtrl_V3=lambda *args: dispatches.append(args))
    for code in (0, 1, 2):
        interface.GripperCtrl(fake_sender, 17000, 800, code, 0)
        require(dispatches[-1] == (17000, 800, code, 0), "Legacy gripper did not dispatch width mode")
    try:
        interface.GripperCtrl(fake_sender, 17000, 800, 4, 0)
    except ValueError:
        pass
    else:
        raise AssertionError("Legacy GripperCtrl accepted an angle-mode code")

    fake_state = SimpleNamespace(
        _arm_gripper_msgs=interface.ArmGripper(),
        _arm_gripper_msgs_mtx=threading.Lock(),
        _arm_gripper_ctrl_msgs=interface.ArmGripperCtrl(),
        _arm_gripper_ctrl_msgs_mtx=threading.Lock(),
        ArmMsgType=messages.ArmMsgType,
        isFilterAbnormalData=lambda: False,
        _CalGripperSDKLimit=lambda value, mode: value,
    )
    observed = {}
    for mode, value in ((0, 17000), (1, 90000)):
        decoded = messages.PiperMessage()
        frame = Message(arbitration_id=0x2A8, data=struct.pack(">ihBB", value, 800, 0x40, mode),
                        timestamp=1.0, is_extended_id=False)
        require(parser.DecodeMessage(frame, decoded), "V3 gripper feedback decoding failed")
        require(int(decoded.gripper_feedback_v3.mode) == mode, "V3 gripper mode not preserved")
        wrapper = interface._UpdateArmGripperState(fake_state, decoded)
        require(wrapper.gripper_state.grippers_angle == value and wrapper.gripper_state.grippers_effort == 800,
                "V3 legacy gripper feedback DTO contract changed")
        observed[str(mode)] = wrapper.gripper_state.grippers_angle

    decoded = messages.PiperMessage()
    frame = Message(arbitration_id=0x159, data=struct.pack(">ihBB", 17000, 800, 1, 0),
                    timestamp=1.0, is_extended_id=False)
    require(parser.DecodeMessage(frame, decoded), "V3 leader gripper control decoding failed")
    wrapper = interface._UpdateArmGripperCtrl(fake_state, decoded)
    require(wrapper.gripper_ctrl.grippers_angle == 17000, "V3 legacy leader gripper DTO changed")
    return {"config_only_sdk_methods": config_checks,
            "normal_move_frames": modes, "legacy_width_control_codes": [0, 1, 2],
            "legacy_feedback_raw_values_by_mode": observed,
            "required_live_gripper_feedback_mode": 0,
            "live_gripper_mode_byte": "0x2A8 byte7",
            "angle_mode_values_are_not_meters": True,
            "interface_instances_created": 0, "transport_instances_created": 0}


def run(node_path):
    blocked = []
    def audit(event, args):
        if event == "socket.__new__":
            blocked.append(event)
            raise RuntimeError("Offline SDK contract audit forbids socket creation")
    sys.addaudithook(audit)
    report = {"passed": False, "timestamp_sec": time.time(), "pinned_commit": PINNED_COMMIT,
              "hardware_control_writes": 0, "interface_instances_created": 0,
              "transport_instances_created": 0, "socket_audit_guard_installed": True}
    try:
        import piper_sdk
        from piper_sdk import C_PiperInterface_V3, C_PiperParserV3
        from piper_sdk.piper_msgs import msg_v3
        from can import Message  # A pure message DTO, never can.Bus.

        version = importlib.metadata.version("piper_sdk")
        report["sdk_version"] = version
        require(version == EXPECTED_VERSION, "Expected pinned SDK version " + EXPECTED_VERSION)
        package_root = Path(piper_sdk.__file__).resolve().parent
        count, digest = source_fingerprint(package_root)
        report["sdk_package_root"] = str(package_root)
        report["sdk_python_source"] = {"files": count, "sha256": digest}
        require(count == EXPECTED_PYTHON_FILES and digest == EXPECTED_SOURCE_SHA256,
                "Installed SDK Python sources do not match the pinned primary repository")
        node_path = node_path.resolve(strict=True)
        source = node_path.read_text()
        report["node"] = {"path": str(node_path), "sha256": hashlib.sha256(source.encode()).hexdigest()}
        calls, paths = node_contract(source)
        require(len(calls) == 18, "Expected exactly 18 public SDK methods in the official node")
        require(sum(len(group) for group in paths.values()) == 52, "Expected exactly 52 consumed DTO fields")
        require(set(paths) == set(WRAPPERS), "Unexpected SDK getter contract")
        signatures, bindings = {}, 0
        for name, variants in sorted(calls.items()):
            signature = inspect.signature(getattr(C_PiperInterface_V3, name))
            for call in variants:
                require(not any(isinstance(arg, ast.Starred) for arg in call.args), "Unsupported starred call")
                require(not any(keyword.arg is None for keyword in call.keywords), "Unsupported keyword expansion")
                signature.bind(None, *[object() for _ in call.args],
                               **{keyword.arg: object() for keyword in call.keywords})
                bindings += 1
            signatures[name] = str(signature)
        dto_fields = {}
        for getter, group in sorted(paths.items()):
            wrapper = getattr(C_PiperInterface_V3, WRAPPERS[getter])()
            for path in group:
                value = wrapper
                for part in path.split("."):
                    value = getattr(value, part)
                dto_fields[getter + "()." + path] = type(value).__name__
        report.update(method_count=len(calls), control_lifecycle_methods=10, getter_methods=8,
                      call_variants_bound=bindings, method_signatures=signatures,
                      dto_field_count=len(dto_fields), dto_fields=dto_fields)
        report["protocol_checks"] = check_protocol(piper_sdk, C_PiperInterface_V3,
                                                     C_PiperParserV3(), Message, msg_v3)
        require(not blocked, "SDK import or DTO/parser checks attempted socket creation")
        report["passed"] = True
    except Exception as exc:
        report["error"] = repr(exc)
    report["blocked_socket_creation_attempts"] = len(blocked)
    report["socket_creations_allowed"] = 0
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--node", type=Path, default=DEFAULT_NODE)
    parser.add_argument("--output", type=Path, default=HERE / "contract-audit.json")
    args = parser.parse_args()
    report = run(args.node)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps({"passed": report["passed"], "sdk_version": report.get("sdk_version"),
                      "methods": report.get("method_count"), "dto_fields": report.get("dto_field_count"),
                      "hardware_control_writes": 0, "report": str(args.output),
                      "error": report.get("error")}, indent=2))
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
