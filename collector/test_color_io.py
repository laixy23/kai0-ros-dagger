"""Real cv_bridge/OpenCV/PyAV color boundary check; never starts a ROS node.

Run under the ROS environment with cv_bridge, sensor_msgs, numpy, cv2, av, h5py.
Only source AST methods are executed: no collector module import or CAN sockets.
"""
import ast
from collections import deque
from contextlib import redirect_stdout
import importlib.util
import io
import logging
import os
from pathlib import Path
import tempfile
import time
from types import SimpleNamespace
import unittest


ROOT = Path(__file__).resolve().parent
MODULES = ["numpy", "cv2", "av", "h5py", "cv_bridge", "sensor_msgs"]
READY = all(importlib.util.find_spec(name) is not None for name in MODULES)


def source_function(path, name):
    return next(node for node in ast.parse(path.read_text()).body if isinstance(node, ast.FunctionDef) and node.name == name)


def source_method(path, class_name, method_name):
    cls = next(node for node in ast.parse(path.read_text()).body if isinstance(node, ast.ClassDef) and node.name == class_name)
    return next(node for node in cls.body if isinstance(node, ast.FunctionDef) and node.name == method_name)


def execute_nodes(nodes, namespace, path):
    exec(compile(ast.fix_missing_locations(ast.Module(body=nodes, type_ignores=[])), str(path), "exec"), namespace)


@unittest.skipUnless(READY, "requires the full ROS/cv_bridge/OpenCV/PyAV/h5py environment")
class RealColorBoundaryTests(unittest.TestCase):
    def test_rgb_and_bgr_ros_messages_policy_payload_and_saved_mp4(self):
        import av
        import cv2
        import h5py
        import numpy as np
        from cv_bridge import CvBridge
        from genpy import Time

        rgb = np.zeros((64, 64, 3), dtype=np.uint8)
        rgb[:32, :32] = [255, 0, 0]
        rgb[:32, 32:] = [0, 255, 0]
        rgb[32:, :32] = [0, 0, 255]
        rgb[32:, 32:] = [240, 160, 40]
        bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        camera_names = ["cam_high", "cam_left_wrist", "cam_right_wrist"]
        bridge = CvBridge()
        collector_path = ROOT / "agilex_openpi_dagger_collect.py"
        writer_path = ROOT / "collect_data.py"
        namespace = dict(
            np=np, cv2=cv2, av=av, h5py=h5py, Path=Path, os=os, logging=logging, time=time,
            rospy=SimpleNamespace(is_shutdown=lambda: False), lang_embeddings="color sentinel",
            image_tools=SimpleNamespace(resize_with_pad=lambda images, width, height: images),
        )
        execute_nodes([source_method(collector_path, "RosOperator", "get_frame")], namespace, collector_path)
        main_get_frame = namespace["get_frame"]
        execute_nodes([source_method(writer_path, "CollectOperator", "get_frame")], namespace, writer_path)
        standalone_get_frame = namespace["get_frame"]
        execute_nodes([source_function(collector_path, "inference_fn")], namespace, collector_path)
        execute_nodes([source_function(writer_path, name) for name in ["encode_video_frames", "create_video_from_images", "save_data"]], namespace, writer_path)

        with tempfile.TemporaryDirectory() as output_dir, redirect_stdout(io.StringIO()):
            for episode, encoding in enumerate(["rgb8", "bgr8"]):
                with self.subTest(encoding=encoding):
                    image = bridge.cv2_to_imgmsg(rgb if encoding == "rgb8" else bgr, encoding=encoding)
                    image.header.stamp = Time(1)
                    depth_array = np.full((64, 64), 1234, dtype=np.uint16)
                    depth_image = bridge.cv2_to_imgmsg(depth_array, encoding="16UC1")
                    depth_image.header.stamp = Time(1)
                    joint = SimpleNamespace(header=image.header)
                    def make_operator():
                        operator = SimpleNamespace(args=SimpleNamespace(use_depth_image=True, use_robot_base=False), bridge=bridge)
                        for camera in ["left", "right", "front"]:
                            setattr(operator, f"img_{camera}_deque", deque([image]))
                            setattr(operator, f"img_{camera}_depth_deque", deque([depth_image]))
                        for role in ["puppet", "master"]:
                            for side in ["left", "right"]:
                                setattr(operator, f"{role}_arm_{side}_deque", deque([joint]))
                        return operator
                    frames = main_get_frame(make_operator())
                    standalone = standalone_get_frame(make_operator())
                    for frame in list(frames[:3]) + list(standalone[:3]):
                        np.testing.assert_array_equal(frame, bgr)
                    for frame in frames[3:6]:
                        np.testing.assert_array_equal(frame, depth_array)
                    payloads = []
                    def infer(payload):
                        payloads.append(payload)
                        return {"actions": np.zeros((1, 14))}
                    namespace["observation_window"] = [{"qpos": np.zeros(14), "images": dict(zip(camera_names, frames[:3]))}]
                    namespace["inference_fn"](None, {"camera_names": camera_names}, SimpleNamespace(infer=infer))
                    self.assertEqual(len(payloads), 1)
                    for image_key in ["top_head", "hand_left", "hand_right"]:
                        np.testing.assert_array_equal(payloads[0]["images"][image_key].transpose(1, 2, 0), rgb)

                    obs = dict(qpos=np.zeros(14), qvel=np.zeros(14), effort=np.zeros(14), base_vel=[0.0, 0.0], images=dict(zip(camera_names, frames[:3])))
                    args = SimpleNamespace(camera_names=camera_names, export_video=True, video_fps=10, video_codec="libx264", video_quality=15)
                    namespace["save_data"](args, [SimpleNamespace(observation=obs)], [np.zeros(14)], str(Path(output_dir) / f"episode_{episode}"))
                    with h5py.File(Path(output_dir) / f"episode_{episode}.hdf5", "r") as data:
                        self.assertEqual(data["action"].shape, (1, 14))
                    for camera in camera_names:
                        path = Path(output_dir) / "video" / camera / f"episode_{episode}.mp4"
                        self.assertTrue(path.is_file(), path)
                        with av.open(str(path)) as container:
                            decoded = next(container.decode(video=0)).to_ndarray(format="rgb24")
                        for y, x in [(16, 16), (16, 48), (48, 16), (48, 48)]:
                            np.testing.assert_allclose(decoded[y, x].astype(int), rgb[y, x].astype(int), atol=12)


if __name__ == "__main__":
    unittest.main()
