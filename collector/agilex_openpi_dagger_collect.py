      
# -- coding: UTF-8
"""
#!/usr/bin/python3
"""

import argparse
import threading
import time
from collections import deque
import sys
import termios
import tty
import select
import math
from pathlib import Path

import cv2
import numpy as np
import rospy
import torch
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from openpi_client import image_tools, websocket_client_policy
from piper_msgs.msg import PosCmd
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import Header, Bool, String, Int32

import os
import h5py
import dm_env
import collections
from collect_data import save_data, create_video_from_images, CollectOperator

_alignment_setup = Path(os.environ.get("DAGGER_ROOT", Path(__file__).resolve().parents[1])) / "setup"
sys.path.insert(0, str(_alignment_setup))
from master_alignment_feedback import alignment_targets, wait_for_master_alignment
from follower_readiness import check_followers_ready
from teaching_readiness import require_pair_ready

CAMERA_NAMES = ["cam_high", "cam_right_wrist", "cam_left_wrist"]

observation_window = None
# smoothing globals
stream_buffer = None
inference_generation = 0
inference_state_lock = threading.Lock()

# DAgger mode control
dagger_mode_active = False
resume_inference_requested = False
resume_inference_in_progress = False
dagger_mode_lock = threading.Lock()

save_data_requested = False
save_data_lock = threading.Lock()
collection_active = False
collection_lock = threading.Lock()

delete_data_requested = False
delete_data_lock = threading.Lock()
waiting_for_user_confirm = False
last_saved_episode_path = None
last_saved_episode_idx = None
last_saved_camera_names = None

lang_embeddings = "fold the cloth"

RIGHT_OFFSET = 0.003
timesteps, actions = [], []
process_thread = None
result_container = {}

class SimpleDAggerCollector:
    """DAgger collector using save_data from collect_data."""
    def __init__(self, camera_names, dataset_dir="./home/agilex/data", task_name=None):
        self.camera_names = camera_names
        self.dataset_dir = dataset_dir
        self.task_name = task_name
        self.is_collecting = False
        self.timesteps = []
        self.actions = []
        self.frame_count = 0
        self.episode_idx = 0
        self.full_dataset_dir = None
        
    def _find_next_episode_idx(self):
        """Find next available episode index."""
        if self.full_dataset_dir is None or not os.path.exists(self.full_dataset_dir):
            self.episode_idx = 0
            print(f"Next episode index: {self.episode_idx}")
            return
            
        existing_episodes = [
            
            f for f in os.listdir(self.full_dataset_dir) 
            if f.startswith('episode_') and f.endswith('.hdf5')
        ]
        if existing_episodes:
            indices = [int(f.split('_')[1].split('.')[0]) for f in existing_episodes]
            self.episode_idx = max(indices) + 1
        else:
            self.episode_idx = 0
        print(f"Next episode index: {self.episode_idx}")
    def start_collection(self):
        """Start collecting data."""
        self.is_collecting = True
        self.timesteps = []
        self.actions = []
        self.frame_count = 0
        print("\n" + "="*70)
        print("Data collection started.")
        print("="*70)
        print(f"Data will be saved to: {self.full_dataset_dir}")
        print(f"Current episode: {self.episode_idx}")
        print("Press 's' to save current data")
        print("="*70 + "\n")
    def stop_collection(self):
        """Stop collecting."""
        self.is_collecting = False
        if self.has_data():
            print(f"\nWarning: {len(self.actions)} frames not saved. Press 's' to save or ignore.")
        else:
            print("\nDAgger collection stopped.")
    def add_frame(self, observation, action):
        """Add one frame (same logic as collect_data)."""
        if not self.is_collecting:
            return
        
        self.frame_count += 1
        
        if self.frame_count == 1:
            timestep = dm_env.TimeStep(
                step_type=dm_env.StepType.FIRST,
                reward=None,
                discount=None,
                observation=observation
            )
            self.timesteps.append(timestep)
        else:
            timestep = dm_env.TimeStep(
                step_type=dm_env.StepType.MID,
                reward=None,
                discount=None,
                observation=observation
            )
            self.timesteps.append(timestep)
            self.actions.append(action)
        
        if self.frame_count % 50 == 0:
            print(f"Collected {self.frame_count} frames (actions: {len(self.actions)})")
    
    def save_current_episode(self, export_video=True, video_fps=30,
                            video_codec='libx264', video_quality=23):
        """Save current episode and end collection."""
        if len(self.actions) == 0:
            print("\033[31mNo data to save.\033[0m")
            return False
        dataset_path = os.path.join(self.full_dataset_dir, f"episode_{self.episode_idx}")
        print(f"\nSaving Episode {self.episode_idx}...")
        print(f"  Frames: {len(self.actions)}")
        class Args:
            def __init__(self, camera_names, export_video, video_fps, video_codec, video_quality):
                self.camera_names = camera_names
                self.export_video = export_video
                self.video_fps = video_fps
                self.video_codec = video_codec
                self.video_quality = video_quality
                self.use_robot_base = False
        
        args = Args(self.camera_names, export_video, video_fps, video_codec, video_quality)
        
        try:
            save_data(args, self.timesteps, self.actions, dataset_path)
            print(f"\n\033[32mEpisode {self.episode_idx} saved.\033[0m")
            global waiting_for_user_confirm, last_saved_episode_path, last_saved_episode_idx, last_saved_camera_names
            global delete_data_requested, delete_data_lock
            with delete_data_lock:
                waiting_for_user_confirm = True
                last_saved_episode_path = dataset_path
                last_saved_episode_idx = self.episode_idx
                last_saved_camera_names = self.camera_names
            print("\n" + "="*70)
            print("Is this capture correct?")
            print("  If not, press 'w' to delete this save.")
            print("  If yes, press any other key to continue.")
            print("="*70)
            print("Waiting for input (10s timeout)...")
            import time
            max_wait_time = 10.0
            wait_start = time.time()
            should_delete = False
            
            while time.time() - wait_start < max_wait_time:
                with delete_data_lock:
                    if delete_data_requested:
                        should_delete = True
                        delete_data_requested = False
                        break
                    if not waiting_for_user_confirm:
                        break
                time.sleep(0.1)
            with delete_data_lock:
                waiting_for_user_confirm = False
            if should_delete:
                hdf5_file = dataset_path + '.hdf5'
                if os.path.exists(hdf5_file):
                    try:
                        os.remove(hdf5_file)
                        print(f"\n\033[33mDeleted: {hdf5_file}\033[0m")
                    except Exception as e:
                        print(f"\n\033[31mFailed to delete hdf5: {e}\033[0m")
                video_dir = os.path.join(os.path.dirname(dataset_path), "video")
                if os.path.exists(video_dir):
                    episode_idx_str = str(self.episode_idx)
                    for cam_name in self.camera_names:
                        cam_video_dir = os.path.join(video_dir, cam_name)
                        video_file = os.path.join(cam_video_dir, f"episode_{episode_idx_str}.mp4")
                        if os.path.exists(video_file):
                            try:
                                os.remove(video_file)
                                print(f"\n\033[33mDeleted video: {video_file}\033[0m")
                            except Exception as e:
                                print(f"\n\033[31mFailed to delete video: {e}\033[0m")
                    try:
                        for cam_name in self.camera_names:
                            cam_video_dir = os.path.join(video_dir, cam_name)
                            if os.path.exists(cam_video_dir) and not os.listdir(cam_video_dir):
                                os.rmdir(cam_video_dir)
                    except Exception:
                        pass
                print("\n\033[32mData deleted.\033[0m")
                self.timesteps = []
                self.actions = []
                self.frame_count = 0
                self.is_collecting = False
            else:
                # 数据正确，继续正常流程
                # 🆕 保存后停止采集，清空数据
                self.episode_idx += 1
                self.timesteps = []
                self.actions = []
                self.frame_count = 0
                self.is_collecting = False
            
            print("\n" + "="*70)
            print("Save done. Collection ended.")
            print("Follower forwarding stays paused while this recording is stopped.")
            print("Press 'r' to exit DAgger and resume inference.")
            print("Or press Space / the left pedal to resume following and start a new recording.")
            print("="*70 + "\n")
            return True
        except Exception as e:
            print(f"\033[31mSave failed: {e}\033[0m")
            import traceback
            traceback.print_exc()
            return False
    
    def get_frame_count(self):
        return self.frame_count
    
    def has_data(self):
        return len(self.actions) > 0


def handle_operator_action(char, source="keyboard", ros_operator=None):
    """Dispatch keyboard and private pedal actions through the same controls."""
    global dagger_mode_active, resume_inference_requested, save_data_requested, collection_active
    global delete_data_requested, waiting_for_user_confirm, resume_inference_in_progress
    if source == 'pedal':
        if char not in ('left', 'save', 'resume', 'delete'):
            return
        pedal = char
        char = {'left': 'd', 'save': 's', 'resume': 'r', 'delete': 'w'}[pedal]
        with delete_data_lock:
            if waiting_for_user_confirm:
                waiting_for_user_confirm = False
                if pedal == 'delete':
                    delete_data_requested = True
                    print("\nDelete requested. Deleting data...\n")
                else:
                    print("\nData confirmed.\n")
                return
        if pedal == 'delete':
            return
        if pedal == 'left' and dagger_mode_active:
            if collection_active:
                print("\nDAgger recording is already requested.\n")
                return
            if ros_operator is None or not ros_operator.dagger_recording_status()[0]:
                print("\nTeaching mode is not confirmed; pedal did not request a new recording.\n")
                return
            char = ' '
    if not char:
        return
    with delete_data_lock:
        is_waiting_confirm = waiting_for_user_confirm
    if is_waiting_confirm:
        if char.lower() == 'w':
            with delete_data_lock:
                delete_data_requested = True
                waiting_for_user_confirm = False
                print("\nDelete requested. Deleting data...\n")
        else:
            with delete_data_lock:
                waiting_for_user_confirm = False
            print("\nData confirmed.\n")
        return
    
    # Check for Ctrl+Q (ASCII 17)
    if ord(char) == 17:
        with dagger_mode_lock:
            if not dagger_mode_active:
                dagger_mode_active = True
                print("\n" + "🎯"*35)
                print("🎯 DAgger mode ACTIVATED! Pausing inference...")
                print("🎯"*35 + "\n")
            else:
                print("\n⚠️ DAgger mode already active\n")
    
    # 'd' key to activate DAgger mode
    elif char.lower() == 'd':
        with dagger_mode_lock:
            if not dagger_mode_active:
                dagger_mode_active = True
                print("\n" + "🎯"*35)
                print("🎯 DAgger mode ACTIVATED! Pausing inference...")
                print("🎯"*35 + "\n")
            else:
                print("\n⚠️ DAgger mode already active\n")
    
    # 'r' key to resume inference mode
    elif char.lower() == 'r':
        # Do not queue an old key behind a long mode transition and
        # accidentally turn it into a retry after that attempt ends.
        if not dagger_mode_lock.acquire(False):
            return
        try:
            if dagger_mode_active and not resume_inference_requested and not resume_inference_in_progress:
                resume_inference_requested = True
                print("\n" + "▶️"*35)
                print("▶️ Policy resume requested; waiting for master handoff...")
                print("▶️"*35 + "\n")
            elif not dagger_mode_active:
                print("\n⚠️ Already in inference mode\n")
        finally:
            dagger_mode_lock.release()
    
    elif char.lower() == 's':
        with dagger_mode_lock:
            if dagger_mode_active:
                with save_data_lock:
                    save_data_requested = True
                print("\n" + "💾"*35)
                print("💾 Save requested - will save at next opportunity...")
                print("💾"*35 + "\n")
            else:
                print("\n⚠️ Not in DAgger mode, cannot save data\n")
    
    elif char == ' ':
        with dagger_mode_lock:
            if dagger_mode_active:
                if resume_inference_requested or resume_inference_in_progress:
                    print("\nMaster handoff is in progress; current correction data is kept.\n")
                    return
                with collection_lock:
                    if not collection_active:
                        collection_active = True
                        print("\nRecording requested; waiting for teaching status if needed.\n")
                    else:
                        print("\nRecording is already requested; current data is kept.\n")
            else:
                print("\nEnter DAgger mode first (press 'd').\n")


def _pedal_pipe_from_environment(name):
    """Adopt only an explicitly inherited private pipe, never stdin/stdout."""
    import stat
    value = os.environ.get(name)
    if value is None:
        return None
    try:
        descriptor = int(value)
        if descriptor < 3 or not stat.S_ISFIFO(os.fstat(descriptor).st_mode):
            raise ValueError('expected a private inherited pipe')
        os.set_inheritable(descriptor, False)
        return descriptor
    except (ValueError, OSError):
        print("\nPedal pipe is unavailable; keyboard controls remain available.\n")
        return None


def keyboard_monitor_thread(ros_operator=None):
    """Select keyboard input and optional private newline pedal commands."""
    old_settings = termios.tcgetattr(sys.stdin)
    command_fd = _pedal_pipe_from_environment('DAGGER_PEDAL_COMMAND_FD')
    ready_fd = _pedal_pipe_from_environment('DAGGER_PEDAL_READY_FD')
    owned_fds = {fd for fd in (command_fd, ready_fd) if fd is not None}
    pending = b''
    tokens = {b'left': 'left', b'save': 'save', b'resume': 'resume', b'delete': 'delete'}
    try:
        tty.setcbreak(sys.stdin.fileno())
        # Commands queued before keyboard readiness must not execute later.
        if command_fd is not None:
            while select.select([command_fd], [], [], 0)[0]:
                if not os.read(command_fd, 4096):
                    os.close(command_fd)
                    owned_fds.discard(command_fd)
                    command_fd = None
                    break
        if ready_fd is not None:
            try:
                if command_fd is not None:
                    os.write(ready_fd, b'ready\n')
            except OSError:
                pass
            finally:
                os.close(ready_fd)
                owned_fds.discard(ready_fd)
        while not rospy.is_shutdown():
            inputs = [sys.stdin] + ([command_fd] if command_fd is not None else [])
            for ready in select.select(inputs, [], [], 0.1)[0]:
                if command_fd is not None and ready == command_fd:
                    try:
                        data = os.read(command_fd, 4096)
                    except OSError:
                        data = b''
                    if not data:
                        os.close(command_fd)
                        owned_fds.discard(command_fd)
                        command_fd = None
                        pending = b''
                        continue
                    pending += data
                    while b'\n' in pending:
                        line, pending = pending.split(b'\n', 1)
                        action = tokens.get(line)
                        if action is not None:
                            handle_operator_action(action, source='pedal', ros_operator=ros_operator)
                    if len(pending) > 4096:
                        os.close(command_fd)
                        owned_fds.discard(command_fd)
                        command_fd = None
                        pending = b''
                else:
                    handle_operator_action(sys.stdin.read(1), source='keyboard', ros_operator=ros_operator)
    finally:
        for descriptor in owned_fds:
            os.close(descriptor)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)


class StreamActionBuffer:
    """
    Maintains action chunk queue and smoothed execution sequence.
    integrate_new_chunk supports latency trim and linear overlap blending (same as smooth).
    """
    def __init__(self, max_chunks=10, decay_alpha=0.25, state_dim=14, smooth_method="temporal"):
        self.chunks = deque()
        self.max_chunks = max_chunks
        self.lock = threading.Lock()
        self.decay_alpha = float(decay_alpha)
        self.state_dim = state_dim
        self.smooth_method = smooth_method
        self.cur_chunk = deque()
        self.k = 0
        self.last_action = None

    def integrate_new_chunk(self, actions_chunk: np.ndarray, max_k: int, min_m: int = 8):
        with self.lock:
            if actions_chunk is None or len(actions_chunk) == 0:
                return
            max_k = max(0, int(max_k))
            min_m = max(1, int(min_m))
            drop_n = min(self.k, max_k)
            if drop_n >= len(actions_chunk):
                return
            new_chunk = [a.copy() for a in actions_chunk[drop_n:]]
            if len(self.cur_chunk) == 0 and self.last_action is not None:
                old_list = [np.asarray(self.last_action, dtype=float).copy() for _ in range(min_m)]
                self.last_action = None
            else:
                old_list = list(self.cur_chunk)
                if len(old_list) > 0 and len(old_list) < min_m:
                    tail = np.asarray(old_list[-1], dtype=float).copy()
                    old_list.extend([tail.copy() for _ in range(min_m - len(old_list))])
                elif len(old_list) == 0:
                    self.cur_chunk = deque(new_chunk, maxlen=None)
                    self.k = 0
                    return
            new_list = list(new_chunk)
            overlap_len = min(len(old_list), len(new_list))
            if overlap_len <= 0:
                self.cur_chunk = deque(new_list, maxlen=None)
                self.k = 0
                return
            if len(old_list) > len(new_list):
                old_list = old_list[:len(new_list)]
                overlap_len = len(new_list)
            if overlap_len == 1:
                w_old = np.array([1.0], dtype=float)
            else:
                w_old = np.linspace(1.0, 0.0, overlap_len, dtype=float)
            w_new = 1.0 - w_old
            smoothed = [
                (w_old[i] * np.asarray(old_list[i], dtype=float) +
                 w_new[i] * np.asarray(new_list[i], dtype=float))
                for i in range(overlap_len)
            ]
            combined = smoothed + new_list[overlap_len:]
            self.cur_chunk = deque([a.copy() for a in combined], maxlen=None)
            self.k = 0

    def pop_next_action(self) -> np.ndarray | None:
        with self.lock:
            if len(self.cur_chunk) == 0:
                return None
            if len(self.cur_chunk) == 1:
                self.last_action = np.asarray(self.cur_chunk[0], dtype=float).copy()
            act = np.asarray(self.cur_chunk.popleft(), dtype=float)
            self.k += 1
            return act

def reset_inference_after_intervention():
    """Discard queued/inflight policy actions from before the correction."""
    global inference_generation, observation_window
    with inference_state_lock:
        inference_generation += 1
        observation_window = None
        if stream_buffer is not None:
            with stream_buffer.lock:
                stream_buffer.cur_chunk.clear()
                stream_buffer.chunks.clear()
                stream_buffer.last_action = None
                stream_buffer.k = 0


def start_inference_thread(args, config, policy, ros_operator):
    th = threading.Thread(target=inference_fn_non_blocking_fast, args=(args, config, policy, ros_operator))
    th.daemon = True
    th.start()

def inference_fn_non_blocking_fast(args, config, policy, ros_operator):
    global stream_buffer
    rate = rospy.Rate(getattr(args, "inference_rate", 4))
    while not rospy.is_shutdown():
        try:
            # Pause inference in DAgger mode
            if dagger_mode_active:
                try:
                    rate.sleep()
                except Exception:
                    time.sleep(0.01)
                continue
            # Update observation
            generation = inference_generation
            update_observation_window(args, config, ros_operator)
            latest_obs = observation_window[-1]
            imgs = [
                latest_obs["images"][config["camera_names"][0]],
                latest_obs["images"][config["camera_names"][1]],
                latest_obs["images"][config["camera_names"][2]],
            ]
            imgs = [cv2.cvtColor(im, cv2.COLOR_BGR2RGB) for im in imgs]
            imgs = image_tools.resize_with_pad(np.array(imgs), 224, 224)
            proprio = latest_obs["qpos"]
            payload = {
                "state": proprio,
                "images": {
                    "top_head": imgs[0].transpose(2, 0, 1),
                    "hand_right": imgs[1].transpose(2, 0, 1),
                    "hand_left": imgs[2].transpose(2, 0, 1),
                },
                "prompt": lang_embeddings,
            }
            actions = policy.infer(payload)["actions"]
            if actions is not None and len(actions) > 0:
                max_k = int(getattr(args, "latency_k", 0))
                min_m = int(getattr(args, "min_smooth_steps", 8))
                with inference_state_lock:
                    if not dagger_mode_active and generation == inference_generation:
                        stream_buffer.integrate_new_chunk(actions, max_k=max_k, min_m=min_m)
            try:
                rate.sleep()
            except Exception:
                pass
        except Exception as e:
            try:
                rate.sleep()
            except Exception:
                time.sleep(0.005)
            continue


def set_seed(seed):
    torch.manual_seed(seed)
    np.random.seed(seed)


class SimpleKalmanFilter:
    def __init__(self, process_variance=1e-6, measurement_variance=1e-7, initial_value=None):
        self.process_variance = process_variance
        self.measurement_variance = measurement_variance
        self.estimate = initial_value
        self.error_estimate = 1.0

    def update(self, measurement):
        if self.estimate is None:
            self.estimate = measurement.copy()
            return self.estimate
        # Compute Kalman gain
        kalman_gain = self.error_estimate / (self.error_estimate + self.measurement_variance)
        # Update estimate
        self.estimate = self.estimate + kalman_gain * (measurement - self.estimate)
        # Update the error estimate
        self.error_estimate = (1 - kalman_gain) * self.error_estimate + abs(self.estimate - measurement) * self.process_variance
        return self.estimate


# Interpolate the actions to make the robot move smoothly
def interpolate_action(args, prev_action, cur_action):
    steps = np.concatenate((np.array(args.arm_steps_length), np.array(args.arm_steps_length)), axis=0)
    diff = np.abs(cur_action - prev_action)
    step = np.ceil(diff / steps).astype(int)
    step = np.max(step)
    if step <= 1:
        return cur_action[np.newaxis, :]
    new_actions = np.linspace(prev_action, cur_action, step + 1)
    return new_actions[1:]

def minimum_jerk_interpolation(args, prev_action, cur_action):
    num_steps = args.jerk_num_steps
    t_normalized = np.linspace(0, 1, num_steps + 1)[1:]
    trajectory = []
    for tau in t_normalized:
        factor = 10 * (tau ** 3) - 15 * (tau ** 4) + 6 * (tau ** 5)
        trajectory.append(prev_action + factor * (cur_action - prev_action))
    return np.array(trajectory)


def get_config(args):
    config = {
        "episode_len": args.max_publish_step,
        "state_dim": 14,
        "chunk_size": args.chunk_size,
        "camera_names": CAMERA_NAMES,
    }
    return config


# Get the observation from the ROS topic
def get_ros_observation(args, ros_operator):
    rate = rospy.Rate(args.publish_rate)
    print_flag = True

    # Max wait; use cached frame if timeout to avoid blocking
    max_wait_s = 0.6
    start_t = time.time()

    while True and not rospy.is_shutdown():
        # Non-destructive snapshot to avoid blocking
        result = ros_operator.get_frame_peek()
        if not result:
            # On timeout, try cached frame if any
            if (time.time() - start_t) > max_wait_s and ros_operator.last_frame_cache is not None:
                if print_flag:
                    print("syn timeout, using last cached frame in get_ros_observation")
                    print_flag = False
                (
                    img_front,
                    img_left,
                    img_right,
                    img_front_depth,
                    img_left_depth,
                    img_right_depth,
                    puppet_arm_left,
                    puppet_arm_right,
                    robot_base,
                ) = ros_operator.last_frame_cache
                return (img_front, img_left, img_right, puppet_arm_left, puppet_arm_right)
            if print_flag:
                print("syn fail when get_ros_observation")
                print_flag = False
            rate.sleep()
            continue
        print_flag = True
        (
            img_front,
            img_left,
            img_right,
            img_front_depth,
            img_left_depth,
            img_right_depth,
            puppet_arm_left,
            puppet_arm_right,
            robot_base,
        ) = result
        return (img_front, img_left, img_right, puppet_arm_left, puppet_arm_right)


# Update the observation window buffer
def update_observation_window(args, config, ros_operator):
    # JPEG transformation
    # Align with training
    def jpeg_mapping(img):
        img = cv2.imencode(".jpg", img)[1].tobytes()
        img = cv2.imdecode(np.frombuffer(img, np.uint8), cv2.IMREAD_COLOR)
        return img

    global observation_window
    if observation_window is None:
        observation_window = deque(maxlen=2)

        # Append the first dummy image
        observation_window.append(
            {
                "qpos": None,
                "images": {
                    config["camera_names"][0]: None,
                    config["camera_names"][1]: None,
                    config["camera_names"][2]: None,
                },
            }
        )

    img_front, img_left, img_right, puppet_arm_left, puppet_arm_right = get_ros_observation(args, ros_operator)
    img_front = jpeg_mapping(img_front)
    img_left = jpeg_mapping(img_left)
    img_right = jpeg_mapping(img_right)

    qpos = np.concatenate(
        (np.array(puppet_arm_left.position), np.array(puppet_arm_right.position)),
        axis=0,
    )

    observation_window.append(
        {
            "qpos": qpos,
            "images": {
                config["camera_names"][0]: img_front,
                config["camera_names"][1]: img_right,
                config["camera_names"][2]: img_left,
            },
        }
    )


def inference_fn(args, config, policy):
    global observation_window
    global lang_embeddings

    # print(f"Start inference_thread_fn: t={t}")
    while True and not rospy.is_shutdown():
        time1 = time.time()

        # fetch images in sequence [front, right, left]
        image_arrs = [
            observation_window[-1]["images"][config["camera_names"][0]],
            observation_window[-1]["images"][config["camera_names"][1]],
            observation_window[-1]["images"][config["camera_names"][2]],
        ]
        # convert bgr ro rgb
        image_arrs = [cv2.cvtColor(img, cv2.COLOR_BGR2RGB) for img in image_arrs]
        image_arrs = image_tools.resize_with_pad(np.array(image_arrs), 224, 224)

        # get last qpos in shape [14, ]
        proprio = observation_window[-1]["qpos"]

        payload = {
            "state": proprio,
            "images": {
                "top_head": image_arrs[0].transpose(2, 0, 1),
                "hand_right": image_arrs[1].transpose(2, 0, 1),
                "hand_left": image_arrs[2].transpose(2, 0, 1),
            },
            "prompt": lang_embeddings,
        }

        # actions shaped as [64, 14] in format [left, right]
        actions = policy.infer(payload)["actions"]

        print(f"Model inference time: {(time.time() - time1)*1000:.3f} ms")

        return actions


# Main loop for the manipulation task
def model_inference(args, config, ros_operator, keyboard_thread_started=False):
    global lang_embeddings
    global dagger_mode_active
    global resume_inference_requested
    global save_data_requested
    global collection_active
    global resume_inference_in_progress

    # Load client
    policy = websocket_client_policy.WebsocketClientPolicy(
        args.host,
        args.port,
    )
    kalman_filters = [SimpleKalmanFilter() for _ in range(config["state_dim"])]
    print(f"Server metadata: {policy.get_server_metadata()}")

    max_publish_step = config["episode_len"]
    chunk_size = config["chunk_size"]

    # Initialize position of the puppet arm
    left0 = [0, 0.32, -0.36, 0, 0.24, 0, 0.07]
    right0 = [0, 0.32, -0.36, 0, 0.24, 0, 0.07]
    ros_operator.wait_for_fresh_policy_inputs()
    ros_operator.puppet_arm_publish_continuous(left0, right0)
    # input("Press enter to continue")
    # ros_operator.puppet_arm_publish_continuous(left0, right0)
    
    # Start keyboard monitor thread AFTER input() to avoid terminal conflicts
    if not keyboard_thread_started:
        keyboard_thread = threading.Thread(target=keyboard_monitor_thread, args=(ros_operator,), daemon=True)
        keyboard_thread.start()
        print("\n" + "="*70)
        print("✅ Keyboard monitor thread started")
        print("🎹 DAgger Mode Controls:")
        print("   Press 'd' key → Activate DAgger mode (pause inference)")
        print("   Press 'r' key → Resume inference mode")
        print("="*70 + "\n")
        time.sleep(0.3)  # Give user time to read
    
    # Initialize the previous action to be the initial robot state
    pre_action = np.zeros(config["state_dim"])
    action = None
    
    # Track if we've entered DAgger mode
    dagger_mode_entered = False
    
    # Inference loop
    with torch.inference_mode():
        while True and not rospy.is_shutdown():
            # The current time step
            t = 0
            rate = rospy.Rate(args.publish_rate)

            action_buffer = np.zeros([chunk_size, config["state_dim"]])
            last_stream_act = None

            while t < max_publish_step and not rospy.is_shutdown():
                resume_now = False
                # Check if DAgger mode is activated
                with dagger_mode_lock:
                    if dagger_mode_active and not dagger_mode_entered:
                        print("\n" + "="*70)
                        print("⏸️  INFERENCE PAUSED - ENTERING DAGGER MODE")
                        print("="*70 + "\n")
                        if not ros_operator.enter_dagger_mode():
                            raise RuntimeError("Failed to enter DAgger mode; inference remains stopped")
                        with collection_lock:
                            ros_operator.data_collector.start_collection()
                            ros_operator.consume_dagger_preroll()
                            collection_active = True
                        dagger_mode_entered = True
                    elif resume_inference_requested and dagger_mode_entered:
                        resume_inference_requested = False
                        resume_inference_in_progress = True
                        resume_now = True

                # Keep the keyboard lock free while observing the FC handoff.
                # Repeated r keys are consumed as part of this one attempt.
                if resume_now:
                    try:
                        # Stop data collection
                        ros_operator.data_collector.stop_collection()
                        with collection_lock:
                            collection_active = False
                        ros_operator.exit_dagger_mode()
                    except Exception as exc:
                        print("\nMaster handoff is incomplete; DAgger and policy pause are kept.")
                        print(str(exc))
                        print("Current correction data is kept. Press 'r' to retry, 's' to save, or Ctrl+C to stop.\n")
                    else:
                        reset_inference_after_intervention()
                        last_stream_act = None
                        pre_action = None
                        t = 0
                        action_buffer = np.zeros([chunk_size, config["state_dim"]])
                        kalman_filters = [SimpleKalmanFilter() for _ in range(config["state_dim"])]
                        ros_operator.start_inference_collection()
                        with dagger_mode_lock:
                            dagger_mode_active = False
                            dagger_mode_entered = False
                        print("\n" + "="*70)
                        print("▶️  DAGGER MODE DEACTIVATED - RESUMING INFERENCE")
                        print("="*70 + "\n")
                    finally:
                        with dagger_mode_lock:
                            resume_inference_in_progress = False
                
                # Check save request
                with save_data_lock:
                    if save_data_requested and dagger_mode_active:
                        ros_operator.set_dagger_following(False)
                        ros_operator.data_collector.stop_collection()
                        with collection_lock:
                            collection_active = False
                        if ros_operator.data_collector.has_data():
                            ros_operator.data_collector.save_current_episode(
                                export_video=getattr(args, 'export_video', True),
                                video_fps=getattr(args, 'video_fps', 30),
                                video_codec=getattr(args, 'video_codec', 'libx264'),
                                video_quality=getattr(args, 'video_quality', 23)
                            )
                        else:
                            print("\nNo data to save.\n")
                        save_data_requested = False
                
                # Mode/enable confirmation controls data quality only. A quiet
                # native command stream is legal while the master is stationary.
                recording_ready = not dagger_mode_active or ros_operator.dagger_recording_ready()

                # A pending Space request must not repeatedly reset an episode
                # while mode confirmation is unavailable.
                with collection_lock:
                    if recording_ready and collection_active and not ros_operator.data_collector.is_collecting:
                        if ros_operator.data_collector.has_data():
                            collection_active = False
                            print("\nUnsaved correction data is kept. Press 's' before starting a new recording.\n")
                        else:
                            ros_operator.set_dagger_following(True)
                            ros_operator.data_collector.start_collection()
    
                # If in DAgger mode, skip inference and collect demonstration data
                if dagger_mode_active:
                    if not recording_ready:
                        rate.sleep()
                        continue
                    # Collect using time-synced get_synced_frame
                    synced_data = ros_operator.get_synced_frame()
                    
                    if synced_data is not None:
                        try:
                            observation, action = ros_operator._build_dagger_sample(synced_data)
                            ros_operator.data_collector.add_frame(observation, action)
                        except Exception as e:
                            print(f"Collection error: {e}")
                    
                    rate.sleep()
                    continue
                
                # Temporal smoothing execution path
                if args.use_temporal_smoothing:
                    global stream_buffer
                    if stream_buffer is None:
                        stream_buffer = StreamActionBuffer(
                            max_chunks=args.buffer_max_chunks,
                            decay_alpha=args.exp_decay_alpha,
                            state_dim=config["state_dim"],
                            smooth_method="temporal",
                        )
                        start_inference_thread(args, config, policy, ros_operator)
                    act = stream_buffer.pop_next_action()
                    if act is not None:
                        if args.ctrl_type == "joint":
                            left_action = act[:7].copy()
                            right_action = act[7:14].copy()
                            left_action[6] = max(0.0, left_action[6]-RIGHT_OFFSET)
                            right_action[6] = max(0.0, right_action[6]-RIGHT_OFFSET)
                            ros_operator.puppet_arm_publish(left_action, right_action)
                        elif args.ctrl_type == "eef":
                            left_action = act[:7]
                            right_action = act[7:14]
                            ros_operator.endpose_publish(left_action, right_action)
                        last_stream_act = act.copy()
                    else:
                        if last_stream_act is not None:
                            if args.ctrl_type == "joint":
                                left_action = last_stream_act[:7]
                                right_action = last_stream_act[7:14]
                                ros_operator.puppet_arm_publish(left_action, right_action)
                            elif args.ctrl_type == "eef":
                                left_action = last_stream_act[:7]
                                right_action = last_stream_act[7:14]
                                ros_operator.endpose_publish(left_action, right_action)
                        else:
                            pass
                    rate.sleep()
                    t += 1
                    continue

                # Update observation window (blocking-chunk mode)
                update_observation_window(args, config, ros_operator)

                # When coming to the end of the action chunk
                if t % chunk_size == 0:
                    # Start inference
                    action_buffer = inference_fn(args, config, policy).copy()
                if dagger_mode_active:
                    continue

                raw_action = action_buffer[t % chunk_size]

                # Kalman filter for action smoothing
                if args.use_kalman_filter:
                    action = np.array([kf.update(raw_action[i]) for i, kf in enumerate(kalman_filters)])
                else:
                    action = raw_action
                # Interpolate the original action sequence
                if args.use_actions_interpolation:
                    if pre_action is None:
                        pre_action = observation_window[-1]["qpos"].copy()
                    # print(f"Time {t}, pre {pre_action}, act {action}")
                    if args.interpolate_method == "linear":
                        interp_actions = interpolate_action(args, pre_action, action)
                    elif args.interpolate_method == "minimum_jerk":
                        interp_actions = minimum_jerk_interpolation(args, pre_action, action)
                    else:
                        raise NotImplementedError
                else:
                    interp_actions = action[np.newaxis, :]
                # Execute the interpolated actions one by one
                for act in interp_actions:
                    if args.ctrl_type == "joint":
                        left_action = act[:7]
                        right_action = act[7:14]

                        # Gripper thresholding
                        if args.gripper_threshold:
                            if left_action[-1] < 0.03:
                                left_action[-1] = 0
                            if right_action[-1] < 0.03:
                                right_action[-1] = 0
                        
                        # ros_operator.puppet_arm_publish(right_action, left_action)
                        ros_operator.puppet_arm_publish(left_action, right_action)
                    elif args.ctrl_type == "eef":
                        left_action = act[:7]
                        right_action = act[7:14]

                        ros_operator.endpose_publish(left_action, right_action)

                    if args.use_robot_base:
                        vel_action = act[14:16]
                        ros_operator.robot_base_publish(vel_action)
                    rate.sleep()
                t += 1

                # Print step info less frequently to avoid cluttering terminal
                if t % 50 == 0:  # Print every 50 steps
                    print(f"Published Step {t} | Press 'd' for DAgger mode, 'r' to resume")
                pre_action = action.copy()


# ROS operator class
class RosOperator:
    
    def __init__(self, args):
        self.robot_base_deque = None
        self.puppet_arm_right_deque = None
        self.puppet_arm_left_deque = None
        self.img_front_deque = None
        self.img_right_deque = None
        self.img_left_deque = None
        self.img_front_depth_deque = None
        self.img_right_depth_deque = None
        self.img_left_depth_deque = None
        self.bridge = None
        self.puppet_arm_left_publisher = None
        self.puppet_arm_right_publisher = None
        self.endpose_left_publisher = None
        self.endpose_right_publisher = None
        self.robot_base_publisher = None
        self.puppet_arm_publish_thread = None
        self.puppet_arm_publish_lock = None
        self.args = args
        self.stop_flag = False
        self.master_handoff_lock = threading.Lock()
        self.master_control_feedback = {}
        self.master_prepared_feedback = {}
        self.master_command_seen_at = {}
        self.follower_ready_feedback = {}
        self._dagger_preroll = None
        self._pending_dagger_preroll = []
        self._dagger_live_after_source_time = None
        self.policy_input_lock = threading.Lock()
        self.policy_input_stamps = {}
        # DAgger mode - master arm control
        self.master_arm_left_deque = None
        self.master_arm_right_deque = None
        self.master_left_enable_pub = None
        self.master_right_enable_pub = None
        self.master_left_config_pub = None
        self.master_right_config_pub = None
        self.master_left_teach_mode_pub = None
        self.master_right_teach_mode_pub = None
        self.master_left_joint_pub = None
        self.master_right_joint_pub = None
        
        self.last_frame_cache = None
        self.args = args
        self.stop_flag = False

        # Dataset path: use dataset_name (strip _dagger/_inference suffix)
        dataset_base = getattr(self.args, "dataset_name", "flat_1107_27_35_mixed1.0")
        dataset_base = dataset_base.replace("_dagger_hdf5", "").replace("_inference_hdf5", "")

        base_data_dir = getattr(self.args, "dataset_dir", "/home/agilex/data")
        # DAgger data dir
        dagger_dir = os.path.join(base_data_dir, f"{dataset_base}_dagger_hdf5", "aloha_mobile_dummy")
        os.makedirs(dagger_dir, exist_ok=True)

        # Inference data dir
        inference_dir = os.path.join(base_data_dir, f"{dataset_base}_inference_hdf5", "aloha_mobile_dummy")
        os.makedirs(inference_dir, exist_ok=True)

        # Init collector
        self.data_collector = SimpleDAggerCollector(
            camera_names=['cam_high', 'cam_left_wrist', 'cam_right_wrist'],
            dataset_dir=base_data_dir,
            task_name=dataset_base
        )

        # Set collector DAgger save dir
        self.data_collector.full_dataset_dir = dagger_dir
        if hasattr(self.data_collector, "_find_next_episode_idx"):
            self.data_collector._find_next_episode_idx()

        # Keep inference dir for export
        self.inference_dataset_dir = inference_dir
        self.dataset_base = dataset_base
        # Init ROS etc.
        self.init()
        self.init_ros()
    
    
    # def __init__(self, args):
    #     self.robot_base_deque = None
    #     self.puppet_arm_right_deque = None
    #     self.puppet_arm_left_deque = None
    #     self.img_front_deque = None
    #     self.img_right_deque = None
    #     self.img_left_deque = None
    #     self.img_front_depth_deque = None
    #     self.img_right_depth_deque = None
    #     self.img_left_depth_deque = None
    #     self.bridge = None
    #     self.puppet_arm_left_publisher = None
    #     self.puppet_arm_right_publisher = None
    #     self.endpose_left_publisher = None
    #     self.endpose_right_publisher = None
    #     self.robot_base_publisher = None
    #     self.puppet_arm_publish_thread = None
    #     self.puppet_arm_publish_lock = None
    #     self.args = args
    #     self.stop_flag = False
    #     # DAgger mode - master arm control
    #     self.master_arm_left_deque = None
    #     self.master_arm_right_deque = None
    #     self.master_left_enable_pub = None
    #     self.master_right_enable_pub = None
    #     self.master_left_config_pub = None
    #     self.master_right_config_pub = None
    #     self.master_left_teach_mode_pub = None
    #     self.master_right_teach_mode_pub = None
    #     self.master_left_joint_pub = None
    #     self.master_right_joint_pub = None
        
    #     self.last_frame_cache = None

    #     # 🆕 DAgger数据采集器（直接使用原版save_data函数）
    #     task_name = getattr(args, 'dataset_name', 'flat_1107_27_35_mixed1.0')
    #     self.data_collector = SimpleDAggerCollector(
    #         camera_names=['cam_high', 'cam_left_wrist', 'cam_right_wrist'],
    #         dataset_dir="/home/agilex/data",
    #         task_name=task_name
    #     )

    #     # --- 🆕 新保存结构 ---
    #     # DAgger 数据保存目录
    #     dagger_dir = os.path.join(
    #         "/home/agilex/data",
    #         f"{task_name}_dagger",
    #         "aloha_mobile_dummy"
    #     )
    #     os.makedirs(dagger_dir, exist_ok=True)

    #     # Inference 数据保存目录
    #     inference_dir = os.path.join(
    #         "/home/agilex/data",
    #         f"{task_name}_inference",
    #         "aloha_mobile_dummy"
    #     )
    #     os.makedirs(inference_dir, exist_ok=True)

    #     # 设置 DAgger 采集器路径
    #     self.data_collector.full_dataset_dir = dagger_dir
    #     self.data_collector._find_next_episode_idx()

    #     # 存储 inference 数据路径供导出使用
    #     self.inference_dataset_dir = inference_dir

    #     # 初始化 ROS
    #     self.init()
    #     self.init_ros()

    
    # def __init__(self, args):
        # self.robot_base_deque = None
        # self.puppet_arm_right_deque = None
        # self.puppet_arm_left_deque = None
        # self.img_front_deque = None
        # self.img_right_deque = None
        # self.img_left_deque = None
        # self.img_front_depth_deque = None
        # self.img_right_depth_deque = None
        # self.img_left_depth_deque = None
        # self.bridge = None
        # self.puppet_arm_left_publisher = None
        # self.puppet_arm_right_publisher = None
        # self.endpose_left_publisher = None
        # self.endpose_right_publisher = None
        # self.robot_base_publisher = None
        # self.puppet_arm_publish_thread = None
        # self.puppet_arm_publish_lock = None
        # self.args = args
        # self.stop_flag = False
        # # DAgger mode - master arm control
        # self.master_arm_left_deque = None
        # self.master_arm_right_deque = None
        # self.master_left_enable_pub = None
        # self.master_right_enable_pub = None
        # self.master_left_config_pub = None
        # self.master_right_config_pub = None
        # self.master_left_teach_mode_pub = None
        # self.master_right_teach_mode_pub = None
        # self.master_left_joint_pub = None
        # self.master_right_joint_pub = None
        
        # self.last_frame_cache = None

    #     # 🆕 DAgger数据采集器（直接使用原版save_data函数）
    #     # 使用命令行参数中的数据集名称，如果没有则使用默认名称
    #     task_name = getattr(args, 'dataset_name', 'dagger_demo')
    #     self.data_collector = SimpleDAggerCollector(
    #         camera_names=['cam_high', 'cam_left_wrist', 'cam_right_wrist'],
    #         dataset_dir="/home/agilex/data",
    #         task_name=task_name
    #     )
    #     # 初始化数据集目录
    #     self.data_collector.full_dataset_dir = os.path.join(
    #         self.data_collector.dataset_dir,
    #         task_name,
    #         "aloha_dagger"
    #     )
    #     os.makedirs(self.data_collector.full_dataset_dir, exist_ok=True)
    #     self.data_collector._find_next_episode_idx()
        
    #     self.init()
    #     self.init_ros()

    def init(self):
        self.bridge = CvBridge()
        self.img_left_deque = deque()
        self.img_right_deque = deque()
        self.img_front_deque = deque()
        self.img_left_depth_deque = deque()
        self.img_right_depth_deque = deque()
        self.img_front_depth_deque = deque()
        self.puppet_arm_left_deque = deque()
        self.puppet_arm_right_deque = deque()
        self.robot_base_deque = deque()
        self.puppet_arm_publish_lock = threading.Lock()
        self.puppet_arm_publish_lock.acquire()
        
        # DAgger mode deques
        self.master_arm_left_deque = deque()
        self.master_arm_right_deque = deque()

    def record_policy_input_stamp(self, name, msg):
        with self.policy_input_lock:
            self.policy_input_stamps[name] = msg.header.stamp.to_sec()

    def require_fresh_policy_inputs(self):
        now = rospy.Time.now().to_sec()
        with self.policy_input_lock:
            stamps = dict(self.policy_input_stamps)
        for name, max_age in [('camera_front', 0.5), ('camera_left', 0.5), ('camera_right', 0.5),
                              ('follower_left', 0.25), ('follower_right', 0.25)]:
            stamp = stamps.get(name)
            if stamp is None or not math.isfinite(now) or not math.isfinite(stamp):
                raise RuntimeError("Missing/nonfinite policy input: " + name)
            age = now - stamp
            if not math.isfinite(age) or not -0.05 <= age <= max_age:
                raise RuntimeError("Stale/future policy input: %s (age %.3fs)" % (name, age))

    def wait_for_fresh_policy_inputs(self, timeout=15.0):
        deadline = time.monotonic() + timeout
        last_error = "No policy input received"
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            try:
                self.require_fresh_policy_inputs()
                return
            except RuntimeError as exc:
                last_error = str(exc)
            time.sleep(0.02)
        raise RuntimeError("Policy startup requires fresh three-camera/two-follower input: " + last_error)

    def puppet_arm_publish(self, left, right, *, require_policy_inputs=True):
        if require_policy_inputs:
            self.require_fresh_policy_inputs()
        joint_state_msg = JointState()
        joint_state_msg.header = Header()
        joint_state_msg.header.stamp = rospy.Time.now()  # Set timestep
        joint_state_msg.name = [
            "joint0",
            "joint1",
            "joint2",
            "joint3",
            "joint4",
            "joint5",
            "joint6",
        ]  # 设置关节名称
        joint_state_msg.position = left
        self.puppet_arm_left_publisher.publish(joint_state_msg)
        joint_state_msg.position = right
        self.puppet_arm_right_publisher.publish(joint_state_msg)

    def endpose_publish(self, left, right):
        self.require_fresh_policy_inputs()
        endpose_msg = PosCmd()
        endpose_msg.x, endpose_msg.y, endpose_msg.z = left[:3]
        endpose_msg.roll, endpose_msg.pitch, endpose_msg.yaw = left[3:6]
        endpose_msg.gripper = left[6]
        self.endpose_left_publisher.publish(endpose_msg)

        endpose_msg.x, endpose_msg.y, endpose_msg.z = right[:3]
        endpose_msg.roll, endpose_msg.pitch, endpose_msg.yaw = right[3:6]
        endpose_msg.gripper = right[6]
        self.endpose_right_publisher.publish(endpose_msg)

    def robot_base_publish(self, vel):
        self.require_fresh_policy_inputs()
        vel_msg = Twist()
        vel_msg.linear.x = vel[0]
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = vel[1]
        self.robot_base_publisher.publish(vel_msg)

    def puppet_arm_publish_continuous(self, left, right):
        rate = rospy.Rate(self.args.publish_rate)
        left_arm = None
        right_arm = None
        while True and not rospy.is_shutdown():
            if len(self.puppet_arm_left_deque) != 0:
                left_arm = list(self.puppet_arm_left_deque[-1].position)
            if len(self.puppet_arm_right_deque) != 0:
                right_arm = list(self.puppet_arm_right_deque[-1].position)
            if left_arm is None or right_arm is None:
                rate.sleep()
                continue
            else:
                break
        left_symbol = [1 if left[i] - left_arm[i] > 0 else -1 for i in range(len(left))]
        right_symbol = [1 if right[i] - right_arm[i] > 0 else -1 for i in range(len(right))]
        flag = True
        step = 0
        while flag and not rospy.is_shutdown():
            if self.puppet_arm_publish_lock.acquire(False):
                return
            left_diff = [abs(left[i] - left_arm[i]) for i in range(len(left))]
            right_diff = [abs(right[i] - right_arm[i]) for i in range(len(right))]
            flag = False
            for i in range(len(left)):
                if left_diff[i] < self.args.arm_steps_length[i]:
                    left_arm[i] = left[i]
                else:
                    left_arm[i] += left_symbol[i] * self.args.arm_steps_length[i]
                    flag = True
            for i in range(len(right)):
                if right_diff[i] < self.args.arm_steps_length[i]:
                    right_arm[i] = right[i]
                else:
                    right_arm[i] += right_symbol[i] * self.args.arm_steps_length[i]
                    flag = True
            self.require_fresh_policy_inputs()
            joint_state_msg = JointState()
            joint_state_msg.header = Header()
            joint_state_msg.header.stamp = rospy.Time.now()  # Set the timestep
            joint_state_msg.name = [
                "joint0",
                "joint1",
                "joint2",
                "joint3",
                "joint4",
                "joint5",
                "joint6",
            ]  # 设置关节名称
            joint_state_msg.position = left_arm
            self.puppet_arm_left_publisher.publish(joint_state_msg)
            joint_state_msg.position = right_arm
            self.puppet_arm_right_publisher.publish(joint_state_msg)
            step += 1
            print("puppet_arm_publish_continuous:", step)
            rate.sleep()

    def puppet_arm_publish_linear(self, left, right):
        num_step = 100
        rate = rospy.Rate(200)

        left_arm = None
        right_arm = None

        while True and not rospy.is_shutdown():
            if len(self.puppet_arm_left_deque) != 0:
                left_arm = list(self.puppet_arm_left_deque[-1].position)
            if len(self.puppet_arm_right_deque) != 0:
                right_arm = list(self.puppet_arm_right_deque[-1].position)
            if left_arm is None or right_arm is None:
                rate.sleep()
                continue
            else:
                break

        traj_left_list = np.linspace(left_arm, left, num_step)
        traj_right_list = np.linspace(right_arm, right, num_step)

        for i in range(len(traj_left_list)):
            traj_left = traj_left_list[i]
            traj_right = traj_right_list[i]
            traj_left[-1] = left[-1]
            traj_right[-1] = right[-1]
            joint_state_msg = JointState()
            joint_state_msg.header = Header()
            joint_state_msg.header.stamp = rospy.Time.now()  # 设置时间戳
            joint_state_msg.name = [
                "joint0",
                "joint1",
                "joint2",
                "joint3",
                "joint4",
                "joint5",
                "joint6",
            ]  # 设置关节名称
            joint_state_msg.position = traj_left
            self.puppet_arm_left_publisher.publish(joint_state_msg)
            joint_state_msg.position = traj_right
            self.puppet_arm_right_publisher.publish(joint_state_msg)
            rate.sleep()

    def puppet_arm_publish_continuous_thread(self, left, right):
        if self.puppet_arm_publish_thread is not None:
            self.puppet_arm_publish_lock.release()
            self.puppet_arm_publish_thread.join()
            self.puppet_arm_publish_lock.acquire(False)
            self.puppet_arm_publish_thread = None
        self.puppet_arm_publish_thread = threading.Thread(target=self.puppet_arm_publish_continuous, args=(left, right))
        self.puppet_arm_publish_thread.start()

    def get_frame(self):
        if (
            len(self.img_left_deque) == 0
            or len(self.img_right_deque) == 0
            or len(self.img_front_deque) == 0
            or (
                self.args.use_depth_image
                and (
                    len(self.img_left_depth_deque) == 0
                    or len(self.img_right_depth_deque) == 0
                    or len(self.img_front_depth_deque) == 0
                )
            )
        ):
            return False
        if self.args.use_depth_image:
            frame_time = min(
                [
                    self.img_left_deque[-1].header.stamp.to_sec(),
                    self.img_right_deque[-1].header.stamp.to_sec(),
                    self.img_front_deque[-1].header.stamp.to_sec(),
                    self.img_left_depth_deque[-1].header.stamp.to_sec(),
                    self.img_right_depth_deque[-1].header.stamp.to_sec(),
                    self.img_front_depth_deque[-1].header.stamp.to_sec(),
                ]
            )
        else:
            frame_time = min(
                [
                    self.img_left_deque[-1].header.stamp.to_sec(),
                    self.img_right_deque[-1].header.stamp.to_sec(),
                    self.img_front_deque[-1].header.stamp.to_sec(),
                ]
            )

        if len(self.img_left_deque) == 0 or self.img_left_deque[-1].header.stamp.to_sec() < frame_time:
            return False
        if len(self.img_right_deque) == 0 or self.img_right_deque[-1].header.stamp.to_sec() < frame_time:
            return False
        if len(self.img_front_deque) == 0 or self.img_front_deque[-1].header.stamp.to_sec() < frame_time:
            return False
        if len(self.puppet_arm_left_deque) == 0 or self.puppet_arm_left_deque[-1].header.stamp.to_sec() < frame_time:
            return False
        if len(self.puppet_arm_right_deque) == 0 or self.puppet_arm_right_deque[-1].header.stamp.to_sec() < frame_time:
            return False
        if self.args.use_depth_image and (
            len(self.img_left_depth_deque) == 0 or self.img_left_depth_deque[-1].header.stamp.to_sec() < frame_time
        ):
            return False
        if self.args.use_depth_image and (
            len(self.img_right_depth_deque) == 0 or self.img_right_depth_deque[-1].header.stamp.to_sec() < frame_time
        ):
            return False
        if self.args.use_depth_image and (
            len(self.img_front_depth_deque) == 0 or self.img_front_depth_deque[-1].header.stamp.to_sec() < frame_time
        ):
            return False
        if self.args.use_robot_base and (
            len(self.robot_base_deque) == 0 or self.robot_base_deque[-1].header.stamp.to_sec() < frame_time
        ):
            return False

        while self.img_left_deque[0].header.stamp.to_sec() < frame_time:
            self.img_left_deque.popleft()
        img_left = self.bridge.imgmsg_to_cv2(self.img_left_deque.popleft(), "bgr8")

        while self.img_right_deque[0].header.stamp.to_sec() < frame_time:
            self.img_right_deque.popleft()
        img_right = self.bridge.imgmsg_to_cv2(self.img_right_deque.popleft(), "bgr8")

        while self.img_front_deque[0].header.stamp.to_sec() < frame_time:
            self.img_front_deque.popleft()
        img_front = self.bridge.imgmsg_to_cv2(self.img_front_deque.popleft(), "bgr8")

        while self.puppet_arm_left_deque[0].header.stamp.to_sec() < frame_time:
            self.puppet_arm_left_deque.popleft()
        puppet_arm_left = self.puppet_arm_left_deque.popleft()

        while self.puppet_arm_right_deque[0].header.stamp.to_sec() < frame_time:
            self.puppet_arm_right_deque.popleft()
        puppet_arm_right = self.puppet_arm_right_deque.popleft()

        img_left_depth = None
        if self.args.use_depth_image:
            while self.img_left_depth_deque[0].header.stamp.to_sec() < frame_time:
                self.img_left_depth_deque.popleft()
            img_left_depth = self.bridge.imgmsg_to_cv2(self.img_left_depth_deque.popleft(), "passthrough")

        img_right_depth = None
        if self.args.use_depth_image:
            while self.img_right_depth_deque[0].header.stamp.to_sec() < frame_time:
                self.img_right_depth_deque.popleft()
            img_right_depth = self.bridge.imgmsg_to_cv2(self.img_right_depth_deque.popleft(), "passthrough")

        img_front_depth = None
        if self.args.use_depth_image:
            while self.img_front_depth_deque[0].header.stamp.to_sec() < frame_time:
                self.img_front_depth_deque.popleft()
            img_front_depth = self.bridge.imgmsg_to_cv2(self.img_front_depth_deque.popleft(), "passthrough")

        robot_base = None
        if self.args.use_robot_base:
            while self.robot_base_deque[0].header.stamp.to_sec() < frame_time:
                self.robot_base_deque.popleft()
            robot_base = self.robot_base_deque.popleft()

        result_tuple = (
            img_front,
            img_left,
            img_right,
            img_front_depth,
            img_left_depth,
            img_right_depth,
            puppet_arm_left,
            puppet_arm_right,
            robot_base,
        )
        self.last_frame_cache = result_tuple
        return result_tuple

    def get_frame_peek(self):
        if (
            len(self.img_left_deque) == 0
            or len(self.img_right_deque) == 0
            or len(self.img_front_deque) == 0
            or (
                self.args.use_depth_image
                and (
                    len(self.img_left_depth_deque) == 0
                    or len(self.img_right_depth_deque) == 0
                    or len(self.img_front_depth_deque) == 0
                )
            )
        ):
            return False

        if self.args.use_depth_image:
            frame_time = min(
                [
                    self.img_left_deque[-1].header.stamp.to_sec(),
                    self.img_right_deque[-1].header.stamp.to_sec(),
                    self.img_front_deque[-1].header.stamp.to_sec(),
                    self.img_left_depth_deque[-1].header.stamp.to_sec(),
                    self.img_right_depth_deque[-1].header.stamp.to_sec(),
                    self.img_front_depth_deque[-1].header.stamp.to_sec(),
                ]
            )
        else:
            frame_time = min(
                [
                    self.img_left_deque[-1].header.stamp.to_sec(),
                    self.img_right_deque[-1].header.stamp.to_sec(),
                    self.img_front_deque[-1].header.stamp.to_sec(),
                ]
            )

        for dq in [
            self.img_left_deque,
            self.img_right_deque,
            self.img_front_deque,
            self.puppet_arm_left_deque,
            self.puppet_arm_right_deque,
        ]:
            if len(dq) == 0 or dq[-1].header.stamp.to_sec() < frame_time:
                return False
        if self.args.use_depth_image:
            for dq in [self.img_left_depth_deque, self.img_right_depth_deque, self.img_front_depth_deque]:
                if len(dq) == 0 or dq[-1].header.stamp.to_sec() < frame_time:
                    return False
        if self.args.use_robot_base:
            if len(self.robot_base_deque) == 0 or self.robot_base_deque[-1].header.stamp.to_sec() < frame_time:
                return False

        def first_ge(dq, t):
            for item in dq:
                if item.header.stamp.to_sec() >= t:
                    return item
            return dq[-1]

        img_left_msg = first_ge(self.img_left_deque, frame_time)
        img_right_msg = first_ge(self.img_right_deque, frame_time)
        img_front_msg = first_ge(self.img_front_deque, frame_time)
        puppet_arm_left = first_ge(self.puppet_arm_left_deque, frame_time)
        puppet_arm_right = first_ge(self.puppet_arm_right_deque, frame_time)

        img_left = self.bridge.imgmsg_to_cv2(img_left_msg, "bgr8")
        img_right = self.bridge.imgmsg_to_cv2(img_right_msg, "bgr8")
        img_front = self.bridge.imgmsg_to_cv2(img_front_msg, "bgr8")

        img_left_depth = None
        img_right_depth = None
        img_front_depth = None
        if self.args.use_depth_image:
            img_left_depth = self.bridge.imgmsg_to_cv2(first_ge(self.img_left_depth_deque, frame_time), "passthrough")
            img_right_depth = self.bridge.imgmsg_to_cv2(first_ge(self.img_right_depth_deque, frame_time), "passthrough")
            img_front_depth = self.bridge.imgmsg_to_cv2(first_ge(self.img_front_depth_deque, frame_time), "passthrough")

        robot_base = None
        if self.args.use_robot_base:
            robot_base = first_ge(self.robot_base_deque,model_inference, frame_time)

        return (
            img_front,
            img_left,
            img_right,
            img_front_depth,
            img_left_depth,
            img_right_depth,
            puppet_arm_left,
            puppet_arm_right,
            robot_base,
        )

    def img_left_callback(self, msg):
        self.record_policy_input_stamp('camera_left', msg)
        if len(self.img_left_deque) >= 2000:
            self.img_left_deque.popleft()
        self.img_left_deque.append(msg)

    def img_right_callback(self, msg):
        self.record_policy_input_stamp('camera_right', msg)
        if len(self.img_right_deque) >= 2000:
            self.img_right_deque.popleft()
        self.img_right_deque.append(msg)

    def img_front_callback(self, msg):
        self.record_policy_input_stamp('camera_front', msg)
        if len(self.img_front_deque) >= 2000:
            self.img_front_deque.popleft()
        self.img_front_deque.append(msg)

    def img_left_depth_callback(self, msg):
        if len(self.img_left_depth_deque) >= 2000:
            self.img_left_depth_deque.popleft()
        self.img_left_depth_deque.append(msg)

    def img_right_depth_callback(self, msg):
        if len(self.img_right_depth_deque) >= 2000:
            self.img_right_depth_deque.popleft()
        self.img_right_depth_deque.append(msg)

    def img_front_depth_callback(self, msg):
        if len(self.img_front_depth_deque) >= 2000:
            self.img_front_depth_deque.popleft()
        self.img_front_depth_deque.append(msg)

    def puppet_arm_left_callback(self, msg):
        self.record_policy_input_stamp('follower_left', msg)
        if len(self.puppet_arm_left_deque) >= 2000:
            self.puppet_arm_left_deque.popleft()
        self.puppet_arm_left_deque.append(msg)

    def puppet_arm_right_callback(self, msg):
        self.record_policy_input_stamp('follower_right', msg)
        if len(self.puppet_arm_right_deque) >= 2000:
            self.puppet_arm_right_deque.popleft()
        self.puppet_arm_right_deque.append(msg)

    def robot_base_callback(self, msg):
        if len(self.robot_base_deque) >= 2000:
            self.robot_base_deque.popleft()
        self.robot_base_deque.append(msg)
    
    def master_arm_left_callback(self, msg):
        """Callback for master arm left joint states"""
        if len(self.master_arm_left_deque) >= 2000:
            self.master_arm_left_deque.popleft()
        self.master_arm_left_deque.append(msg)
    
    def master_arm_right_callback(self, msg):
        """Callback for master arm right joint states"""
        if len(self.master_arm_right_deque) >= 2000:
            self.master_arm_right_deque.popleft()
        self.master_arm_right_deque.append(msg)
    
    def get_synced_frame(self):
        """
        Get time-synced frame (same as collect_data).
        
        Returns:
            tuple: (front_img, left_img, right_img, puppet_left, puppet_right, master_left, master_right)
            or None if not ready.
        """
        # Check all deques have data
        if (len(self.img_left_deque) == 0 or len(self.img_right_deque) == 0 or 
            len(self.img_front_deque) == 0):
            return None
        
        # Earliest frame timestamp across sensors
        frame_time = min([
            self.img_left_deque[-1].header.stamp.to_sec(),
            self.img_right_deque[-1].header.stamp.to_sec(),
            self.img_front_deque[-1].header.stamp.to_sec()
        ])
        if (self._dagger_live_after_source_time is not None
                and frame_time <= self._dagger_live_after_source_time):
            return None
        
        # Check all data has reached this time
        for dq in [self.img_left_deque, self.img_right_deque, self.img_front_deque,
                   self.puppet_arm_left_deque, self.puppet_arm_right_deque]:
            if not dq or dq[-1].header.stamp.to_sec() < frame_time:
                return None
        
        def pop(dq):
            # Drop data older than frame_time
            while len(dq) > 1 and dq[0].header.stamp.to_sec() < frame_time:
                dq.popleft()
            # Return first with timestamp >= frame_time
            return dq.popleft()
        
        try:
            # Get data in original order
            img_left = self.bridge.imgmsg_to_cv2(pop(self.img_left_deque), 'bgr8')
            img_right = self.bridge.imgmsg_to_cv2(pop(self.img_right_deque), 'bgr8')
            img_front = self.bridge.imgmsg_to_cv2(pop(self.img_front_deque), 'bgr8')
            # These master observations are diagnostic only. DAgger qpos/action
            # use actual follower states, so a legal idle master must not block
            # camera/follower synchronization or receive an invented timestamp.
            master_arm_left = self.master_arm_left_deque[-1] if self.master_arm_left_deque else None
            master_arm_right = self.master_arm_right_deque[-1] if self.master_arm_right_deque else None
            puppet_arm_left = pop(self.puppet_arm_left_deque)
            puppet_arm_right = pop(self.puppet_arm_right_deque)
            
            return (img_front, img_left, img_right, 
                    puppet_arm_left, puppet_arm_right, 
                    master_arm_left, master_arm_right)
        except Exception as e:
            print(f"get_synced_frame error: {e}")
            return None

    def _build_dagger_sample(self, synced_data):
        """Use the same actual-follower observation/action schema for both paths."""
        front_img, left_img, right_img, puppet_left, puppet_right, _, _ = synced_data
        observation = collections.OrderedDict()
        observation['images'] = {
            'cam_high': front_img,
            'cam_left_wrist': left_img,
            'cam_right_wrist': right_img,
        }
        observation['qpos'] = np.concatenate((puppet_left.position, puppet_right.position))
        observation['qvel'] = np.concatenate((puppet_left.velocity, puppet_right.velocity))
        observation['effort'] = np.concatenate((puppet_left.effort, puppet_right.effort))
        observation['base_vel'] = [0.0, 0.0]
        action = np.concatenate((puppet_left.position[:7], puppet_right.position[:7]))
        return observation, action

    def _snapshot_dagger_sample(self):
        """Sample five synchronized streams without consuming the control deques."""
        streams = tuple(tuple(stream) for stream in (
            self.img_left_deque, self.img_right_deque, self.img_front_deque,
            self.puppet_arm_left_deque, self.puppet_arm_right_deque))
        if not all(streams):
            return None
        source_time = min(stream[-1].header.stamp.to_sec() for stream in streams[:3])
        if any(stream[-1].header.stamp.to_sec() < source_time for stream in streams):
            return None
        messages = [next(message for message in stream
                         if message.header.stamp.to_sec() >= source_time)
                    for stream in streams]
        left_img, right_img, front_img = (
            self.bridge.imgmsg_to_cv2(message, 'bgr8') for message in messages[:3])
        observation, action = self._build_dagger_sample(
            (front_img, left_img, right_img, messages[3], messages[4], None, None))
        return source_time, observation, action

    def start_dagger_preroll(self):
        """Keep only one second of real samples while the original entry runs."""
        self._pending_dagger_preroll = []
        self._dagger_live_after_source_time = None
        state = {'stop': threading.Event(), 'samples': deque()}
        self._dagger_preroll = state

        def sample():
            last_source_time = None
            while not state['stop'].is_set() and not rospy.is_shutdown():
                try:
                    frame = self._snapshot_dagger_sample()
                    if frame is not None:
                        source_time, observation, action = frame
                        if last_source_time is None or source_time > last_source_time:
                            sampled_at = time.monotonic()
                            state['samples'].append((sampled_at, source_time, observation, action))
                            last_source_time = source_time
                            while state['samples'][0][1] < source_time - 1.0:
                                state['samples'].popleft()
                except Exception as exc:
                    print(f"DAgger pre-roll sample error: {exc}")
                state['stop'].wait(1.0 / self.args.publish_rate)

        state['thread'] = threading.Thread(target=sample, daemon=False)
        state['thread'].start()

    def finish_dagger_preroll(self, ready_at=None):
        """End this data-only worker and retain the second before normal readiness."""
        state = self._dagger_preroll
        state['stop'].set()
        state['thread'].join()
        self._dagger_preroll = None
        self._pending_dagger_preroll = ([] if ready_at is None or rospy.is_shutdown() else [
            sample for sample in state['samples'] if ready_at - 1.0 <= sample[1] <= ready_at])

    def consume_dagger_preroll(self):
        """Append the entry prefix once, after start_collection has reset the episode."""
        samples = self._pending_dagger_preroll
        self._pending_dagger_preroll = []
        for _, source_time, observation, action in samples:
            self.data_collector.add_frame(observation, action)
            self._dagger_live_after_source_time = source_time
    
    def master_control_status_callback(self, msg, side):
        with self.master_handoff_lock:
            received = time.monotonic()
            self.master_control_feedback[side] = (received, msg.angular.y, msg.linear.z)
            self.master_prepared_feedback[side] = {
                'received_monotonic': received, 'source_time': msg.angular.x,
                'phase_since': msg.angular.z, 'config': msg.angular.y,
                'teaching': msg.linear.z, 'enabled': msg.linear.y}

    def master_command_callback(self, msg, side):
        with self.master_handoff_lock:
            self.master_command_seen_at[side] = time.monotonic()

    def follower_ready_callback(self, msg, side):
        with self.master_handoff_lock:
            self.follower_ready_feedback[side] = {
                'received_monotonic': time.monotonic(), 'source_time': msg.angular.x,
                'ready': msg.linear.x == 1.0, 'command_time': msg.linear.y}

    def wait_for_master_control(self, config, teaching, since, timeout=15.0, quiet_period=0.0):
        deadline = None if timeout is None else time.monotonic() + timeout
        while not rospy.is_shutdown() and (deadline is None or time.monotonic() < deadline):
            now = time.monotonic()
            with self.master_handoff_lock:
                ready = all(
                    side in self.master_control_feedback
                    and self.master_control_feedback[side][0] >= since
                    and -0.05 <= now - self.master_control_feedback[side][0] < 0.5
                    and self.master_control_feedback[side][1:] == (config, int(teaching))
                    for side in ('left', 'right')
                )
                quiet = all(now - max(since, self.master_command_seen_at.get(side, since)) >= quiet_period
                            for side in ('left', 'right'))
                streaming = not teaching or all(
                    side in self.master_command_seen_at
                    and self.master_command_seen_at[side] >= since
                    and self.master_command_seen_at[side] <= now + 0.05
                    for side in ('left', 'right')
                )
            if ready and quiet and streaming:
                if not teaching or self.dagger_recording_status()[0]:
                    return
            time.sleep(0.02)
        raise RuntimeError("Master handoff incomplete: " + self._master_handoff_diagnostics(
            config, teaching, since, quiet_period) + "; policy stays paused")

    def _master_handoff_diagnostics(self, config, teaching, since, quiet_period):
        now = time.monotonic()
        with self.master_handoff_lock:
            modes = dict(self.master_control_feedback)
            metadata = dict(getattr(self, 'master_prepared_feedback', {}))
            commands = dict(self.master_command_seen_at)
        details = ["expected mode=0x%02X teaching=%d" % (config, teaching)]
        for side in ('left', 'right'):
            row = modes.get(side)
            if row is None:
                state = 'mode=missing'
            else:
                state = 'mode=0x%02X teaching=%s enabled=%s status_age=%.3fs after_request=%s' % (
                    int(row[1]), row[2], metadata.get(side, {}).get('enabled', 'unknown'),
                    now - row[0], row[0] >= since)
            if teaching:
                command_state = 'post_request_command=%s' % (commands.get(side, -1) >= since)
            else:
                quiet_for = now - max(since, commands.get(side, since))
                command_state = 'publisher=%s quiet_for=%.3fs required=%.3fs' % (
                    'quiet' if quiet_for >= quiet_period else 'still-publishing', quiet_for, quiet_period)
            details.append(side + '[' + state + '; ' + command_state + ']')
        return '; '.join(details)

    def dagger_recording_status(self):
        """Read confirmed teaching mode; never infer a motion fault from silence."""
        now, wall_now = time.monotonic(), time.time()
        with self.master_handoff_lock:
            rows = {side: dict(row) for side, row in self.master_prepared_feedback.items()}
            commands = dict(self.master_command_seen_at)
        for side in ('left', 'right'):
            row = rows.get(side)
            if row is None:
                return False, side + ' master mode confirmation is missing'
            values = [row.get(key) for key in ('received_monotonic', 'source_time', 'config', 'teaching', 'enabled')]
            if not all(isinstance(value, (int, float)) and math.isfinite(value) for value in values):
                return False, side + ' master mode confirmation is invalid'
            if (not -0.05 <= now - row['received_monotonic'] < 0.5
                    or not -0.05 <= wall_now - row['source_time'] < 0.5):
                return False, side + ' master mode confirmation is temporarily unavailable'
            if row['config'] != 0xFA or row['teaching'] != 1 or row['enabled'] != 1:
                return False, side + ' master teaching/following mode is not confirmed'
        if any(side not in commands or now - commands[side] >= 0.5 for side in ('left', 'right')):
            return True, 'Waiting for new master input; stationary follower states may still be recorded'
        return True, 'Teaching/following mode confirmed'

    def dagger_recording_ready(self):
        ready, reason = self.dagger_recording_status()
        notice = (ready, reason)
        if notice != getattr(self, '_dagger_recording_notice', None):
            prefix = 'Recording status' if ready else 'Data append paused; current data is kept'
            print('\n' + prefix + ': ' + reason + '\n')
            self._dagger_recording_notice = notice
        return ready

    def set_dagger_following(self, enabled):
        """Gate native FA forwarding while a correction recording is stopped."""
        message = String(data='resume_following' if enabled else 'pause_following')
        self.master_left_config_pub.publish(message)
        self.master_right_config_pub.publish(message)

    def exit_dagger_mode(self):
        """Restore exclusive policy control before allowing inference to resume."""
        since = time.monotonic()
        config_msg = String(data="slave")
        self.master_left_config_pub.publish(config_msg)
        self.master_right_config_pub.publish(config_msg)
        self.wait_for_master_control(0xFC, False, since, quiet_period=0.25)
        rospy.loginfo("Master slave-mode acknowledged and command publishers quiet; restoring policy control")

    def enable_master_arms(self):
        """Enable both master arms for DAgger mode"""
        rospy.loginfo("🔧 Enabling left and right master arms for DAgger mode...")
        enable_msg = Bool(data=True)
        
        for i in range(3):
            self.master_left_enable_pub.publish(enable_msg)
            self.master_right_enable_pub.publish(enable_msg)
            rospy.loginfo("Master arms enable %d/3", i+1)
            rospy.sleep(0.05)
        
        rospy.loginfo("✅ Both master arms enabled")
        rospy.sleep(0.2)
    
    def move_masters_to_slave_pose(self, duration=3.0, prepare_following=False):
        """Move master arms to match current slave arm positions"""
        rospy.loginfo("🎯 Moving master arms to match slave positions...")
        
        # Get current slave positions
        if len(self.puppet_arm_left_deque) == 0 or len(self.puppet_arm_right_deque) == 0:
            rospy.logerr("❌ No slave arm data available")
            return False
        
        left_slave_pos = list(self.puppet_arm_left_deque[-1].position)
        right_slave_pos = list(self.puppet_arm_right_deque[-1].position)
        
        rospy.loginfo("Left slave pos: %s", left_slave_pos[:3])
        rospy.loginfo("Right slave pos: %s", right_slave_pos[:3])
        return self._move_masters_to_pose(
            left_slave_pos, right_slave_pos, duration, "slave pose", hold_seconds=3.0,
            prepare_following=prepare_following)

    def _move_masters_to_pose(self, left, right, duration, label, hold_seconds=0.0,
                              prepare_following=False):
        """Verify arrival, then optionally hold using fresh actual six-joint feedback."""
        targets = {"left": left, "right": right}
        try:
            alignment_targets(targets)  # Reject malformed targets before publishing anything.
            duration = float(duration)
            if not math.isfinite(duration) or duration <= 0:
                raise ValueError("Alignment duration must be finite and positive")
            duration = max(3.0, duration)
            hold_seconds = float(hold_seconds)
            if not math.isfinite(hold_seconds) or hold_seconds < 0:
                raise ValueError("Final-pose hold duration must be finite and nonnegative")
        except (TypeError, ValueError, OverflowError) as exc:
            rospy.logerr("Invalid master alignment target: %s", exc)
            return False
        messages = {}
        for side in ("left", "right"):
            message = JointState()
            message.header = Header()
            message.name = ['joint_0', 'joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
            message.position = list(targets[side])
            message.velocity = [0.0] * 7
            message.effort = [0.0] * 7
            messages[side] = message

        def publish_final_targets():
            for side in ("left", "right"):
                messages[side].header.stamp = rospy.Time.now()
            self.master_left_joint_pub.publish(messages["left"])
            self.master_right_joint_pub.publish(messages["right"])

        rate = rospy.Rate(10)
        end_time = time.monotonic() + duration
        while not rospy.is_shutdown() and time.monotonic() < end_time:
            publish_final_targets()
            rate.sleep()
        if rospy.is_shutdown():
            return False
        report = wait_for_master_alignment(
            targets, since_timestamp=time.time(), publish=publish_final_targets,
            shutdown=rospy.is_shutdown, timeout=8.0, tolerance_mdeg=1000, stable_seconds=0.3)
        arrived_at = time.monotonic()
        if not hasattr(self, "master_alignment_reports"):
            self.master_alignment_reports = {}
        self.master_alignment_reports[label] = report
        for side, row in report["sides"].items():
            message = "%s %s master errors J1..J6 (deg): %s; gripper delta %.1f mm (diagnostic only)"
            values = (label, side, [round(value, 3) for value in row["error_degrees"]],
                      row["gripper_error_um"] / 1000.0)
            if report["passed"]:
                rospy.loginfo(message, *values)
            else:
                rospy.logerr(message, *values)
        if not report["passed"]:
            rospy.logerr("Master alignment failed at %s: %s; teaching remains blocked", label, report.get("error"))
            return False
        rospy.loginfo("Both masters' six actual joints converged at %s; gripper matching not verified", label)
        if hold_seconds:
            if prepare_following:
                return self._hold_final_pose_and_prepare(
                    targets, publish_final_targets, hold_seconds, arrived_at, label)
            rospy.loginfo("Final master pose verified. Holding position for %.1f seconds before automatic teaching/recording...",
                          hold_seconds)
            hold_report = wait_for_master_alignment(
                targets, since_timestamp=time.time(), publish=publish_final_targets,
                shutdown=rospy.is_shutdown, timeout=hold_seconds + 1.5,
                tolerance_mdeg=1000, stable_seconds=hold_seconds)
            self.master_alignment_reports[label + " hold"] = hold_report
            if not hold_report["passed"]:
                rospy.logerr("Final master pose hold failed: %s; teaching/recording remain blocked",
                             hold_report.get("error"))
                return False
            rospy.loginfo("Final master pose held and verified for %.1f seconds; switching to teaching next",
                          hold_seconds)
        return True

    def _masters_prepared_current(self, since, wall_since):
        now, wall_now = time.monotonic(), time.time()
        with self.master_handoff_lock:
            rows = {side: dict(row) for side, row in self.master_prepared_feedback.items()}
        for side in ('left', 'right'):
            row = rows.get(side)
            if row is None:
                return False
            values = [row.get(key) for key in ('received_monotonic', 'source_time', 'phase_since',
                                               'config', 'teaching', 'enabled')]
            if not all(isinstance(value, (int, float)) and math.isfinite(value) for value in values):
                return False
            if (row['received_monotonic'] < since or row['phase_since'] < wall_since
                    or row['phase_since'] > wall_now + 0.05
                    or row['phase_since'] > row['source_time'] + 0.05
                    or not -0.05 <= now - row['received_monotonic'] < 0.5
                    or not -0.05 <= wall_now - row['source_time'] < 0.5
                    or row['config'] != 0xFC or row['teaching'] != 0 or row['enabled'] != 1):
                return False
        return True

    def _check_preparation_cancelled(self, state):
        if state['cancel'].is_set() or rospy.is_shutdown():
            raise RuntimeError("Following preparation was cancelled; masters remain position-held")
        if time.monotonic() >= state['deadline']:
            raise RuntimeError("Parallel holding/following preparation exceeded its deadline")

    def _prepare_following_worker(self, state):
        try:
            self._check_preparation_cancelled(state)
            self.assert_followers_ready()
            self._check_preparation_cancelled(state)
            self.probe_followers_for_teaching(
                timeout=min(3.0, state['deadline'] - time.monotonic()), cancel_event=state['cancel'])
            self._check_preparation_cancelled(state)
            since, wall_since = time.monotonic(), time.time()
            state.update(requested_monotonic=since, requested_wall=wall_since)
            message = String(data='prepare_master')
            for side in ('left', 'right'):
                self._check_preparation_cancelled(state)
                getattr(self, 'master_' + side + '_config_pub').publish(message)
            while not self._masters_prepared_current(since, wall_since):
                self._check_preparation_cancelled(state)
                time.sleep(0.02)
            self._check_preparation_cancelled(state)
            state.update(success=True, prepared_at=time.monotonic())
        except BaseException as exc:
            state['error'] = exc
        finally:
            state['done'].set()

    def _start_following_preparation(self, started_at, timeout=15.0):
        state = {'started_monotonic': started_at, 'started_wall': time.time(),
                 'deadline': started_at + timeout, 'cancel': threading.Event(),
                 'done': threading.Event(), 'success': False, 'hold_passed': False, 'error': None}
        worker = threading.Thread(target=self._prepare_following_worker, args=(state,), daemon=True)
        state['thread'] = worker
        self.parallel_following_preparation = state
        worker.start()
        return state

    def _following_preparation_guard(self, state):
        self._check_preparation_cancelled(state)
        if state['done'].is_set():
            if state['error'] is not None:
                raise RuntimeError("Following preparation failed: " + str(state['error'])) from state['error']
            if not state['success']:
                raise RuntimeError("Following preparation ended without readiness")
        # Missing/stale/temporarily unprepared status only delays release.
        # An explicit fresh fault/unknown mode or lost enable is stronger
        # negative evidence, even if initial preparation has not completed.
        if 'requested_monotonic' in state:
            now, wall_now = time.monotonic(), time.time()
            with self.master_handoff_lock:
                rows = {side: dict(row) for side, row in self.master_prepared_feedback.items()}
            for side, row in rows.items():
                if (row['received_monotonic'] >= state['requested_monotonic']
                        and 0 <= now - row['received_monotonic'] < 0.5
                        and -0.05 <= wall_now - row['source_time'] < 0.5
                        and (row['config'] != 0xFC or row['teaching'] != 0 or row['enabled'] != 1)):
                    raise RuntimeError("Prepared master entered an invalid mode or lost enable: " + side)

    def _following_preparation_ready(self, state):
        self._following_preparation_guard(state)
        return (state['done'].is_set() and state['success']
                and self._masters_prepared_current(state['requested_monotonic'], state['requested_wall']))

    def _hold_final_pose_and_prepare(self, targets, publish, hold_seconds, arrived_at, label):
        """Hold and prepare concurrently; only their combined success can permit release."""
        state = self._start_following_preparation(arrived_at, timeout=15.0)
        hold_report = None
        try:
            rospy.loginfo("Final pose reached: holding %.1fs and preparing follower tracking concurrently", hold_seconds)
            hold_report = wait_for_master_alignment(
                targets, since_timestamp=time.time(), publish=publish, shutdown=rospy.is_shutdown,
                timeout=max(0.01, state['deadline'] - time.monotonic()),
                tolerance_mdeg=1000, stable_seconds=hold_seconds,
                initial_stable_since=arrived_at,
                guard=lambda: self._following_preparation_guard(state),
                completion_ready=lambda: self._following_preparation_ready(state))
            self.master_alignment_reports[label + ' hold'] = hold_report
            if not hold_report['passed']:
                rospy.logerr("Parallel final hold/preparation failed; teaching remains blocked: %s", hold_report.get('error'))
                return False
            self._following_preparation_guard(state)
            state['hold_passed'] = True
            rospy.loginfo("Final hold and both prepared follower paths verified; release may now be requested")
            return True
        finally:
            if not state['hold_passed']:
                state['cancel'].set()
            remaining = max(0.0, state['deadline'] - time.monotonic())
            state['thread'].join(timeout=min(0.25, remaining))

    def release_prepared_masters(self):
        gripper_test_side = os.environ.get('DAGGER_TEST_MASTER_GRIPPER_RELEASE', 'both')
        state = getattr(self, 'parallel_following_preparation', None)
        if state is None or not state['hold_passed']:
            raise RuntimeError("The simultaneous final hold and preparation have not both completed")
        # A brief loss of prepared status after the hold must not turn an
        # otherwise healthy FC holding state into an exit. The original shared
        # deadline and guard still reject faults, cancellation and timeout.
        while not self._following_preparation_ready(state):
            time.sleep(0.02)
        # Validate the pair once before the commit. The first arm may report FA
        # immediately; that expected transition must not prevent the second
        # already-prepared arm from receiving the same release request.
        for side in ('left', 'right'):
            self._check_preparation_cancelled(state)
            use_gripper_test = gripper_test_side == 'both' or (side == 'right' and gripper_test_side == 'right')
            request = 'release_master_gripper' if use_gripper_test else 'release_master'
            message = String(data=request)
            getattr(self, 'master_' + side + '_config_pub').publish(message)
    
    def assert_followers_ready(self):
        """Reject teaching requests before both physical follower paths are ready."""
        topics = {side: getattr(self.args, 'puppet_arm_' + side + '_cmd_topic')
                  for side in ('left', 'right')}
        connections = {side: getattr(self, 'puppet_arm_' + side + '_publisher').get_num_connections()
                       for side in ('left', 'right')}
        self.follower_readiness_report = check_followers_ready(topics, connections)

    def probe_followers_for_teaching(self, timeout=3.0, cancel_event=None):
        """Prove both command callbacks work while masters still hold position.

        Only repeat a single snapshot of the followers' actual positions. Stop
        this probe before requesting master preparation so holding probes and native forwarding
        cannot compete for the follower command topics.
        """
        if not math.isfinite(timeout) or timeout <= 0:
            raise ValueError("Invalid follower holding-probe timeout")
        targets = {}
        for side in ('left', 'right'):
            feedback = getattr(self, 'puppet_arm_' + side + '_deque')
            if not feedback:
                raise RuntimeError("No actual follower feedback for holding probe: " + side)
            message = feedback[-1]
            age = rospy.Time.now().to_sec() - message.header.stamp.to_sec()
            if not math.isfinite(age) or not -0.05 <= age <= 0.25:
                raise RuntimeError("Stale actual follower feedback for holding probe: " + side)
            targets[side] = list(message.position)
        alignment_targets(targets)
        monotonic_since, wall_since = time.monotonic(), time.time()
        deadline = monotonic_since + timeout
        self.follower_probe_report = {'passed': False, 'targets': targets,
                                      'started_at': wall_since}
        last_error = 'No follower acknowledgement received'
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            if cancel_event is not None and cancel_event.is_set():
                raise RuntimeError("Follower holding probe was cancelled")
            self.puppet_arm_publish(list(targets['left']), list(targets['right']), require_policy_inputs=False)
            with self.master_handoff_lock:
                heartbeats = dict(self.follower_ready_feedback)
            now, wall_now = time.monotonic(), time.time()
            try:
                if any(heartbeats.get(side, {}).get('received_monotonic', -1) < monotonic_since
                       for side in ('left', 'right')):
                    raise RuntimeError('Both follower heartbeats must postdate this holding probe')
                require_pair_ready(heartbeats, now_wall=wall_now, now_monotonic=now,
                                   minimum_command_time=wall_since)
            except RuntimeError as exc:
                last_error = str(exc)
            else:
                self.follower_probe_report.update(passed=True, completed_at=wall_now)
                return
            time.sleep(0.1)
        raise RuntimeError("Both followers must acknowledge fresh fixed-position commands before master release: "
                           + last_error)

    def switch_masters_to_teach_mode(self):
        """Request the driver-owned teaching transition; readiness is checked separately."""
        self.assert_followers_ready()
        self.probe_followers_for_teaching()
        rospy.loginfo("Requesting master teaching configuration; waiting for both command streams...")

        # The configuration callback enters drag teaching only after completing
        # its transition, then opens master-command publishing. A concurrent
        # teach=1 request would unlock the arms several seconds before followers
        # can receive their commands.
        config_msg = String(data="master")
        self.master_left_config_pub.publish(config_msg)
        self.master_right_config_pub.publish(config_msg)
    
    def move_masters_to_safe_pose(self, duration=3.0):
        """Move master arms to safe intermediate position before matching slave"""
        rospy.loginfo("🛡️ Moving master arms to safe intermediate position...")
        
        # Safe pose (same as inference initial pose)
        safe_left = [0, 0.32, -0.36, 0, 0.24, 0, 0.07]
        safe_right = [0, 0.32, -0.36, 0, 0.24, 0, 0.07]
        
        rospy.loginfo("Safe position: %s", safe_left[:3])
        return self._move_masters_to_pose(safe_left, safe_right, duration, "intermediate pose")
    
    def start_inference_collection(self):
        """Start a fresh inference recording after startup or resuming with r."""
        global process_thread, timesteps, actions, result_container
        if process_thread is not None and process_thread.is_alive():
            raise RuntimeError("Inference collection is already running")
        timesteps, actions = [], []
        result_container = {}
        self.stop_flag = False
        self._inference_unsaved = True
        self._inference_save_path = None

        def _run_process_and_store():
            try:
                result_container['res'] = self.process()
            except Exception as exc:
                # ROS Rate.sleep() raises during a normal shutdown. Keep the
                # already collected prefix, while reporting real capture errors.
                shutdown_interrupt = getattr(rospy, 'ROSInterruptException', ())
                if not (isinstance(exc, shutdown_interrupt) and rospy.is_shutdown()):
                    result_container['error'] = exc
            finally:
                result_container.setdefault('res', (timesteps, actions))

        process_thread = threading.Thread(target=_run_process_and_store, daemon=False)
        process_thread.start()

    def stop_inference_collection(self, timeout=5.0, tail_seconds=0.0):
        """Optionally sample the paused policy's real motion before stopping capture."""
        global process_thread, result_container
        try:
            if tail_seconds > 0 and process_thread is not None:
                deadline = time.monotonic() + tail_seconds
                while (not self.stop_flag and process_thread.is_alive()
                       and not rospy.is_shutdown() and 'error' not in result_container):
                    remaining = deadline - time.monotonic()
                    if remaining <= 0:
                        break
                    time.sleep(min(0.01, remaining))
        finally:
            self.stop_flag = True
        if process_thread is not None:
            process_thread.join(timeout=timeout)
            if process_thread.is_alive():
                raise RuntimeError("Inference collection did not stop within the timeout")
        if 'error' in result_container:
            raise RuntimeError("Inference collection failed") from result_container['error']
        return result_container.get('res', ([], []))

    def save_inference_collection(self, frames, frame_actions):
        """Save a stopped, unsaved inference segment once, without consuming it on failure."""
        global process_thread
        if process_thread is not None and process_thread.is_alive():
            raise RuntimeError("Stop inference collection before saving")
        if not getattr(self, '_inference_unsaved', True):
            return None
        if not frame_actions:
            self._inference_unsaved = False
            print("No unsaved inference frames to save.", flush=True)
            return None

        # Use path from RosOperator.__init__
        base_data_dir = getattr(self.args, "dataset_dir", "/home/agilex/data")
        dataset_base = getattr(self, "data_base_name", None)
        if dataset_base is None:
            # Fallback: args.dataset_name then default
            dataset_base = getattr(self.args, "dataset_name", None) or "flat_1107_27_35_mixed1.0"
            dataset_base = dataset_base.replace("_dagger_hdf5", "").replace("_inference_hdf5", "")

        # Inference save dir
        dataset_dir = os.path.join(base_data_dir, f"{dataset_base}_inference_hdf5", "aloha_mobile_dummy")
        os.makedirs(dataset_dir, exist_ok=True)

        # Find existing episode .hdf5 and increment index
        existing = [f for f in os.listdir(dataset_dir) if f.startswith("episode_") and f.endswith(".hdf5")]
        if len(existing) == 0:
            next_idx = 0
        else:
            try:
                indices = sorted([int(f.split("_")[1].split(".")[0]) for f in existing])
                next_idx = indices[-1] + 1
            except Exception:
                # Fallback to 0 on parse error
                next_idx = len(existing)

        dataset_path = os.path.join(dataset_dir, f"episode_{next_idx}")
        # Reuse this segment's path if Ctrl+C interrupted its earlier export.
        # save_data pops its inputs, so shallow copies keep the real prefix
        # available for the exit save without copying the camera images.
        dataset_path = getattr(self, '_inference_save_path', None) or dataset_path
        self._inference_save_path = dataset_path
        frame_count = len(frame_actions)
        print(f"Saving inference: {frame_count} frames -> {dataset_path}.hdf5", flush=True)
        save_data(self.args, list(frames), list(frame_actions), dataset_path)
        self._inference_unsaved = False
        frames.clear()
        frame_actions.clear()
        print(f"\033[32m>>> Saved inference: {dataset_path}.hdf5 ({frame_count} frames)\033[0m", flush=True)
        return dataset_path

    def stop_and_save_inference(self):
        """Finish only inference recording; never enter teaching or issue robot commands."""
        frames, frame_actions = self.stop_inference_collection()
        return self.save_inference_collection(frames, frame_actions)

    def enter_dagger_mode(self):
        """Align masters, hold and prepare tracking concurrently, then acknowledge teaching."""
        global process_thread, timesteps, actions, result_container
        timesteps, actions = self.stop_inference_collection(tail_seconds=1.0)

        if rospy.is_shutdown():
            return False

        if len(actions) == 0:
            print("\033[31m\nNo data collected, skipping save.\033[0m")
            return False

        self.save_inference_collection(timesteps, actions)
        print("\n" + "=" * 70)
        print("🎯 ENTERING DAGGER MODE")
        print("=" * 70)
        self.stop_flag = True

        # Then enter DAgger flow
        if rospy.is_shutdown():
            return False
        self.start_dagger_preroll()
        ready_at = None
        try:
            self.enable_master_arms()
            rospy.loginfo("\n🛡️ Safety Step: Moving to intermediate position...")
            if not self.move_masters_to_safe_pose(duration=3.0):
                print("\n❌ Failed to move master arms to safe position\n")
                return False

            rospy.loginfo("\n🎯 Final Step: Matching slave positions...")
            if not self.move_masters_to_slave_pose(duration=3.0, prepare_following=True):
                print("\n❌ Failed to move master arms to slave position\n")
                return False

            if rospy.is_shutdown():
                return False
            since = time.monotonic()
            self.release_prepared_masters()
            self.wait_for_master_control(0xFA, True, since, timeout=None)
            ready_at = rospy.Time.now().to_sec()
        finally:
            self.finish_dagger_preroll(ready_at)

        print("\n" + "=" * 70)
        print("✅ DAGGER MODE ACTIVE")
        print("🎮 Drag master arms to control slave arms")
        print("=" * 70 + "\n")
        print(f"Dataset name: {getattr(self, 'dataset_base', self.args.dataset_name)}")
        print("Correction recording starts automatically now; no Space key is needed")
        print("Press 's' during collection to save")
        print("Press 'r' to exit DAgger and resume inference")
        print("=" * 70 + "\n")
        # print(f"dataset")

        return True

    
    
    # def enter_dagger_mode(self):
    #     """Enter DAgger mode: enable master arms, move to safe pose, then slave pose, enable teaching"""
    #     global process_thread, timesteps, actions, result_container
    #     process_thread.join()
    #     timesteps, actions = result_container.get('res', ([], []))

    #     if len(actions) == 0:
    #         print("\033[31m\n未采集到任何数据，放弃保存。\033[0m")
    #         return

    #     dataset_dir = os.path.join(self.args.dataset_dir, self.args.dataset_name, self.args.task_name)
    #     os.makedirs(dataset_dir, exist_ok=True)
    #     existing = [
    #         f for f in os.listdir(dataset_dir)
    #         if f.startswith("episode_") and f.endswith(".hdf5")
    #     ]
    #     if existing:
    #         indices = [int(f.split("_")[1].split(".")[0]) for f in existing]
    #         next_idx = max(indices) + 1
    #     else:
    #         next_idx = 0

    #     dataset_path = os.path.join(dataset_dir, f"episode_{next_idx}")
    #     # dataset_path = os.path.join(dataset_dir, f"episode_{self.args.episode_idx}")
    #     save_data(self.args, timesteps, actions, dataset_path)
    #     print("\033[32m>>> 已保存至：", dataset_path + ".hdf5\033[0m")
    #     print("\n" + "=" * 70)
    #     print("🎯 ENTERING DAGGER MODE")
    #     print("=" * 70)
    #     self.stop_flag = True
        
    #     # Step 1: Enable master arms
    #     self.enable_master_arms()
        
    #     # 🆕 Step 2: Move master arms to safe intermediate position first
    #     rospy.loginfo("\n🛡️ Safety Step: Moving to intermediate position...")
    #     if not self.move_masters_to_safe_pose(duration=3.0):
    #         print("\n❌ Failed to move master arms to safe position\n")
    #         return False
        
    #     # Step 3: Move master arms to slave positions
    #     rospy.loginfo("\n🎯 Final Step: Matching slave positions...")
    #     if not self.move_masters_to_slave_pose(duration=3.0):
    #         print("\n❌ Failed to move master arms to slave position\n")
    #         return False
        
    #     # Step 4: Switch to teaching mode
    #     self.switch_masters_to_teach_mode()
        
    #     # 🆕 Step 5: 等待用户按空格键开始采集（不自动开始）
    #     print("\n" + "=" * 70)
    #     print("✅ DAGGER MODE ACTIVE")
    #     print("🎮 Drag master arms to control slave arms")
    #     print("=" * 70 + "\n")
    #     print(f"📁 数据集名称: {self.data_collector.task_name}")
    #     print("📋 按【空格键】开始采集数据")
    #     print("📋 采集过程中按【s】键保存数据")
    #     print("📋 按【r】键退出DAgger模式，返回推理")
    #     print("=" * 70 + "\n")
        
    #     return True

    def init_ros(self):
        rospy.init_node("joint_state_publisher", anonymous=True)
        rospy.Subscriber(
            self.args.img_left_topic,
            Image,
            self.img_left_callback,
            queue_size=1000,
            tcp_nodelay=True,
        )
        rospy.Subscriber(
            self.args.img_right_topic,
            Image,
            self.img_right_callback,
            queue_size=1000,
            tcp_nodelay=True,
        )
        rospy.Subscriber(
            self.args.img_front_topic,
            Image,
            self.img_front_callback,
            queue_size=1000,
            tcp_nodelay=True,
        )
        if self.args.use_depth_image:
            rospy.Subscriber(
                self.args.img_left_depth_topic,
                Image,
                self.img_left_depth_callback,
                queue_size=1000,
                tcp_nodelay=True,
            )
            rospy.Subscriber(
                self.args.img_right_depth_topic,
                Image,
                self.img_right_depth_callback,
                queue_size=1000,
                tcp_nodelay=True,
            )
            rospy.Subscriber(
                self.args.img_front_depth_topic,
                Image,
                self.img_front_depth_callback,
                queue_size=1000,
                tcp_nodelay=True,
            )
        rospy.Subscriber(
            self.args.puppet_arm_left_topic,
            JointState,
            self.puppet_arm_left_callback,
            queue_size=1000,
            tcp_nodelay=True,
        )
        rospy.Subscriber(
            self.args.puppet_arm_right_topic,
            JointState,
            self.puppet_arm_right_callback,
            queue_size=1000,
            tcp_nodelay=True,
        )
        rospy.Subscriber(
            self.args.robot_base_topic,
            Odometry,
            self.robot_base_callback,
            queue_size=1000,
            tcp_nodelay=True,
        )
        self.puppet_arm_left_publisher = rospy.Publisher(self.args.puppet_arm_left_cmd_topic, JointState, queue_size=10)
        self.puppet_arm_right_publisher = rospy.Publisher(
            self.args.puppet_arm_right_cmd_topic, JointState, queue_size=10
        )
        self.endpose_left_publisher = rospy.Publisher(self.args.endpose_left_cmd_topic, PosCmd, queue_size=10)
        self.endpose_right_publisher = rospy.Publisher(self.args.endpose_right_cmd_topic, PosCmd, queue_size=10)
        self.robot_base_publisher = rospy.Publisher(self.args.robot_base_cmd_topic, Twist, queue_size=10)
        
        # DAgger mode - master arm subscribers and publishers
        for side in ('left', 'right'):
            rospy.Subscriber('/teach/follower_ready_' + side, Twist,
                             self.follower_ready_callback, callback_args=side, queue_size=1,
                             tcp_nodelay=True)
            rospy.Subscriber('/teach/master_control_status_' + side, Twist,
                             self.master_control_status_callback, callback_args=side, queue_size=1)
            rospy.Subscriber(getattr(self.args, 'puppet_arm_' + side + '_cmd_topic'), JointState,
                             self.master_command_callback, callback_args=side, queue_size=1, tcp_nodelay=True)
        rospy.Subscriber(
            "/puppet_master/joint_left",
            JointState,
            self.master_arm_left_callback,
            queue_size=1000,
            tcp_nodelay=True,
        )
        rospy.Subscriber(
            "/puppet_master/joint_right",
            JointState,
            self.master_arm_right_callback,
            queue_size=1000,
            tcp_nodelay=True,
        )
        
        self.master_left_enable_pub = rospy.Publisher('/teach/master_enable_left', Bool, queue_size=10)
        self.master_right_enable_pub = rospy.Publisher('/teach/master_enable_right', Bool, queue_size=10)
        self.master_left_config_pub = rospy.Publisher('/teach/master_config_left', String, queue_size=10)
        self.master_right_config_pub = rospy.Publisher('/teach/master_config_right', String, queue_size=10)
        self.master_left_teach_mode_pub = rospy.Publisher('/teach/teach_mode_left', Int32, queue_size=10)
        self.master_right_teach_mode_pub = rospy.Publisher('/teach/teach_mode_right', Int32, queue_size=10)
        self.master_left_joint_pub = rospy.Publisher('/master_controled/joint_left', JointState, queue_size=10)
        self.master_right_joint_pub = rospy.Publisher('/master_controled/joint_right', JointState, queue_size=10)

    def process(self):
        global timesteps, actions
        count = 0
        rate = rospy.Rate(self.args.frame_rate)
        print("\033[36m>>> Inference collection started. Press d to stop and enter DAgger.\033[0m")

        # --- 数据检测变量 (左右臂分离) ---
        last_qpos_left, last_qpos_right = None, None
        consecutive_unchanged_count_left, consecutive_unchanged_count_right = 0, 0
        UNCHANGED_THRESHOLD = 100 # 连续100帧无变化则发出警告

        while (count < self.args.max_timesteps + 1) and not rospy.is_shutdown() and not self.stop_flag:
            result = self.get_frame()
            if not result:
                rate.sleep()
                continue
            count += 1
            (img_front, img_left, img_right, img_front_depth, img_left_depth, img_right_depth,
             puppet_arm_left, puppet_arm_right, robot_base) = result

            image_dict = {self.args.camera_names[0]: img_front,
                          self.args.camera_names[1]: img_left,
                          self.args.camera_names[2]: img_right}
            obs = collections.OrderedDict()
            obs['images'] = image_dict
            if self.args.use_depth_image:
                obs['images_depth'] = {self.args.camera_names[0]: img_front_depth,
                                       self.args.camera_names[1]: img_left_depth,
                                       self.args.camera_names[2]: img_right_depth}
            obs['qpos'] = np.concatenate((puppet_arm_left.position, puppet_arm_right.position))
            obs['qvel'] = np.concatenate((puppet_arm_left.velocity, puppet_arm_right.velocity))
            obs['effort'] = np.concatenate((puppet_arm_left.effort, puppet_arm_right.effort))
            obs['base_vel'] = [robot_base.twist.twist.linear.x, robot_base.twist.twist.angular.z] if self.args.use_robot_base else [0.0, 0.0]

            # --- 分别检测左右臂的 qpos 数据是否连续不变 ---
            # 左臂
            current_qpos_left = puppet_arm_left.position
            if last_qpos_left is not None and np.array_equal(current_qpos_left, last_qpos_left):
                consecutive_unchanged_count_left += 1
            else:
                consecutive_unchanged_count_left = 0
            if consecutive_unchanged_count_left >= UNCHANGED_THRESHOLD:
                print(f"\033[33mWarning: left arm position unchanged for {consecutive_unchanged_count_left} frames.\033[0m")
            last_qpos_left = current_qpos_left

            # 右臂
            current_qpos_right = puppet_arm_right.position
            if last_qpos_right is not None and np.array_equal(current_qpos_right, last_qpos_right):
                consecutive_unchanged_count_right += 1
            else:
                consecutive_unchanged_count_right = 0
            if consecutive_unchanged_count_right >= UNCHANGED_THRESHOLD:
                print(f"\033[33mWarning: right arm position unchanged for {consecutive_unchanged_count_right} frames.\033[0m")
            last_qpos_right = current_qpos_right
            # --- 检测结束 ---

            if count == 1:
                timesteps.append(dm_env.TimeStep(dm_env.StepType.FIRST, None, None, obs))
                continue
            timesteps.append(dm_env.TimeStep(dm_env.StepType.MID, None, None, obs))
            left_action = puppet_arm_left.position[:7]
            right_action = puppet_arm_right.position[:7]
            # left_action = np.concatenate((puppet_arm_left.position[:6], [master_arm_left.position[6]]))
            # right_action = np.concatenate((puppet_arm_right.position[:6], [master_arm_right.position[6]]))
            actions.append(np.concatenate((left_action, right_action)))
            print("Frame data: ", count)
            rate.sleep()

        print(f"\n>>> Inference capture ended, {len(actions)} frames ready for saving")
        return timesteps, actions

def get_arguments():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--max_publish_step",
        action="store",
        type=int,
        help="Maximum number of action publishing steps",
        default=10000,
        required=False,
    )
    parser.add_argument(
        "--seed",
        action="store",
        type=int,
        help="Random seed",
        default=None,
        required=False,
    )
    parser.add_argument(
        "--img_front_topic",
        action="store",
        type=str,
        help="img_front_topic",
        default="/camera_f/color/image_raw",
        required=False,
    )
    parser.add_argument(
        "--img_left_topic",
        action="store",
        type=str,
        help="img_left_topic",
        default="/camera_l/color/image_raw",
        required=False,
    )
    parser.add_argument(
        "--img_right_topic",
        action="store",
        type=str,
        help="img_right_topic",
        default="/camera_r/color/image_raw",
        required=False,
    )
    parser.add_argument(
        "--img_front_depth_topic",
        action="store",
        type=str,
        help="img_front_depth_topic",
        default="/camera_f/depth/image_raw",
        required=False,
    )
    parser.add_argument(
        "--img_left_depth_topic",
        action="store",
        type=str,
        help="img_left_depth_topic",
        default="/camera_l/depth/image_raw",
        required=False,
    )
    parser.add_argument(
        "--img_right_depth_topic",
        action="store",
        type=str,
        help="img_right_depth_topic",
        default="/camera_r/depth/image_raw",
        required=False,
    )
    parser.add_argument(
        "--puppet_arm_left_cmd_topic",
        action="store",
        type=str,
        help="puppet_arm_left_cmd_topic",
        default="/master/joint_left",
        required=False,
    )
    parser.add_argument(
        "--puppet_arm_right_cmd_topic",
        action="store",
        type=str,
        help="puppet_arm_right_cmd_topic",
        default="/master/joint_right",
        required=False,
    )
    parser.add_argument(
        "--puppet_arm_left_topic",
        action="store",
        type=str,
        help="puppet_arm_left_topic",
        default="/puppet/joint_left",
        required=False,
    )
    parser.add_argument(
        "--puppet_arm_right_topic",
        action="store",
        type=str,
        help="puppet_arm_right_topic",
        default="/puppet/joint_right",
        required=False,
    )
    parser.add_argument('--master_arm_left_topic', default='/master/joint_left')

    parser.add_argument('--master_arm_right_topic', default='/master/joint_right')

    parser.add_argument(
        "--endpose_left_cmd_topic",
        action="store",
        type=str,
        help="endpose_left_cmd_topic",
        default="/pos_cmd_left",
        required=False,
    )
    parser.add_argument(
        "--endpose_right_cmd_topic",
        action="store",
        type=str,
        help="endpose_right_cmd_topic",
        default="/pos_cmd_right",
        required=False,
    )

    parser.add_argument(
        "--robot_base_topic",
        action="store",
        type=str,
        help="robot_base_topic",
        default="/odom_raw",
        required=False,
    )
    parser.add_argument(
        "--robot_base_cmd_topic",
        action="store",
        type=str,
        help="robot_base_topic",
        default="/cmd_vel",
        required=False,
    )
    parser.add_argument(
        "--use_robot_base",
        action="store_true",
        help="Whether to use the robot base to move around",
        default=False,
        required=False,
    )
    parser.add_argument(
        "--publish_rate",
        action="store",
        type=int,
        help="The rate at which to publish the actions",
        default=30,
        required=False,
    )
    # smoothing related args
    parser.add_argument(
        "--use_temporal_smoothing",
        action="store_true",
        help="Enable non-blocking temporal smoothing execution",
        default=False,
        required=False,
    )
    parser.add_argument(
        "--latency_k",
        type=int,
        help="Max latency steps to drop from new chunks",
        default=8,
        required=False,
    )
    parser.add_argument(
        "--inference_rate",
        type=float,
        help="Inference loop rate (Hz) for background thread",
        default=3.0,
        required=False,
    )
    parser.add_argument(
        "--min_smooth_steps",
        type=int,
        help="Minimum smoothing steps for overlap",
        default=8,
        required=False,
    )
    parser.add_argument(
        "--buffer_max_chunks",
        type=int,
        help="Maximum number of chunks in buffer",
        default=10,
        required=False,
    )
    parser.add_argument(
        "--exp_decay_alpha",
        type=float,
        help="Exponential decay alpha (reserved)",
        default=0.25,
        required=False,
    )
    parser.add_argument(
        "--gripper_threshold",
        action="store_true",
        help="Whether to use gripper thresholding",
        default=True,
        required=False,
    )
    parser.add_argument(
        "--chunk_size",
        action="store",
        type=int,
        help="Action chunk size",
        default=50,
        required=False,
    )
    parser.add_argument(
        "--arm_steps_length",
        action="store",
        type=float,
        help="The maximum change allowed for each joint per timestep",
        default=[0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.2],
        required=False,
    )
    parser.add_argument(
        "--use_actions_interpolation",
        action="store_true",
        help="Whether to interpolate the actions if the difference is too large",
        default=False,
        required=False,
    )
    parser.add_argument(
        "--use_kalman_filter",
        action="store_true",
        help="Whether to use Kalman filter for action smoothing",
        default=True,
        required=False,
    )
    parser.add_argument(
        "--interpolate_method",
        type=str,
        choices=["linear", "minimum_jerk"],
        help="Interpolation method for action smoothing",
        default="linear",
    )
    parser.add_argument(
        "--jerk_num_steps",
        action="store",
        type=int,
        help="Number of steps for minimum jerk interpolation",
        default=10,
        required=False,
    )
    parser.add_argument(
        "--use_depth_image",
        action="store_true",
        help="Whether to use depth images",
        default=False,
        required=False,
    )
    # Video export options (align with collect_data)
    parser.add_argument(
        "--export_video",
        action="store_true",
        help="Export RGB videos for each camera when saving DAgger episodes",
        default=True,
        required=False,
    )
    parser.add_argument(
        "--video_fps",
        action="store",
        type=int,
        help="FPS for exported videos",
        default=30,
        required=False,
    )
    parser.add_argument(
        "--video_codec",
        action="store",
        type=str,
        choices=["libx264", "libx265", "libsvtav1"],
        help="Codec for exported videos",
        default="libx264",
        required=False,
    )
    parser.add_argument(
        "--video_quality",
        action="store",
        type=int,
        help="CRF quality for exported videos (lower is higher quality)",
        default=23,
        required=False,
    )
    parser.add_argument(
        "--host",
        action="store",
        type=str,
        help="Policy server host IP (e.g. 192.168.1.10)",
        default="192.168.1.10",
        required=False,
    )
    parser.add_argument(
        "--port",
        action="store",
        type=int,
        help="Websocket server port",
        default=8000,
        required=False,
    )

    parser.add_argument(
        "--ctrl_type",
        type=str,
        choices=["joint", "eef"],
        help="Control type for the robot arm",
        default="joint",
    )
    parser.add_argument(
        "--dataset_name",
        type=str,
        help="Dataset name for DAgger data (e.g. flat_1101_27_100_mixed1.0)",
        default="dagger",
        required=False,
    )
    parser.add_argument('--dataset_dir', type=str, default="/home/agilex/data")
    parser.add_argument('--task_name', type=str, default="aloha_mobile_dummy")
    parser.add_argument('--episode_idx', type=int, default=0)
    parser.add_argument('--max_timesteps', type=int, default=100000000)
    parser.add_argument('--frame_rate', type=int, default=30)
    parser.add_argument('--camera_names', nargs='+', default=['cam_high', 'cam_left_wrist', 'cam_right_wrist'])


    args = parser.parse_args()
    return args


def main():
    global process_thread, result_container
    args = get_arguments()
    # collect_operator = CollectOperator(args)
    ros_operator = RosOperator(args)
    if args.seed is not None:
        set_seed(args.seed)
    config = get_config(args)
    
    # Run ros_operator.process() in a background thread so model_inference can run concurrently.
    # We run model_inference in the main thread so input()/keyboard handling works as intended.
    
    ros_operator.start_inference_collection()

    # The Bash owner stops arm forwarding before signalling this collector.
    # Finish the already sampled inference prefix even after ROS shutdown or
    # KeyboardInterrupt; no new observations or control commands are requested.
    try:
        model_inference(args, config, ros_operator)
    finally:
        ros_operator.stop_and_save_inference()

if __name__ == "__main__":
    main()

    
