#!/usr/bin/env python3
# -*-coding:utf8-*-
# 本文件为同时打开主从臂的节点，通过mode参数控制是读取还是控制
# 默认认为从臂有夹爪
# mode为0时为发送主从臂消息，
# mode为1时为控制从臂，不发送主臂消息，此时如果要控制从臂，需要给主臂的topic发送消息
from typing import (
    Optional,
)
import rospy
import rosnode
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String, Int32
import time
import threading
import argparse
import math
from pathlib import Path
import sys
from piper_sdk import *
from piper_sdk import C_PiperInterface_V3
from piper_msgs.msg import PiperStatusMsg, PosCmd
from geometry_msgs.msg import Pose,PoseStamped, Twist
from std_srvs.srv import Trigger, TriggerResponse
from tf.transformations import quaternion_from_euler  # 用于欧拉角到四元数的转换

sys.path.insert(0, str(Path(__file__).resolve().parents[4] / "setup"))
from prepare_follower_mode import sample_feedback
from prepare_follower_enabled import checked_enabled_feedback
from teaching_readiness import follower_readiness, require_pair_ready
from master_native_forwarding import NativeMasterReader


def check_ros_master():
    try:
        rosnode.rosnode_ping('rosout', max_count=1, verbose=False)
        rospy.loginfo("ROS Master is running.")
    except rosnode.ROSNodeIOException:
        rospy.logerr("ROS Master is not running.")
        raise RuntimeError("ROS Master is not running.")

class C_PiperRosNode():
    """机械臂ros节点
    """
    def __init__(self) -> None:
        check_ros_master()
        rospy.init_node('piper_start_all_node', anonymous=True)

        self.can_port = "can0"
        if rospy.has_param('~can_port'):
            self.can_port = rospy.get_param("~can_port")
            rospy.loginfo("%s is %s", rospy.resolve_name('~can_port'), self.can_port)
        else: 
            rospy.loginfo("未找到can_port参数,请输入 _can_port:=can0 类似的格式")
            exit(0)
        
        # 模式，模式为1的时候，才能够控制从臂
        self.mode = 0
        if rospy.has_param('~mode'):
            self.mode = rospy.get_param("~mode")
            rospy.loginfo("%s is %s", rospy.resolve_name('~mode'), self.mode)
        else:
            rospy.loginfo("未找到mode参数,请输入 _mode:=0 类似的格式")
            exit(0)

        # 🆕 新增：mode_master参数，只对mode=0有效，控制主臂的起始状态
        self.mode_master = True  # 默认主臂模式
        if rospy.has_param('~mode_master') and self.mode == 0:
            self.mode_master = rospy.get_param("~mode_master")
            rospy.loginfo("%s is %s", rospy.resolve_name('~mode_master'), self.mode_master)

        # Deployment-only teaching interlock. Generic launches keep the legacy
        # default; the wj-dev launch explicitly enables it for all four nodes.
        self.require_follower_ready = rospy.get_param('~require_follower_ready', False)
        self.follower_ready_timeout_s = float(rospy.get_param('~follower_ready_timeout_s', 5.0))
        if (type(self.require_follower_ready) is not bool
                or not math.isfinite(self.follower_ready_timeout_s) or self.follower_ready_timeout_s <= 0):
            raise RuntimeError("Invalid follower teaching interlock parameters")
        if self.mode == 0 and self.require_follower_ready and self.mode_master:
            raise RuntimeError("Interlocked masters must start in FC and receive a verified alignment target before teaching")
        # Heartbeats use their own short lock: they must remain receivable while
        # the mode-transition lock is held during the existing six-second wait.
        self._readiness_lock = threading.Lock()
        self._mode_transition_lock = threading.RLock()
        self._torque_write_lock = threading.Lock()
        self._manual_disable_requested = threading.Event()
        self._follower_ready_rows = {}
        self._last_publish_loop_monotonic = 0.0
        self._last_joint_command_wall = 0.0
        self._last_joint_command_valid = False
        self._joint_subscription_ready = False
        self._last_alignment_target = None
        self._last_alignment_command_gripper_um = None
        self._transition_in_progress = False
        self._master_publish_enabled = False
        self._native_forward_lock = threading.Lock()
        self._native_forward_phase = None
        self._native_master_reader = None
        self._native_last_actual = None
        self.native_handoff_report = None
        
        # 🆕 新增：当前主从状态 (只对mode=0有效)
        if self.mode == 0:
            self.current_linkage_config = 0xFA if self.mode_master else 0xFC  # 0xFA=主臂示教, 0xFC=从臂跟随
            self.in_teach_mode = self.mode_master  # 主臂模式默认进入示教
            rospy.loginfo(f"mode=0 初始配置: {'主臂示教模式(0xFA)' if self.mode_master else '从臂跟随模式(0xFC)'}")
        else:
            # mode=1 固定为从臂，不需要切换
            rospy.loginfo("mode=1 固定为从臂模式，不支持主从切换")

        # 是否自动使能，默认不自动使能，只有模式为1的时候才能够被设置为自动使能
        self.auto_enable = False
        if rospy.has_param('~auto_enable'):
            if(rospy.get_param("~auto_enable") and self.mode == 1):
                self.auto_enable = True
        rospy.loginfo("%s is %s", rospy.resolve_name('~auto_enable'), self.auto_enable)
        self.gripper_exist = True

        # 🆕 新增：状态变量
        self.is_enabled = False
        self.new_config_ = None

        # publish
        self.joint_std_pub_puppet = rospy.Publisher('/puppet/joint_states', JointState, queue_size=1, tcp_nodelay=True)
        # 默认模式为0，读取主从臂消息
        if(self.mode == 0):
            self.joint_std_pub_master = rospy.Publisher('/master/joint_states', JointState, queue_size=1, tcp_nodelay=True)
            # 🆕 新增：mode=0时的状态发布器
            self.master_status_pub = rospy.Publisher('/master/arm_status', Twist, queue_size=1)
            self.mode_status_pub = rospy.Publisher('/master/mode_status', String, queue_size=1)
            
        self.arm_status_pub = rospy.Publisher('/puppet/arm_status', PiperStatusMsg, queue_size=1, tcp_nodelay=True)
        self.end_pose_pub = rospy.Publisher('/puppet/end_pose', PoseStamped, queue_size=1, tcp_nodelay=True)
        self.end_pose_euler_pub = rospy.Publisher('/puppet/end_pose_euler', PosCmd, queue_size=1, tcp_nodelay=True)
        
        self.__enable_flag = False
        # 从臂消息
        self.joint_state_slave = JointState()
        self.joint_state_slave.name = ['joint0', 'joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        self.joint_state_slave.position = [0.0] * 7
        self.joint_state_slave.velocity = [0.0] * 7
        self.joint_state_slave.effort = [0.0] * 7
        # 主臂消息
        self.joint_state_master = JointState()
        self.joint_state_master.name = ['joint0', 'joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        self.joint_state_master.position = [0.0] * 7
        self.joint_state_master.velocity = [0.0] * 7
        self.joint_state_master.effort = [0.0] * 7

        self.piper = C_PiperInterface_V3(can_name=self.can_port)
        self.piper.ConnectPort()

        if self.mode == 0 and self.require_follower_ready:
            self._follower_ready_subscribers = [
                rospy.Subscriber('/teach/follower_ready_' + side, Twist,
                                 self.follower_ready_callback, callback_args=side,
                                 queue_size=1, tcp_nodelay=True)
                for side in ('left', 'right')]
        
        # 🆕 新增：mode=0时初始化机械臂配置
        if self.mode == 0:
            self._initialize_master_arm_config()
            if self.require_follower_ready:
                self._native_master_reader = NativeMasterReader(self.can_port, shutdown=rospy.is_shutdown)
                self._native_master_reader.start()
                self._native_master_reader.wait_started(timeout=1.0)
                self._native_forward_thread = threading.Thread(
                    target=self.MasterNativeForwardingThread, daemon=True)
                self._native_forward_thread.start()

        # service
        str_can_port = str(self.can_port)
        # 主臂单独回零
        self.master_go_zero_service = rospy.Service('/'+str_can_port+'/go_zero_master', Trigger, self.handle_master_go_zero_service)
        # 主从臂一起回零
        self.master_go_zero_service = rospy.Service('/'+str_can_port+'/go_zero_master_slave', Trigger, self.handle_master_slave_go_zero_service)
        # Restoring the master and slave
        self.restore_ms_mode_service = rospy.Service('/'+str_can_port+'/restore_ms_mode', Trigger, self.handle_restore_ms_mode_service)
        
        # 模式为1的时候，订阅控制消息
        if(self.mode == 1):
            sub_pos_th = threading.Thread(target=self.SubPosThread)
            sub_joint_th = threading.Thread(target=self.SubJointThread)
            sub_enable_th = threading.Thread(target=self.SubEnableThread)
            
            sub_pos_th.daemon = True
            sub_joint_th.daemon = True
            sub_enable_th.daemon = True
            
            sub_pos_th.start()
            sub_joint_th.start()
            sub_enable_th.start()
            if self.require_follower_ready:
                side = {'can_left_slave': 'left', 'can_right_slave': 'right'}.get(self.can_port)
                if side is None:
                    raise RuntimeError("Interlocked follower must use an explicitly mapped slave CAN port")
                self._follower_ready_publisher = rospy.Publisher(
                    '/teach/follower_ready_' + side, Twist, queue_size=1, tcp_nodelay=True)
                monitor = threading.Thread(target=self.FollowerReadinessThread, daemon=True)
                monitor.start()

        # 🆕 新增：mode=0时启动主从模式控制订阅器
        if(self.mode == 0):

            sub_joint_th = threading.Thread(target=self.SubJointThread)
            sub_joint_th.daemon = True
            sub_joint_th.start()


            master_control_th = threading.Thread(target=self.SubMasterControlThread)
            master_control_th.daemon = True
            master_control_th.start()

    # 🆕 新增：初始化主臂配置 (mode=0专用)
    def _initialize_master_arm_config(self):
        """初始化主臂配置：根据mode_master参数设置初始状态"""
        if self.require_follower_ready and not self.mode_master:
            # The managed preparation has already restored native CAN1 holding
            # control. Repeating an FC role command here can re-enter standby
            # and disrupt that hold; validate it without another control write.
            since = time.time()
            feedback = sample_feedback(self.can_port, since_timestamp=since)
            checked_enabled_feedback(feedback, now=time.time(), since_timestamp=since)
            if feedback['ctrl_mode'] != 1:
                raise RuntimeError("Prepared master must remain in enabled CAN1 position control")
            self.is_enabled = True
            self.__enable_flag = True
            self.current_linkage_config = 0xFC
            self.new_config_ = 0xFC
            self.in_teach_mode = False
            self._master_publish_enabled = False
            # A raw startup pose is not a verified DAgger alignment command.
            self._last_alignment_target = None
            rospy.loginfo("Prepared master CAN1 holding state verified; no startup control command sent")
            return
        try:
            rospy.loginfo("🔧 正在初始化主臂配置...")
            
            # 根据mode_master参数设置初始配置
            rospy.loginfo(f"📍 设置初始模式: {'主臂示教模式' if self.mode_master else '从臂跟随模式'}")
            self._checked_sdk_write('MasterSlaveConfig', 
                linkage_config=self.current_linkage_config,
                feedback_offset=0x00,
                ctrl_offset=0x00,
                linkage_offset=0x00
            )
            time.sleep(2)
            
            # 初始化使能
            rospy.loginfo("📍 初始化使能状态")
            self._checked_sdk_write('EnableArm', 7)
            time.sleep(2)
            self.is_enabled = True
            self.__enable_flag = True
            
            if self.mode_master:
                # 主臂模式：进入示教模式
                rospy.loginfo("📍 主臂模式：进入拖动示教模式")
                self._checked_sdk_write('MotionCtrl_1', grag_teach_ctrl=0x01)  # 进入拖动示教模式
                self._master_publish_enabled = True
            else:
                # 从臂模式：设置CAN控制模式
                rospy.loginfo("📍 从臂模式：设置CAN控制模式")
                self._checked_sdk_write('MotionCtrl_2', 
                    ctrl_mode=0x01,     # CAN控制模式
                    move_mode=0x01,     # MOVE J
                    move_spd_rate_ctrl=30  # 30%速度
                )
                # 确保退出示教模式
                self._checked_sdk_write('MotionCtrl_1', grag_teach_ctrl=0x02)  # 退出拖动示教模式
            
            rospy.loginfo("✅ 主臂配置初始化完成")
            
        except Exception as e:
            rospy.logerr(f"❌ 主臂配置初始化失败: {e}")

    # 🆕 新增：主从模式控制订阅线程 (mode=0专用)
    def SubMasterControlThread(self):
        """主从模式控制订阅线程"""
        # 在线切换主从配置订阅器
        rospy.Subscriber('/master/linkage_config', String, self.linkage_config_callback, queue_size=1)
        # 示教模式切换订阅器
        rospy.Subscriber('/master/teach_mode', Int32, self.teach_mode_callback, queue_size=1)
        # 使能控制订阅器
        rospy.Subscriber('/master/enable', Bool, self.master_enable_callback, queue_size=1)
        rospy.spin()

    # 🆕 新增：在线切换主从配置回调 (mode=0专用)
    def linkage_config_callback(self, msg):
        # Separate ROS subscriptions run on different callback threads. Do not
        # allow a direct teach request to overtake this configuration sequence.
        with self._mode_transition_lock:
            self._set_linkage_config(msg)

    def _set_linkage_config(self, msg):
        """在线切换主从配置回调"""
        if self.mode != 0:
            rospy.logwarn("只有mode=0的节点才支持主从配置切换")
            return
            
        config_str = msg.data.lower().strip()
        if config_str in ('pause_following', 'resume_following'):
            with self._native_forward_lock:
                phase = self._native_forward_phase
                if phase is not None:
                    phase['following_enabled'] = config_str == 'resume_following'
            return
        if self.require_follower_ready and config_str in ('prepare_master', 'release_master',
                                                         'release_master_gripper'):
            if config_str == 'prepare_master':
                self._prepare_native_master_mode()
            elif config_str == 'release_master_gripper':
                self._release_native_master_mode(release_gripper=True)
            else:
                self._release_native_master_mode()
            return
        
        # 支持的配置
        config_map = {
            "master": 0xFA,    # 主臂示教模式
            "slave": 0xFC,     # 从臂跟随模式
            "0xfa": 0xFA,
            "0xfc": 0xFC,
            "fa": 0xFA,
            "fc": 0xFC
        }
        
        if config_str not in config_map:
            rospy.logwarn(f"❌ 不支持的配置: {config_str}")
            rospy.logwarn("   支持的配置: master/0xFA (主臂示教), slave/0xFC (从臂跟随)")
            return
            
        new_config = config_map[config_str]
        
        if new_config == self.current_linkage_config:
            rospy.loginfo(f"📍 当前已经是配置 0x{new_config:02X}")
            return

        if self.require_follower_ready:
            if new_config == 0xFA:
                self._enter_native_master_mode()
            else:
                self._transition_in_progress = True
                self._stop_native_forwarding()
                try:
                    self._return_to_current_native_hold()
                    self._publish_mode_status()
                except Exception as exc:
                    self._mark_native_mode_unknown()
                    rospy.logerr("Native FC holding recovery failed: %s", exc)
                finally:
                    self._transition_in_progress = False
                    if self._manual_disable_requested.is_set():
                        self._mark_manually_disabled_state()
            return

        fa_attempted = False
        try:
            rospy.loginfo(f"🔄 切换主从配置: 0x{self.current_linkage_config:02X} -> 0x{new_config:02X}")
            self._transition_in_progress = True
            self._master_publish_enabled = False
            self.in_teach_mode = False
            if new_config == 0xFA:
                self._wait_for_follower_readiness()
            
            # 记录当前使能状态
            was_enabled = self.is_enabled
            
            # 发送新配置
            fa_attempted = new_config == 0xFA
            self._checked_sdk_write('MasterSlaveConfig', 
                linkage_config=new_config,
                feedback_offset=0x00,
                ctrl_offset=0x00,
                linkage_offset=0x00
            )
            
            time.sleep(2)  # 等待配置生效
            
            # 如果之前是使能状态，确保切换后也是使能的
            if was_enabled:
                rospy.loginfo("📍 确保切换后保持使能状态...")
                self._checked_sdk_write('EnableArm', 7)
                time.sleep(2)
                self.is_enabled = True
                self.__enable_flag = True
            
            # 根据新配置初始化相应模式
            if new_config == 0xFA:
                # 主臂示教模式
                self._init_master_teach_mode()
            elif new_config == 0xFC:
                # 从臂跟随模式
                self._init_slave_follow_mode()
            
            self.current_linkage_config = new_config
            rospy.loginfo(f"✅ 主从配置切换成功: 0x{new_config:02X}")
            
            # 发布状态更新·
            self._publish_mode_status()

            self.new_config_ = new_config  # 更新全局变量
        except Exception as e:
            self._master_publish_enabled = False
            self.in_teach_mode = False
            if self.require_follower_ready and not self._manual_disable_requested.is_set():
                try:
                    if fa_attempted:
                        self.current_linkage_config = 0x00
                        self.new_config_ = None
                        self._return_to_alignment_hold()
                    else:
                        self._hold_last_alignment_pose()
                except Exception as hold_exc:
                    rospy.logerr(f"主臂保持目标不可用: {hold_exc}")
            rospy.logerr(f"❌ 主从配置切换失败: {e}")
        finally:
            self._transition_in_progress = False
            if self._manual_disable_requested.is_set():
                self._mark_manually_disabled_state()

    # 🆕 新增：初始化主臂示教模式
    def _init_master_teach_mode(self):
        """初始化主臂示教模式 (0xFA)"""
        if self.require_follower_ready:
            raise RuntimeError("Managed FA release must use native priming and forwarding")
        rospy.loginfo("🎯 初始化主臂示教模式 (0xFA)")
        if not self.is_enabled:
            raise RuntimeError("Master must already be enabled before teaching")
        if self.is_enabled:
            # 重新使能以确保状态正确
            rospy.loginfo("📍 重新使能机械臂...")
            self._checked_sdk_write('EnableArm', 7)
            time.sleep(2)
            # The original configuration/enable waits stay intact. Readiness
            # must still be current immediately before torque-control release.
            self._wait_for_follower_readiness(timeout=0.0)
            # 进入拖动示教模式
            self._checked_sdk_write('MotionCtrl_1', grag_teach_ctrl=0x01)  # 进入拖动示教模式
            self.in_teach_mode = True
            self._master_publish_enabled = True
            self.new_config_ = 0xFA
            rospy.loginfo("📍 主臂示教模式：已进入拖动示教模式，可以手动拖拽")

    # 🆕 新增：初始化从臂跟随模式
    def _init_slave_follow_mode(self):
        """初始化从臂跟随模式 (0xFC)"""
        rospy.loginfo("🎯 初始化从臂跟随模式 (0xFC)")
        self._master_publish_enabled = False
        
        if self.is_enabled:
            # 从臂模式需要退出拖动示教模式
            self._checked_sdk_write('MotionCtrl_1', grag_teach_ctrl=0x02)  # 退出拖动示教模式
            self.in_teach_mode = False
            
            time.sleep(1)
            
            # 重新使能以确保状态正确
            rospy.loginfo("📍 重新使能机械臂...")
            self._checked_sdk_write('EnableArm', 7)
            time.sleep(2)
            
            # 设置为CAN控制模式
            self._checked_sdk_write('MotionCtrl_2', 
                ctrl_mode=0x01,     # CAN控制模式
                move_mode=0x01,     # MOVE J
                move_spd_rate_ctrl=30
            )
            rospy.loginfo("📍 从臂跟随模式：已退出示教模式，等待接收控制指令")

    # 🆕 新增：示教模式切换回调 (mode=0专用)
    def teach_mode_callback(self, msg):
        with self._mode_transition_lock:
            self._set_teach_mode(msg)

    def _set_teach_mode(self, msg):
        """示教模式切换回调"""
        if self.mode != 0:
            rospy.logwarn("只有mode=0的节点才支持示教模式切换")
            return

        if self.require_follower_ready:
            # On this firmware FA itself releases the master. An independent
            # MotionCtrl_1(teach=1) must never bypass native priming/readiness.
            if msg.data == 1:
                if self.current_linkage_config != 0xFA or not self.in_teach_mode:
                    rospy.logerr("Managed teaching requires the native FA handoff")
                return
            if msg.data == 0:
                self._stop_native_forwarding()
                try:
                    self._return_to_current_native_hold()
                    self._publish_mode_status()
                except Exception as exc:
                    self._mark_native_mode_unknown()
                    rospy.logerr("Managed teaching exit failed: %s", exc)
                return
            
        try:
            if msg.data == 1:
                if self.current_linkage_config != 0xFA or self._transition_in_progress:
                    raise RuntimeError("Teaching request requires a completed FA configuration transition")
                if self.in_teach_mode and self._master_publish_enabled:
                    return
                self._wait_for_follower_readiness()
                # 进入拖动示教模式
                rospy.loginfo("🔄 切换到拖动示教模式")
                self._checked_sdk_write('MotionCtrl_1', grag_teach_ctrl=0x01)
                self.in_teach_mode = True
                self._master_publish_enabled = True
                
            elif msg.data == 0:
                self._master_publish_enabled = False
                # 退出拖动示教模式
                rospy.loginfo("🔒 退出拖动示教模式，进入控制模式")
                self._checked_sdk_write('MotionCtrl_1', grag_teach_ctrl=0x02)
                self.in_teach_mode = False
                
                # 重新设置为CAN控制模式
                time.sleep(0.5)
                self._checked_sdk_write('MotionCtrl_2', ctrl_mode=0x01, move_mode=0x01, move_spd_rate_ctrl=30)
                
        except Exception as e:
            self._master_publish_enabled = False
            self.in_teach_mode = False
            if self.require_follower_ready and not self._manual_disable_requested.is_set():
                try:
                    if self.current_linkage_config == 0xFA:
                        self.current_linkage_config = 0x00
                        self.new_config_ = None
                        self._return_to_alignment_hold()
                    else:
                        self._hold_last_alignment_pose()
                except Exception as hold_exc:
                    rospy.logerr(f"主臂保持目标不可用: {hold_exc}")
            rospy.logerr(f"❌ 示教模式切换失败: {e}")
        finally:
            if self._manual_disable_requested.is_set():
                self._mark_manually_disabled_state()

    # 🆕 新增：主臂使能回调 (mode=0专用)
    def master_enable_callback(self, msg):
        if self.mode != 0:
            rospy.logwarn("只有mode=0的节点才支持使能控制")
            return
        if msg.data is False:
            # A requested stop must not queue behind a six-second mode change.
            # Latch before taking the short write lock; an in-flight SDK write
            # may finish, but no later automatic write may overtake this stop.
            self._manual_disable_requested.set()
            self._mark_manually_disabled_state()
            try:
                with self._torque_write_lock:
                    self.piper.DisableArm(7)
            except Exception as exc:
                rospy.logerr(f"手动失能发送失败；取消已锁定: {exc}")
            return
        with self._mode_transition_lock:
            self._set_master_enable(msg)

    def _set_master_enable(self, msg):
        """主臂使能/失能回调"""
        if self.mode != 0:
            rospy.logwarn("只有mode=0的节点才支持使能控制")
            return
            
        try:
            self._raise_if_manual_disabled()
            if msg.data and not self.is_enabled:
                # 使能主臂
                self._checked_sdk_write('EnableArm', 7)
                self.is_enabled = True
                self.__enable_flag = True
                rospy.loginfo("✅ 主臂已使能")
                
                # 使能后根据当前配置进行初始化
                time.sleep(2)
                if self.current_linkage_config == 0xFA:
                    self._init_master_teach_mode()
                elif self.current_linkage_config == 0xFC:
                    self._init_slave_follow_mode()
                
        except Exception as e:
            self._master_publish_enabled = False
            self.in_teach_mode = False
            if (self.require_follower_ready and self.current_linkage_config == 0xFA
                    and not self._manual_disable_requested.is_set()):
                try:
                    self.current_linkage_config = 0x00
                    self.new_config_ = None
                    self._return_to_alignment_hold()
                except Exception as hold_exc:
                    rospy.logerr(f"主臂返回FC保持失败: {hold_exc}")
            rospy.logerr(f"❌ 使能操作失败: {e}")
        finally:
            if self._manual_disable_requested.is_set():
                self._mark_manually_disabled_state()

    # 🆕 新增：发布模式状态 (mode=0专用)
    def _publish_mode_status(self):
        """发布当前模式状态"""
        if self.mode == 0:
            mode_msg = String()
            config = 0x00 if self._manual_disable_requested.is_set() else self.current_linkage_config
            config_desc = {0xFA: "主臂示教模式", 0xFC: "从臂跟随模式"}.get(config, "控制模式未确认")
            if config == 0xFA and self.require_follower_ready:
                with self._native_forward_lock:
                    phase = self._native_forward_phase
                    waiting_reason = phase.get('waiting_reason') if phase is not None else None
                if waiting_reason == 'missing-data':
                    config_desc += "，等待主臂新数据"
            mode_msg.data = f"0x{config:02X}:{config_desc}"
            self.mode_status_pub.publish(mode_msg)

    # 🆕 新增：发布主臂状态 (mode=0专用)
    def _publish_master_status(self):
        """发布主臂状态"""
        if self.mode == 0:
            try:
                # 使用Twist消息传递状态信息
                status_msg = Twist()
                
                # 用Twist的线性和角速度字段来传递状态信息
                status_msg.linear.x = 1.0  # 连接状态（固定为1）
                cancelled = self._manual_disable_requested.is_set()
                status_msg.linear.y = 1.0 if self.is_enabled and not cancelled else 0.0
                status_msg.linear.z = 1.0 if self.in_teach_mode and not cancelled else 0.0
                
                # 添加时间戳信息
                status_msg.angular.x = rospy.Time.now().to_sec()
                
                # 添加配置信息
                status_msg.angular.y = 0.0 if cancelled else float(self.current_linkage_config)
                status_msg.angular.z = self._native_prepared_since() if self.require_follower_ready else 0.0
                
                self.master_status_pub.publish(status_msg)
                
                # 发布模式状态
                self._publish_mode_status()
                
            except Exception as e:
                rospy.logerr_throttle(10, f"❌ 发布主臂状态失败: {e}")

    def GetEnableFlag(self):
        return self.__enable_flag and not self._manual_disable_requested.is_set()

    def _raise_if_manual_disabled(self):
        if self._manual_disable_requested.is_set():
            raise RuntimeError("Manual disable is latched; restart this node before re-enabling")

    def _mark_manually_disabled_state(self):
        # The permanent cancellation latch is already set. Invalidate the phase
        # without waiting behind even an in-flight ROS publication; manual
        # DisableArm may wait only for the existing single SDK write lock.
        if hasattr(self, '_native_forward_phase'):
            self._native_forward_phase = None
        self._master_publish_enabled = False
        self.in_teach_mode = False
        self.is_enabled = False
        self.__enable_flag = False
        self.current_linkage_config = 0x00
        self.new_config_ = None
        with self._readiness_lock:
            self._last_joint_command_valid = False

    def _checked_sdk_write(self, method, *args, **kwargs):
        """Serialize one SDK write with explicit manual disable, never a sleep."""
        with self._torque_write_lock:
            self._raise_if_manual_disabled()
            result = getattr(self.piper, method)(*args, **kwargs)
            self._raise_if_manual_disabled()
            return result

    def follower_ready_callback(self, msg, side):
        with self._readiness_lock:
            self._follower_ready_rows[side] = {
                'ready': msg.linear.x == 1.0,
                'source_time': msg.angular.x,
                'command_time': msg.linear.y,
                'received_monotonic': time.monotonic(),
            }

    def _native_stage(self, name, source_time=None):
        event = {'stage': name, 'wall_time': time.time(), 'monotonic': time.monotonic()}
        if source_time is not None:
            event.update(source_time=source_time, source_latency_s=event['wall_time'] - source_time)
        report = self.native_handoff_report
        if report is not None:
            report['stages'].append(event)
        rospy.loginfo("Native master handoff %s: %s", self.can_port, event)

    def _record_gripper_handoff(self, stage, **values):
        """Record wire values without treating an FA command as actual feedback."""
        event = dict(stage=stage, wall_time=time.time(), **values)
        if self.native_handoff_report is not None:
            self.native_handoff_report.setdefault('gripper_stages', []).append(event)
        rospy.loginfo("Native master gripper handoff %s: %s", self.can_port, event)





    def _stop_native_forwarding(self):
        lock = getattr(self, '_native_forward_lock', None)
        if lock is not None:
            with lock:
                self._native_forward_phase = None

    def _alignment_command_key(self):
        """Bind preparation to the six arm joints; the gripper keeps following."""
        with self._readiness_lock:
            target = self._last_alignment_target
            valid = self._last_joint_command_valid
        if (not valid or target is None or len(target) != 6
                or not all(math.isfinite(value) for value in target)):
            return None
        return tuple(target)

    def _invalidate_native_preparation(self, reason):
        with self._native_forward_lock:
            phase = self._native_forward_phase
            if phase is not None and phase['kind'] == 'actual':
                phase['prepared'] = False
                phase['error'] = reason
                self._native_forward_phase = None
        if self.native_handoff_report is not None:
            self.native_handoff_report['prepared'] = False

    def _native_prepared_since(self):
        """Publish a preparation token only while the full actual path is fresh."""
        if (self._manual_disable_requested.is_set() or not self.is_enabled
                or not self.GetEnableFlag() or self.current_linkage_config != 0xFC
                or self.in_teach_mode or self._transition_in_progress):
            return 0.0
        key = self._alignment_command_key()
        with self._native_forward_lock:
            phase = self._native_forward_phase
            if (phase is None or phase['kind'] != 'actual' or not phase.get('prepared')
                    or phase['error']):
                return 0.0
            if key != phase.get('alignment_key'):
                phase['prepared'] = False
                return 0.0
        try:
            snapshot = self._native_master_reader.snapshot()
            if snapshot is None:
                return 0.0
            if (snapshot['kind'] != 'actual' or snapshot['epoch'] != phase['epoch']
                    or snapshot['source_time'] < phase['since']):
                raise RuntimeError('Prepared native actual epoch/source is invalid')
        except Exception:
            with self._native_forward_lock:
                if self._native_forward_phase is phase:
                    phase['prepared'] = False
            return 0.0
        with self._readiness_lock:
            rows = {side: dict(row) for side, row in self._follower_ready_rows.items()}
        try:
            self._native_require_recent_pair(rows, phase['since'])
        except RuntimeError:
            # A transient heartbeat gap is not a different preparation epoch.
            # Keep FC tracking live and withhold the readiness token until it recovers.
            return 0.0
        with self._native_forward_lock:
            if (self._native_forward_phase is phase and phase.get('prepared')
                    and time.monotonic() - phase['last_progress'] < 0.2):
                return phase['since']
        return 0.0

    def _native_require_recent_pair(self, rows, minimum_source):
        now = time.time()
        require_pair_ready(rows, now_wall=now, now_monotonic=time.monotonic(),
                           minimum_command_time=minimum_source)
        for side in ('left', 'right'):
            if not -0.05 <= now - rows[side]['command_time'] <= 0.5:
                raise RuntimeError(side + ' follower executed-source ACK is older than 0.5s')

    def _native_reader_diagnostics(self, phase=None):
        try:
            return self._native_master_reader.diagnostics(after=phase.get('previous') if phase else None)
        except Exception as exc:
            return {'diagnostic_error': str(exc)}

    def _mark_native_mode_unknown(self):
        self._stop_native_forwarding()
        self._master_publish_enabled = False
        self.in_teach_mode = False
        self.current_linkage_config = 0x00
        self.new_config_ = None

    def _arm_native_forwarding(self, kind, *, expected_phase=None):
        self._raise_if_manual_disabled()
        reader = self._native_master_reader
        if reader is None:
            raise RuntimeError("Native master reader was not initialized")
        reader.wait_started(timeout=1.0)
        with self._native_forward_lock:
            self._raise_if_manual_disabled()
            if expected_phase is not None and (
                    self._native_forward_phase is not expected_phase or expected_phase['error']):
                raise RuntimeError('Prepared FC phase changed before native FA arming')
            if kind == 'fa' and self._native_last_actual is None:
                raise RuntimeError("No native FC pose was forwarded before release")
            since = time.time()
            epoch = reader.begin_phase(kind, since_timestamp=since)
            started = time.monotonic()
            startup_timeout_s = None if kind == 'fa' else 0.25
            phase = {'kind': kind, 'epoch': epoch, 'since': since,
                     'started_monotonic': started,
                     'startup_timeout_s': startup_timeout_s,
                     'startup_deadline': None if startup_timeout_s is None else started + startup_timeout_s,
                     'previous': None,
                     'alignment_key': self._alignment_command_key() if kind == 'actual' else None,
                     'prepared': False,
                     'following_enabled': True,
                     'handoff_confirmed': False, 'waiting': False, 'waiting_reason': None,
                     'first_source': None, 'last_source': None, 'last_progress': started,
                     'error': None}
            self._native_forward_phase = phase
        self._native_stage(kind + '-armed')
        if startup_timeout_s is None:
            rospy.loginfo("Native master %s %s waiting for the first fresh complete group without a startup timeout",
                          self.can_port, kind)
        else:
            rospy.loginfo("Native master %s %s startup window %.2fs; forwarding begins on the first fresh complete group",
                          self.can_port, kind, startup_timeout_s)
        return phase

    def _native_forward_failed(self, phase, error):
        diagnostics = self._native_reader_diagnostics(phase)
        with self._native_forward_lock:
            if phase is not None and self._native_forward_phase is phase:
                phase['error'] = str(error)
                phase['prepared'] = False
                self._native_forward_phase = None
                if phase['kind'] == 'fa':
                    self._master_publish_enabled = False
                    self.in_teach_mode = False
                    self.current_linkage_config = 0x00
                    self.new_config_ = None
        if self.native_handoff_report is not None:
            self.native_handoff_report.update(prepared=False, native_diagnostics=diagnostics)
        rospy.logerr("Native forwarding stopped: %s; native diagnostics: %s", error, diagnostics)

    def _native_wait_for_data(self, phase, reason):
        """Mark recoverable FA waiting under the existing forwarding lock."""
        changed = not phase['waiting'] or phase['waiting_reason'] != reason
        phase['waiting'] = True
        phase['waiting_reason'] = reason
        if changed:
            self._native_stage('fa-waiting-' + reason)
            rospy.logwarn("Native master %s waiting (%s); fresh complete native data will resume forwarding "
                          "without cached command replay or a mode change",
                          self.can_port, reason)

    def _native_forward_tick(self):
        """Forward only a complete advancing native RX sample, independent of mode locks."""
        phase = None
        first_source = None
        resumed = False
        try:
            with self._native_forward_lock:
                phase = self._native_forward_phase
                if phase is None:
                    return
                self._raise_if_manual_disabled()
                if phase['kind'] == 'fa' and not phase['following_enabled']:
                    return
                # FA waits for fresh complete groups without a startup deadline.
                # FC preparation retains its existing startup/stream watchdogs.
                if (phase['first_source'] is None and phase['startup_deadline'] is not None
                        and time.monotonic() >= phase['startup_deadline']):
                    raise RuntimeError(f"No first complete native {phase['kind']} master pose within "
                                       f"{phase['startup_timeout_s']:.2f}s startup window")
                snapshot = self._native_master_reader.snapshot(after=phase['previous'])
                if snapshot is None:
                    if phase['first_source'] is not None and time.monotonic() - phase['last_progress'] >= 0.25:
                        if phase['kind'] == 'fa':
                            self._native_wait_for_data(phase, 'missing-data')
                            return
                        raise RuntimeError("Native master pose stream is missing/stale")
                    return
                if snapshot['kind'] != phase['kind'] or snapshot['epoch'] != phase['epoch']:
                    return
                joints = snapshot['joint_mdeg']
                gripper = snapshot['command_gripper_um']
                source = snapshot['source_time']
                if (len(joints) != 6 or not all(math.isfinite(value) for value in joints)
                        or not math.isfinite(gripper) or not 0 <= gripper <= 80000
                        or not math.isfinite(source) or source < phase['since']
                        or not -0.05 <= time.time() - source <= 0.2):
                    raise RuntimeError("Invalid/stale native master pose")
                if phase['kind'] == 'actual':
                    if self._alignment_command_key() != phase['alignment_key']:
                        raise RuntimeError("Prepared alignment command changed")
                message = JointState()
                message.header.stamp = rospy.Time.from_sec(source)
                message.name = ['joint0', 'joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
                message.position = [value / 57324.840764 for value in joints] + [gripper / 1_000_000]
                # Phase replacement and publication share this short lock. The
                # first-message event therefore cannot describe an older epoch.
                self._raise_if_manual_disabled()
                if self._native_forward_phase is not phase:
                    return
                self.joint_std_pub_master.publish(message)
                phase['previous'] = dict(snapshot['component_stamps'])
                phase['last_source'] = source
                phase['last_progress'] = time.monotonic()
                resumed = phase['waiting']
                phase['waiting'] = False
                phase['waiting_reason'] = None
                if phase['kind'] == 'actual':
                    self._native_last_actual = snapshot
                if phase['first_source'] is None:
                    phase['first_source'] = source
                    first_source = source
            if first_source is not None:
                self._native_stage(phase['kind'] + '-first-forwarded', first_source)
                self._record_gripper_handoff(phase['kind'] + '-first-gripper',
                    value_kind='actual-width' if phase['kind'] == 'actual' else 'teaching-command-width',
                    raw_width_um=snapshot['gripper_um'],
                    forwarded_width_um=snapshot['command_gripper_um'],
                    effort_mnm=snapshot.get('gripper_effort_mnm'),
                    status_byte=snapshot.get('gripper_status_byte'),
                    byte7=snapshot.get('gripper_byte7'),
                    width_mode_byte=snapshot.get('gripper_byte7') if phase['kind'] == 'actual' else None,
                    set_zero_code=snapshot.get('gripper_byte7') if phase['kind'] == 'fa' else None,
                    source_time=snapshot.get('gripper_source_time'), hex=snapshot.get('gripper_hex'))
            if resumed:
                self._native_stage('fa-native-resumed', source)
                rospy.loginfo("Native master %s resumed from fresh complete data; no mode change", self.can_port)
        except Exception as exc:
            self._native_forward_failed(phase, exc)

    def MasterNativeForwardingThread(self):
        while not rospy.is_shutdown():
            self._native_forward_tick()
            time.sleep(0.005)

    def _wait_native_first(self, phase):
        deadline = phase['startup_deadline']
        while not rospy.is_shutdown():
            self._raise_if_manual_disabled()
            with self._native_forward_lock:
                if phase['error']:
                    raise RuntimeError(phase['error'])
                if self._native_forward_phase is not phase:
                    raise RuntimeError("Native forwarding phase was cancelled/replaced")
                if phase['first_source'] is not None:
                    return phase['first_source']
            if deadline is not None and time.monotonic() >= deadline:
                raise RuntimeError(f"No first complete native {phase['kind']} master pose within "
                                   f"{phase['startup_timeout_s']:.2f}s startup window")
            time.sleep(0.005)
        raise RuntimeError("ROS shutdown during native master handoff")

    def _wait_native_pair_ack(self, phase, minimum_source, timeout, *, deadline=None):
        if deadline is None and timeout is not None:
            deadline = time.monotonic() + timeout
        last_error = 'No follower source acknowledgement'
        while not rospy.is_shutdown():
            self._raise_if_manual_disabled()
            with self._native_forward_lock:
                if phase['error']:
                    raise RuntimeError(phase['error'])
                if self._native_forward_phase is not phase:
                    raise RuntimeError("Native forwarding phase was cancelled/replaced")
            with self._readiness_lock:
                rows = {side: dict(row) for side, row in self._follower_ready_rows.items()}
            if deadline is not None and time.monotonic() >= deadline:
                raise RuntimeError("Native follower acknowledgement timed out: " + last_error)
            try:
                self._native_require_recent_pair(rows, minimum_source)
                return
            except RuntimeError as exc:
                last_error = str(exc)
            if deadline is not None and time.monotonic() >= deadline:
                raise RuntimeError("Native follower acknowledgement timed out: " + last_error)
            time.sleep(0.005)
        raise RuntimeError("ROS shutdown during native follower acknowledgement")

    def _native_handoff_error(self, error, *, fa_attempted):
        phase = self._native_forward_phase
        prior = self.native_handoff_report or {}
        diagnostics = prior.get('native_diagnostics') if phase is None else None
        if diagnostics is None:
            diagnostics = self._native_reader_diagnostics(phase)
        if self.native_handoff_report is None:
            self.native_handoff_report = {'passed': False, 'prepared': False, 'stages': []}
        self.native_handoff_report.update(error=str(error), prepared=False,
                                          native_diagnostics=diagnostics)
        self._stop_native_forwarding()
        self._master_publish_enabled = False
        self.in_teach_mode = False
        if not self._manual_disable_requested.is_set() and not rospy.is_shutdown():
            try:
                self._mark_native_mode_unknown()
                if fa_attempted:
                    self._return_to_current_native_hold()
            except Exception as hold_exc:
                self._mark_native_mode_unknown()
                rospy.logerr("Native handoff holding recovery failed: %s", hold_exc)
        rospy.logerr("Native master handoff failed: %s; native diagnostics: %s", error, diagnostics)

    def _prepare_native_master_mode(self):
        """Prepare continuous FC tracking without releasing the master."""
        if self.current_linkage_config != 0xFC or self.in_teach_mode:
            rospy.logwarn("prepare_master requires a position-held FC master; no role change requested")
            return False
        self.native_handoff_report = {'passed': False, 'prepared': False,
                                      'stages': [], 'fa_attempted': False}
        self._transition_in_progress = True
        self._master_publish_enabled = False
        try:
            self._raise_if_manual_disabled()
            key = self._alignment_command_key()
            if key is None or not self.is_enabled or not self.GetEnableFlag():
                raise RuntimeError("Native preparation requires an enabled master and completed six-joint alignment command")
            if self.joint_std_pub_master.get_num_connections() < 1:
                raise RuntimeError("Native master publisher has no follower connection")
            actual = self._arm_native_forwarding('actual')
            self._wait_native_first(actual)
            self._wait_native_pair_ack(actual, actual['since'], self.follower_ready_timeout_s)
            with self._native_forward_lock:
                if self._native_forward_phase is not actual or actual['error']:
                    raise RuntimeError("FC preparation stopped before source acknowledgement")
                if self._alignment_command_key() != actual['alignment_key']:
                    raise RuntimeError("Alignment command changed during FC preparation")
                actual['prepared'] = True
            self.native_handoff_report.update(prepared=True, prepared_since=actual['since'])
            self._native_stage('fc-priming-acknowledged', actual['last_source'])
            self._publish_mode_status()
        except Exception as exc:
            self._native_handoff_error(exc, fa_attempted=False)
        finally:
            self._transition_in_progress = False
            if self._manual_disable_requested.is_set():
                self._mark_manually_disabled_state()
        return bool(self.native_handoff_report.get('prepared'))

    def _release_native_master_mode(self, release_gripper=False):
        """Release only an existing live prepared FC phase; never prime again."""
        fa_attempted = False
        try:
            self._raise_if_manual_disabled()
            with self._native_forward_lock:
                actual = self._native_forward_phase
            if actual is None or actual['kind'] != 'actual':
                raise RuntimeError("release_master requires a current fresh prepared FC phase")
            deadline = time.monotonic() + 0.5
            while self._native_prepared_since() != actual['since']:
                self._raise_if_manual_disabled()
                with self._native_forward_lock:
                    if (self._native_forward_phase is not actual or not actual.get('prepared')
                            or actual['error']):
                        raise RuntimeError('Prepared FC phase is invalid/replaced')
                if time.monotonic() >= deadline or rospy.is_shutdown():
                    raise RuntimeError('Prepared FC path is not currently fresh/acknowledged')
                time.sleep(0.005)
            self._transition_in_progress = True
            with self._native_forward_lock:
                if self._native_forward_phase is not actual or not actual.get('prepared'):
                    raise RuntimeError("Prepared FC phase changed before release")
                actual['prepared'] = False
            self.native_handoff_report['prepared'] = False
            latest_actual = self._native_last_actual or {}
            self._record_gripper_handoff('fc-to-fa-boundary', value_kind='actual-width',
                experimental_release=release_gripper and self.can_port in ('can_left_mas', 'can_right_mas'),
                last_alignment_width_um=self._last_alignment_command_gripper_um,
                raw_width_um=latest_actual.get('gripper_um'),
                effort_mnm=latest_actual.get('gripper_effort_mnm'),
                status_byte=latest_actual.get('gripper_status_byte'),
                source_time=latest_actual.get('gripper_source_time'))
            fa = self._arm_native_forwarding('fa', expected_phase=actual)
            if release_gripper and self.can_port in ('can_left_mas', 'can_right_mas'):
                # Use the already-prepared actual snapshot; there is no new
                # feedback read, acknowledgement wait, or recovery branch.
                snapshot = self._native_last_actual
                self._checked_sdk_write('GripperCtrl', snapshot['command_gripper_um'], 0, 0x00, 0)
                self._record_gripper_handoff('gripper-release-sent', value_kind='actual-width',
                    raw_width_um=snapshot['gripper_um'],
                    requested_width_um=snapshot['command_gripper_um'],
                    source_time=snapshot.get('gripper_source_time'))
            self._raise_if_manual_disabled()
            self._native_stage('fa-command-start')
            with self._native_forward_lock:
                if self._native_forward_phase is not fa or fa['error']:
                    raise RuntimeError('Native FA phase was cancelled before the role command')
            fa_attempted = True
            self.native_handoff_report['fa_attempted'] = True
            self._checked_sdk_write('MasterSlaveConfig', linkage_config=0xFA,
                                    feedback_offset=0, ctrl_offset=0, linkage_offset=0)
            self._native_stage('fa-command-return')
            first_source = self._wait_native_first(fa)
            # The masters may start at different times. Wait for both real,
            # fresh executed-source ACKs without a handoff timeout.
            self._wait_native_pair_ack(fa, first_source, timeout=None)
            self._raise_if_manual_disabled()
            with self._native_forward_lock:
                if self._native_forward_phase is not fa or fa['error']:
                    raise RuntimeError("Native FA stream stopped before readiness acknowledgement")
                self.current_linkage_config = 0xFA
                self.new_config_ = 0xFA
                self.in_teach_mode = True
                self._master_publish_enabled = True
                fa['handoff_confirmed'] = True
            self.native_handoff_report['passed'] = True
            self._native_stage('fa-following-acknowledged', first_source)
            self._publish_mode_status()
        except Exception as exc:
            self._native_handoff_error(exc, fa_attempted=fa_attempted)
        finally:
            self._transition_in_progress = False
            if self._manual_disable_requested.is_set():
                self._mark_manually_disabled_state()
        return bool(self.native_handoff_report.get('passed'))

    def _enter_native_master_mode(self):
        """Compatibility for old collectors: prepare then release in this call."""
        if self._prepare_native_master_mode():
            self._release_native_master_mode()

    def _return_to_current_native_hold(self):
        """Exit FA, then hold a NEW native measured pose; never pull to old alignment."""
        self._stop_native_forwarding()
        self._raise_if_manual_disabled()
        if rospy.is_shutdown():
            raise RuntimeError("ROS shutdown; no automatic holding recovery")
        self._mark_native_mode_unknown()
        since = time.time()
        self._checked_sdk_write('MasterSlaveConfig', linkage_config=0xFC,
                                feedback_offset=0, ctrl_offset=0, linkage_offset=0)
        deadline = time.monotonic() + 3.0
        previous = None
        anchor = None
        stable_since = None
        while time.monotonic() < deadline and not rospy.is_shutdown():
            self._raise_if_manual_disabled()
            try:
                row = sample_feedback(self.can_port, since_timestamp=since)
            except RuntimeError as exc:
                if not str(exc).startswith('Missing required CAN feedback frames'):
                    raise
                time.sleep(0.01)
                continue
            self._raise_if_manual_disabled()
            joints, previous = checked_enabled_feedback(
                row, now=time.time(), since_timestamp=since, previous=previous)
            if anchor is None:
                anchor = tuple(joints)
                self._checked_sdk_write('JointCtrl', *anchor)
                self._checked_sdk_write('MotionCtrl_1', grag_teach_ctrl=0x02)
                self._checked_sdk_write('JointCtrl', *anchor)
                since = time.time()
                self._checked_sdk_write('MotionCtrl_2', ctrl_mode=1, move_mode=1, move_spd_rate_ctrl=10)
                self._checked_sdk_write('JointCtrl', *anchor)
                continue
            if row['ctrl_mode'] != 1:
                raise RuntimeError("Fresh-current master holding recovery did not reach CAN1")
            if stable_since is None:
                stable_since = time.monotonic()
            if time.monotonic() - stable_since >= 0.3:
                self.current_linkage_config = 0xFC
                self.new_config_ = 0xFC
                self.in_teach_mode = False
                self._master_publish_enabled = False
                with self._readiness_lock:
                    self._last_alignment_target = None
                    self._last_joint_command_valid = False
                self._native_stage('fresh-current-fc-hold-verified')
                return
        raise RuntimeError("Fresh-current master holding recovery timed out or was cancelled")

    def FollowerReadinessThread(self):
        """Passively sample CAN away from the 200 Hz publishing/control loop."""
        previous = None
        while not rospy.is_shutdown():
            raw = None
            try:
                raw = sample_feedback(self.can_port)
            except Exception:
                pass  # Publish false rather than refreshing a cached success.
            with self._readiness_lock:
                loop_at = self._last_publish_loop_monotonic
                command_at = self._last_joint_command_wall
                subscribed = self._joint_subscription_ready
                command_valid = self._last_joint_command_valid
            # Take these clocks after sampling; each raw CAN component's age
            # is recomputed at publication, not at the old aggregate timestamp.
            snapshot = follower_readiness(
                raw, now_wall=time.time(), now_monotonic=time.monotonic(),
                loop_monotonic=loop_at, enabled_flag=self.GetEnableFlag(),
                subscriber_ready=subscribed and command_valid, command_time=command_at,
                previous_raw_stamps=previous)
            if 'raw_stamps' in snapshot:
                previous = snapshot['raw_stamps']
            message = Twist()
            message.linear.x = 1.0 if snapshot['ready'] else 0.0
            message.angular.x = snapshot['source_time']
            message.linear.y = snapshot['command_time']
            self._follower_ready_publisher.publish(message)
            time.sleep(0.03)

    def _hold_last_alignment_pose(self):
        """Keep the last accepted master pose while followers are unavailable."""
        self._raise_if_manual_disabled()
        with self._readiness_lock:
            target = self._last_alignment_target
        if (target is None or len(target) != 6
                or not all(math.isfinite(value) for value in target)):
            raise RuntimeError("No trusted completed alignment target; refusing teaching release")
        if not self.is_enabled or not self.GetEnableFlag():
            raise RuntimeError("Master is not enabled; refusing teaching release")
        self._checked_sdk_write('MotionCtrl_2', ctrl_mode=0x01, move_mode=0x01, move_spd_rate_ctrl=10)
        self._checked_sdk_write('JointCtrl', *target)

    def _return_to_alignment_hold(self):
        """Undo an attempted FA role without disabling or selecting a new pose."""
        if self.require_follower_ready:
            return self._return_to_current_native_hold()
        self._raise_if_manual_disabled()
        self._master_publish_enabled = False
        self.in_teach_mode = False
        with self._readiness_lock:
            target = self._last_alignment_target
        if (target is None or len(target) != 6
                or not all(math.isfinite(value) for value in target)):
            raise RuntimeError("Cannot restore FC holding without a trusted alignment target")
        if not self.is_enabled or not self.GetEnableFlag():
            raise RuntimeError("Cannot restore FC holding while master is not enabled")
        self._checked_sdk_write('JointCtrl', *target)
        self._checked_sdk_write('MotionCtrl_1', grag_teach_ctrl=0x02)
        self._checked_sdk_write('JointCtrl', *target)
        phase = time.time()
        self._checked_sdk_write('MasterSlaveConfig', linkage_config=0xFC, feedback_offset=0,
                                     ctrl_offset=0, linkage_offset=0)
        self._checked_sdk_write('JointCtrl', *target)
        self._checked_sdk_write('MotionCtrl_2', ctrl_mode=0x01, move_mode=0x01, move_spd_rate_ctrl=10)
        self._checked_sdk_write('JointCtrl', *target)
        deadline = time.monotonic() + 3.0
        previous = None
        last_error = 'No fresh master CAN feedback after FC recovery'
        while time.monotonic() < deadline and not rospy.is_shutdown():
            try:
                row = sample_feedback(self.can_port, since_timestamp=phase)
            except RuntimeError as exc:
                if not str(exc).startswith('Missing required CAN feedback frames'):
                    raise
                last_error = str(exc)
                time.sleep(0.03)
                continue
            # After the bounded FC reseeding sequence, confirmation is passive.
            # A fault or disabled motor must not cause blind repetition of an
            # old target against an uncertain pose.
            joints, previous = checked_enabled_feedback(
                row, now=time.time(), since_timestamp=phase, previous=previous)
            if row['ctrl_mode'] != 1:
                last_error = 'Master did not acknowledge CAN1 after FC recovery'
                time.sleep(0.03)
                continue
            self.current_linkage_config = 0xFC
            self.new_config_ = 0xFC
            return
        raise RuntimeError('FC position-hold recovery was not acknowledged: ' + last_error)

    def _wait_for_follower_readiness(self, timeout=None):
        self._raise_if_manual_disabled()
        if not self.require_follower_ready:
            return
        with self._readiness_lock:
            target = self._last_alignment_target
            command_valid = self._last_joint_command_valid
        if (not command_valid or target is None or len(target) != 6
                or not all(math.isfinite(value) for value in target)):
            raise RuntimeError("No trusted completed alignment target; refusing teaching release")
        if not self.is_enabled or not self.GetEnableFlag():
            raise RuntimeError("Master is not enabled; refusing teaching release")
        wait_seconds = self.follower_ready_timeout_s if timeout is None else timeout
        deadline = time.monotonic() + wait_seconds
        last_error = 'No follower heartbeats'
        while not rospy.is_shutdown():
            self._raise_if_manual_disabled()
            with self._readiness_lock:
                rows = {side: dict(row) for side, row in self._follower_ready_rows.items()}
            try:
                require_pair_ready(rows, now_wall=time.time(), now_monotonic=time.monotonic())
                return
            except RuntimeError as exc:
                last_error = str(exc)
            self._hold_last_alignment_pose()
            if time.monotonic() >= deadline:
                raise RuntimeError('Follower readiness timed out; master remains position-held: ' + last_error)
            time.sleep(0.05)
        raise RuntimeError('ROS shutdown before teaching release')

    def Pubilsh(self):
        """机械臂消息发布
        """
        rate = rospy.Rate(200)  # 200 Hz
        enable_flag = False
        # 设置超时时间（秒）
        timeout = 5
        # 记录进入循环前的时间
        start_time = time.time()
        elapsed_time_flag = False
        while not rospy.is_shutdown():
            if(self.auto_enable and self.mode == 1):
                while not (enable_flag):
                    elapsed_time = time.time() - start_time
                    print("--------------------")
                    enable_flag = self.piper.GetArmLowSpdInfoMsgs().motor_1.foc_status.driver_enable_status and \
                        self.piper.GetArmLowSpdInfoMsgs().motor_2.foc_status.driver_enable_status and \
                        self.piper.GetArmLowSpdInfoMsgs().motor_3.foc_status.driver_enable_status and \
                        self.piper.GetArmLowSpdInfoMsgs().motor_4.foc_status.driver_enable_status and \
                        self.piper.GetArmLowSpdInfoMsgs().motor_5.foc_status.driver_enable_status and \
                        self.piper.GetArmLowSpdInfoMsgs().motor_6.foc_status.driver_enable_status
                    print("使能状态:",enable_flag)
                    if(enable_flag):
                        self.__enable_flag = True
                    self._checked_sdk_write('EnableArm', 7)
                    self._checked_sdk_write('GripperCtrl', 0,1000,0x02, 0)
                    self._checked_sdk_write('GripperCtrl', 0,1000,0x01, 0)
                    print("--------------------")
                    # 检查是否超过超时时间
                    if elapsed_time > timeout:
                        print("超时....")
                        elapsed_time_flag = True
                        enable_flag = True
                        break
                    time.sleep(1)
                    pass
            if(elapsed_time_flag):
                print("程序自动使能超时,退出程序")
                exit(0)
            # 发布消息
            self.PublishSlaveArmJointAndGripper()
            self.PublishSlaveArmState()
            self.PublishSlaveArmEndPose()
            # 模式为0的时候，发布主臂消息
            if self.mode == 0 and not self.require_follower_ready:
                if (self._master_publish_enabled and self.in_teach_mode and not self._transition_in_progress
                        and not self._manual_disable_requested.is_set()):
                    self.PublishMasterArmJointAndGripper()
                    rospy.loginfo_throttle(10, "📡 Publishing master arm joint states (mode_master=True)")
                else:
                    rospy.loginfo_throttle(10, "⏸️ Master joint states publishing disabled (mode_master=False)")
            self._publish_master_status()
            with self._readiness_lock:
                self._last_publish_loop_monotonic = time.monotonic()
            rate.sleep()
    
    # 其他原有方法保持不变...
    def PublishSlaveArmState(self):
        arm_status = PiperStatusMsg()
        arm_status.ctrl_mode = self.piper.GetArmStatus().arm_status.ctrl_mode
        arm_status.arm_status = self.piper.GetArmStatus().arm_status.arm_status
        arm_status.mode_feedback = self.piper.GetArmStatus().arm_status.mode_feed
        arm_status.teach_status = self.piper.GetArmStatus().arm_status.teach_status
        arm_status.motion_status = self.piper.GetArmStatus().arm_status.motion_status
        arm_status.trajectory_num = self.piper.GetArmStatus().arm_status.trajectory_num
        arm_status.err_code = self.piper.GetArmStatus().arm_status.err_code
        arm_status.joint_1_angle_limit = self.piper.GetArmStatus().arm_status.err_status.joint_1_angle_limit
        arm_status.joint_2_angle_limit = self.piper.GetArmStatus().arm_status.err_status.joint_2_angle_limit
        arm_status.joint_3_angle_limit = self.piper.GetArmStatus().arm_status.err_status.joint_3_angle_limit
        arm_status.joint_4_angle_limit = self.piper.GetArmStatus().arm_status.err_status.joint_4_angle_limit
        arm_status.joint_5_angle_limit = self.piper.GetArmStatus().arm_status.err_status.joint_5_angle_limit
        arm_status.joint_6_angle_limit = self.piper.GetArmStatus().arm_status.err_status.joint_6_angle_limit
        arm_status.communication_status_joint_1 = self.piper.GetArmStatus().arm_status.err_status.communication_status_joint_1
        arm_status.communication_status_joint_2 = self.piper.GetArmStatus().arm_status.err_status.communication_status_joint_2
        arm_status.communication_status_joint_3 = self.piper.GetArmStatus().arm_status.err_status.communication_status_joint_3
        arm_status.communication_status_joint_4 = self.piper.GetArmStatus().arm_status.err_status.communication_status_joint_4
        arm_status.communication_status_joint_5 = self.piper.GetArmStatus().arm_status.err_status.communication_status_joint_5
        arm_status.communication_status_joint_6 = self.piper.GetArmStatus().arm_status.err_status.communication_status_joint_6
        self.arm_status_pub.publish(arm_status)
    
    def PublishSlaveArmEndPose(self):
        # 末端位姿
        endpos = PoseStamped()
        endpos.pose.position.x = self.piper.GetArmEndPoseMsgs().end_pose.X_axis/1000000
        endpos.pose.position.y = self.piper.GetArmEndPoseMsgs().end_pose.Y_axis/1000000
        endpos.pose.position.z = self.piper.GetArmEndPoseMsgs().end_pose.Z_axis/1000000
        roll = self.piper.GetArmEndPoseMsgs().end_pose.RX_axis/1000
        pitch = self.piper.GetArmEndPoseMsgs().end_pose.RY_axis/1000
        yaw = self.piper.GetArmEndPoseMsgs().end_pose.RZ_axis/1000
        roll = math.radians(roll)
        pitch = math.radians(pitch)
        yaw = math.radians(yaw)
        quaternion = quaternion_from_euler(roll, pitch, yaw)
        endpos.pose.orientation.x = quaternion[0]
        endpos.pose.orientation.y = quaternion[1]
        endpos.pose.orientation.z = quaternion[2]
        endpos.pose.orientation.w = quaternion[3]
        # 为末端位姿增加时间戳
        endpos.header.stamp = rospy.Time.now()
        self.end_pose_pub.publish(endpos)
        
        end_pose_euler = PosCmd()
        end_pose_euler.x = self.piper.GetArmEndPoseMsgs().end_pose.X_axis/1000000
        end_pose_euler.y = self.piper.GetArmEndPoseMsgs().end_pose.Y_axis/1000000
        end_pose_euler.z = self.piper.GetArmEndPoseMsgs().end_pose.Z_axis/1000000
        end_pose_euler.roll = roll
        end_pose_euler.pitch = pitch
        end_pose_euler.yaw = yaw
        end_pose_euler.gripper = self.piper.GetArmGripperMsgs().gripper_state.grippers_angle/1000000
        end_pose_euler.mode1 = 0
        end_pose_euler.mode2 = 0
        self.end_pose_euler_pub.publish(end_pose_euler)
    
    def PublishSlaveArmJointAndGripper(self):
        # 从臂反馈消息
        self.joint_state_slave.header.stamp = rospy.Time.now()
        joint_0:float = (self.piper.GetArmJointMsgs().joint_state.joint_1/1000) * 0.017444
        joint_1:float = (self.piper.GetArmJointMsgs().joint_state.joint_2/1000) * 0.017444
        joint_2:float = (self.piper.GetArmJointMsgs().joint_state.joint_3/1000) * 0.017444
        joint_3:float = (self.piper.GetArmJointMsgs().joint_state.joint_4/1000) * 0.017444
        joint_4:float = (self.piper.GetArmJointMsgs().joint_state.joint_5/1000) * 0.017444
        joint_5:float = (self.piper.GetArmJointMsgs().joint_state.joint_6/1000) * 0.017444
        joint_6:float = self.piper.GetArmGripperMsgs().gripper_state.grippers_angle/1000000
        vel_0:float = self.piper.GetArmHighSpdInfoMsgs().motor_1.motor_speed/1000
        vel_1:float = self.piper.GetArmHighSpdInfoMsgs().motor_2.motor_speed/1000
        vel_2:float = self.piper.GetArmHighSpdInfoMsgs().motor_3.motor_speed/1000
        vel_3:float = self.piper.GetArmHighSpdInfoMsgs().motor_4.motor_speed/1000
        vel_4:float = self.piper.GetArmHighSpdInfoMsgs().motor_5.motor_speed/1000
        vel_5:float = self.piper.GetArmHighSpdInfoMsgs().motor_6.motor_speed/1000
        effort_6:float = self.piper.GetArmGripperMsgs().gripper_state.grippers_effort/1000
        self.joint_state_slave.position = [joint_0,joint_1, joint_2, joint_3, joint_4, joint_5,joint_6]  # Example values
        self.joint_state_slave.velocity = [vel_0, vel_1, vel_2, vel_3, vel_4, vel_5, 0.0]  # Example values
        self.joint_state_slave.effort[6] = effort_6
        self.joint_std_pub_puppet.publish(self.joint_state_slave)
    
    def PublishMasterArmJointAndGripper(self):
        if self.require_follower_ready:
            # Managed forwarding has independent native RX freshness/epochs;
            # never substitute SDK caches or re-stamp them as current poses.
            self._native_forward_tick()
            return
        # 主臂控制消息
        self.joint_state_master.header.stamp = rospy.Time.now()
        
        # 🔧 读取控制指令（0x155/0x156/0x157 CAN消息）
        joint_0:float = (self.piper.GetArmJointCtrl().joint_ctrl.joint_1/1000) * 0.017444
        joint_1:float = (self.piper.GetArmJointCtrl().joint_ctrl.joint_2/1000) * 0.017444
        joint_2:float = (self.piper.GetArmJointCtrl().joint_ctrl.joint_3/1000) * 0.017444
        joint_3:float = (self.piper.GetArmJointCtrl().joint_ctrl.joint_4/1000) * 0.017444
        joint_4:float = (self.piper.GetArmJointCtrl().joint_ctrl.joint_5/1000) * 0.017444
        joint_5:float = (self.piper.GetArmJointCtrl().joint_ctrl.joint_6/1000) * 0.017444
        joint_6:float = self.piper.GetArmGripperCtrl().gripper_ctrl.grippers_angle/1000000
        
        # 🔧 防护：如果在示教模式下检测到全零（异常情况），则使用反馈位置
        if self.current_linkage_config == 0xFA and abs(joint_0) < 0.001 and abs(joint_1) < 0.001 and abs(joint_2) < 0.001:
            rospy.logwarn_throttle(1, "⚠️ GetArmJointCtrl返回零值，使用反馈位置替代")
            joint_0 = (self.piper.GetArmJointMsgs().joint_state.joint_1/1000) * 0.017444
            joint_1 = (self.piper.GetArmJointMsgs().joint_state.joint_2/1000) * 0.017444
            joint_2 = (self.piper.GetArmJointMsgs().joint_state.joint_3/1000) * 0.017444
            joint_3 = (self.piper.GetArmJointMsgs().joint_state.joint_4/1000) * 0.017444
            joint_4 = (self.piper.GetArmJointMsgs().joint_state.joint_5/1000) * 0.017444
            joint_5 = (self.piper.GetArmJointMsgs().joint_state.joint_6/1000) * 0.017444
            joint_6 = self.piper.GetArmGripperMsgs().gripper_state.grippers_angle/1000000
        
        self.joint_state_master.position = [joint_0,joint_1, joint_2, joint_3, joint_4, joint_5,joint_6]  # Example values
        self.joint_std_pub_master.publish(self.joint_state_master)
    
    def SubPosThread(self):
        """机械臂末端位姿订阅
        
        """
        rospy.Subscriber('/pos_cmd', PosCmd, self.pos_callback,queue_size=1,tcp_nodelay=True)
        rospy.spin()
    
    def SubJointThread(self):
        """机械臂关节订阅 - 使用专门的主臂控制话题"""
        if self.mode == 0:
            # mode=0时，主臂被控模式使用专门的控制话题
            self._joint_subscription = rospy.Subscriber('/master_controled/joint_states', JointState, self.joint_callback, queue_size=1, tcp_nodelay=True)
            rospy.loginfo("📍 主臂被控模式订阅: /master_controled/joint_states")
        else:
            # mode=1时，从臂使用原来的话题
            self._joint_subscription = rospy.Subscriber('/master/joint_states', JointState, self.joint_callback, queue_size=1, tcp_nodelay=True)
            rospy.loginfo("📍 从臂模式订阅: /master/joint_states")
        with self._readiness_lock:
            self._joint_subscription_ready = True
        
        rospy.spin()
    
    def SubEnableThread(self):
        """机械臂使能
        
        """
        rospy.Subscriber('/enable_flag', Bool, self.enable_callback,queue_size=1,tcp_nodelay=True)
        rospy.spin()

    def pos_callback(self, pos_data):
        """机械臂末端位姿订阅回调函数

        Args:
            pos_data (): 
        """
        factor = 180 / 3.1415926
        x = round(pos_data.x*1000) * 1000
        y = round(pos_data.y*1000) * 1000
        z = round(pos_data.z*1000) * 1000
        rx = round(pos_data.roll*1000*factor) 
        ry = round(pos_data.pitch*1000*factor)
        rz = round(pos_data.yaw*1000*factor)
        rospy.loginfo("Received PosCmd:")
        rospy.loginfo("x: %f", x)
        rospy.loginfo("y: %f", y)
        rospy.loginfo("z: %f", z)
        rospy.loginfo("roll: %f", rx)
        rospy.loginfo("pitch: %f", ry)
        rospy.loginfo("yaw: %f", rz)
        rospy.loginfo("gripper: %f", pos_data.gripper)
        rospy.loginfo("mode1: %d", pos_data.mode1)
        rospy.loginfo("mode2: %d", pos_data.mode2)
        if(self.GetEnableFlag()):
            self._checked_sdk_write('MotionCtrl_1', 0x00, 0x00, 0x00)
            self._checked_sdk_write('MotionCtrl_2', 0x01, 0x00, 50)
            self._checked_sdk_write('EndPoseCtrl', x, y, z, 
                                    rx, ry, rz)
            gripper = round(pos_data.gripper*1000*1000)
            if(pos_data.gripper>80000): gripper = 80000
            if(pos_data.gripper<0): gripper = 0
            if(self.gripper_exist):
                self._checked_sdk_write('GripperCtrl', abs(gripper), 1000, 0x01, 0)
            self._checked_sdk_write('MotionCtrl_2', 0x01, 0x00, 50)
    
    def joint_callback(self, joint_data):
        if self.mode == 0:
            # Do not queue alignment behind a long FA transition: replaying it
            # later would put a released master back into CAN position mode.
            if not self._mode_transition_lock.acquire(False):
                return
            try:
                if self._transition_in_progress or self.in_teach_mode:
                    return
                self._apply_joint_command(joint_data)
            except Exception:
                with self._readiness_lock:
                    self._last_joint_command_valid = False
                self._invalidate_native_preparation('Master alignment callback failed')
                raise
            finally:
                self._mode_transition_lock.release()
        else:
            try:
                self._apply_joint_command(joint_data)
            except Exception:
                with self._readiness_lock:
                    self._last_joint_command_valid = False
                raise

    def _apply_joint_command(self, joint_data):
        """机械臂关节角回调函数

        Args:
            joint_data (): 
        """
        import os
        import datetime

        if (len(joint_data.position) != 7
                or not all(math.isfinite(value) for value in joint_data.position)):
            raise RuntimeError("Joint command must contain seven finite values")
        command_source_time = 0.0
        try:
            command_source_time = float(joint_data.header.stamp.to_sec())
        except (AttributeError, TypeError, ValueError):
            pass
        if self.require_follower_ready and (
                not math.isfinite(command_source_time) or command_source_time <= 0
                or not -0.05 <= time.time() - command_source_time <= 10.0):
            raise RuntimeError("Joint command source timestamp is invalid/stale/future")
        
        factor = 57324.840764 #1000*180/3.14
        factor1 = 57.32484
        
        # # 显示信息（保持原有的）
        # rospy.loginfo("Received Joint States:")
        # rospy.loginfo("joint_0: %f", joint_data.position[0]*1)
        # rospy.loginfo("joint_1: %f", joint_data.position[1]*1)
        # rospy.loginfo("joint_2: %f", joint_data.position[2]*1)
        # rospy.loginfo("joint_3: %f", joint_data.position[3]*1)
        # rospy.loginfo("joint_4: %f", joint_data.position[4]*1)
        # rospy.loginfo("joint_5: %f", joint_data.position[5]*1)
        # rospy.loginfo("joint_6: %f", joint_data.position[6]*1)
        
        # 计算转换后的关节值
        joint_0 = round(joint_data.position[0]*factor)
        joint_1 = round(joint_data.position[1]*factor)
        joint_2 = round(joint_data.position[2]*factor)
        joint_3 = round(joint_data.position[3]*factor)
        joint_4 = round(joint_data.position[4]*factor)
        joint_5 = round(joint_data.position[5]*factor)
        joint_6 = round(joint_data.position[6]*1000*1000)
        if(joint_6>80000): joint_6 = 80000
        if(joint_6<0): joint_6 = 0
        if self.mode == 0:
            new_key = (joint_0, joint_1, joint_2, joint_3, joint_4, joint_5)
            if self._alignment_command_key() != new_key:
                self._invalidate_native_preparation('Master alignment target changed')
                with self._readiness_lock:
                    self._last_joint_command_valid = False
        
        # # 🔧 新增：保存数据到txt文件
        # try:
        #     # 创建保存目录（如果不存在）
        #     save_dir = os.path.expanduser("/home/agilex/cobot_magic/Piper_ros_private-ros-noetic_debug/traj")
        #     if not os.path.exists(save_dir):
        #         os.makedirs(save_dir)
            
        #     # 文件路径
        #     txt_file_path = os.path.join(save_dir, "joint_states.txt")
            
        #     # 获取当前时间戳
        #     timestamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
            
        #     # 准备写入的数据
        #     raw_data = "RAW: %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f" % (
        #         joint_data.position[0], joint_data.position[1], joint_data.position[2],
        #         joint_data.position[3], joint_data.position[4], joint_data.position[5], joint_data.position[6]
        #     )
            
        #     converted_data = "CONVERTED: %d, %d, %d, %d, %d, %d, %d" % (
        #         joint_0, joint_1, joint_2, joint_3, joint_4, joint_5, joint_6
        #     )
            
        #     degrees_data = "DEGREES: %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.6f" % (
        #         joint_data.position[0]*57.32484, joint_data.position[1]*57.32484, joint_data.position[2]*57.32484,
        #         joint_data.position[3]*57.32484, joint_data.position[4]*57.32484, joint_data.position[5]*57.32484, joint_data.position[6]
        #     )
            
        #     # 写入文件
        #     with open(txt_file_path, 'a', encoding='utf-8') as f:
        #         f.write(f"[{timestamp}] {raw_data}\n")
        #         f.write(f"[{timestamp}] {converted_data}\n")
        #         f.write(f"[{timestamp}] {degrees_data}\n")
        #         f.write(f"[{timestamp}] ENABLE_FLAG: {self.GetEnableFlag()}\n")
        #         f.write("-" * 80 + "\n")
            
        #     # 每100次记录打印一次保存信息（避免日志过多）
        #     if not hasattr(self, '_save_count'):
        #         self._save_count = 0
        #     self._save_count += 1
            
        #     if self._save_count % 100 == 1:  # 第1次和每100次
        #         rospy.loginfo("Joint data saved to: %s (count: %d)", txt_file_path, self._save_count)
                
        # except Exception as e:
        #     rospy.logwarn("Failed to save joint data to txt file: %s", str(e))
        
        # 原有的控制逻辑（保持不变）
        if(self.GetEnableFlag()):
            self._checked_sdk_write('MotionCtrl_2', 0x01, 0x01, 100)
            self._checked_sdk_write('JointCtrl', joint_0, joint_1, joint_2, 
                                    joint_3, joint_4, joint_5)
            self._checked_sdk_write('GripperCtrl', abs(joint_6), 1000, 0x01, 0)
            self._checked_sdk_write('MotionCtrl_2', 0x01, 0x01, 100)
            with self._readiness_lock:
                # A queued old command must not impersonate the collector's
                # new probe merely because its callback completed afterward.
                self._last_joint_command_wall = command_source_time
                self._last_joint_command_valid = True
                if self.mode == 0:
                    self._last_alignment_target = (joint_0, joint_1, joint_2, joint_3, joint_4, joint_5)
                    self._last_alignment_command_gripper_um = joint_6
            pass
    
    def enable_callback(self, enable_flag:Bool):
        """机械臂使能回调函数

        Args:
            enable_flag (): 
        """
        rospy.loginfo("Received enable flag:")
        rospy.loginfo("enable_flag: %s", enable_flag.data)
        if(enable_flag.data):
            self.__enable_flag = True
            self._checked_sdk_write('EnableArm', 7)
            self._checked_sdk_write('GripperCtrl', 0,1000,0x02, 0)
            self._checked_sdk_write('GripperCtrl', 0,1000,0x01, 0)
        else:
            self.__enable_flag = False
            self.piper.DisableArm(7)
            self._checked_sdk_write('GripperCtrl', 0,1000,0x00, 0)
    
    def handle_master_go_zero_service(self,req):
        response = TriggerResponse()
        rospy.loginfo(f"-----------------------RESET---------------------------")
        rospy.loginfo(f"{self.can_port} send piper master go zero service")
        rospy.loginfo(f"-----------------------RESET---------------------------")
        self._checked_sdk_write('ReqMasterArmMoveToHome', 1)
        response.success = True
        response.message = str({self.can_port}) + "send piper master go zero service success"
        rospy.loginfo(f"Returning resetResponse: {response.success}, {response.message}")
        return response

    def handle_master_slave_go_zero_service(self,req):
        response = TriggerResponse()
        rospy.loginfo(f"-----------------------RESET---------------------------")
        rospy.loginfo(f"{self.can_port} send piper master slave go zero service")
        rospy.loginfo(f"-----------------------RESET---------------------------")
        self._checked_sdk_write('ReqMasterArmMoveToHome', 2)
        response.success = True
        response.message = str({self.can_port}) + "send piper master slave go zero service success"
        rospy.loginfo(f"Returning resetResponse: {response.success}, {response.message}")
        return response
    
    def handle_restore_ms_mode_service(self,req):
        response = TriggerResponse()
        rospy.loginfo(f"-----------------------RESET---------------------------")
        rospy.loginfo(f"{self.can_port} send piper restore master slave mode service")
        rospy.loginfo(f"-----------------------RESET---------------------------")
        self._checked_sdk_write('ReqMasterArmMoveToHome', 0)
        response.success = True
        response.message = str({self.can_port}) + "send piper restore master slave mode service success"
        rospy.loginfo(f"Returning resetResponse: {response.success}, {response.message}")
        return response

if __name__ == '__main__':
    try:
        piper_ms = C_PiperRosNode()
        
        # 🆕 新增：启动时打印使用说明
        if piper_ms.mode == 0:
            rospy.loginfo("=" * 60)
            rospy.loginfo("🎯 增强版主从控制功能已集成！")
            rospy.loginfo("=" * 60)
            rospy.loginfo("📋 Launch参数:")
            rospy.loginfo(f"  - mode: {piper_ms.mode} (0=主臂读取模式)")
            rospy.loginfo(f"  - mode_master: {piper_ms.mode_master} ({'主臂示教模式' if piper_ms.mode_master else '从臂跟随模式'})")
            rospy.loginfo(f"  - 当前配置: 0x{piper_ms.current_linkage_config:02X}")
            rospy.loginfo("=" * 60)
            rospy.loginfo("📋 ROS控制指令:")
            rospy.loginfo("  1. 使能: rostopic pub /master/enable std_msgs/Bool 'data: true'")
            rospy.loginfo("  2. 失能: rostopic pub /master/enable std_msgs/Bool 'data: false'")
            rospy.loginfo("  3. 示教: rostopic pub /master/teach_mode std_msgs/Int32 'data: 1'")
            rospy.loginfo("  4. 控制: rostopic pub /master/teach_mode std_msgs/Int32 'data: 0'")
            rospy.loginfo("  🎯 在线切换主从配置:")
            rospy.loginfo("     - 主臂示教: rostopic pub /master/linkage_config std_msgs/String 'data: master'")
            rospy.loginfo("     - 从臂跟随: rostopic pub /master/linkage_config std_msgs/String 'data: slave'")
            rospy.loginfo("     - 十六进制: rostopic pub /master/linkage_config std_msgs/String 'data: 0xFA'")
            rospy.loginfo("=" * 60)
        elif piper_ms.mode == 1:
            rospy.loginfo("🤖 从臂模式启动 - 等待接收控制指令")
        
        piper_ms.Pubilsh()
    except rospy.ROSInterruptException:
        pass
