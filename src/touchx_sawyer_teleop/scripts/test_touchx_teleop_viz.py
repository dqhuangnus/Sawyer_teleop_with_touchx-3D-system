#!/usr/bin/env python3 
"""
TouchX teleoperation with RViz visualization (no Gazebo needed).

IK joint angles are published to /joint_states -> robot_state_publisher -> TF -> RViz.

Markers shown in RViz:
  - Green sphere : target end-effector position
  - Red sphere   : actual end-effector position (from TF)
  - White text   : live position and error readout
  - Grey bowl    : physical sensor bowl on table
  - Orange box   : teleoperation workspace boundary

Usage:
  Terminal 1: roslaunch omni_common omni_state.launch
  Terminal 2: roslaunch touchx_sawyer_teleop test_touchx_viz.launch
"""

import sys
import os
import math
import threading
import importlib.util
import intera_interface

import rospy
import tf2_ros
import tf.transformations as tft
from geometry_msgs.msg import PoseStamped, Point
from omni_msgs.msg import OmniButtonEvent, OmniFeedback
from sensor_msgs.msg import JointState
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

# Data-collection helpers (live alongside this script in scripts/).
# Gracefully degrade if the capture deps (pypylon / websocket-client / h5py)
# are not installed — teleop still works, recording is just disabled.
try:
    from sensors import TactileReader, BaslerCameraManager
    from data_recorder import DataRecorder
    _RECORDING_AVAILABLE = True
except Exception:
    _RECORDING_AVAILABLE = False


# ── RelaxedIK solver ──────────────────────────────────────────────────────────
RELAXED_IK_PATH     = '/root/sawyer_haptic_workspace/src/relaxed_ik_core'
RELAXED_IK_WRAPPER  = RELAXED_IK_PATH + '/wrappers/python_wrapper.py'
RELAXED_IK_SETTINGS = RELAXED_IK_PATH + '/configs/settings.yaml'

rik_spec   = importlib.util.spec_from_file_location("python_wrapper", RELAXED_IK_WRAPPER)
rik_module = importlib.util.module_from_spec(rik_spec)
rik_spec.loader.exec_module(rik_module)
RelaxedIKRust = rik_module.RelaxedIKRust

# ── Robot joint names ─────────────────────────────────────────────────────────
ARM_JOINTS = [
    'right_j0', 'right_j1', 'right_j2',
    'right_j3', 'right_j4', 'right_j5', 'right_j6'
]
GRIPPER_JOINTS = [
    'finger_joint',
    'right_outer_knuckle_joint',
    'left_inner_knuckle_joint',  'right_inner_knuckle_joint',
    'left_inner_finger_joint',   'right_inner_finger_joint',
]
GRIPPER_OPEN_POS   = 0.0
GRIPPER_CLOSED_POS = 0.8

DEFAULT_HOME_JOINTS = [0.3474, -1.3143, -0.5663, 1.3630, 0.0967, 1.4469, 3.0276]

BASE_FRAME     = 'reference/base'
EE_FRAME       = 'reference/right_hand'
GRIPPER_LENGTH = 0.212   # wrist to fingertip, metres

# ── Bowl geometry (metres) ───────────────────────────────────────────────────
BOWL_OUTER_SIZE   = 0.80   # full width of outer rim
BOWL_BASE_SIZE    = 0.24   # flat centre square
BOWL_SLOPE_HEIGHT = 0.09   # height slopes rise from centre to rim
BOWL_THICKNESS    = 0.005  # panel thickness
BRACKET_HEIGHT    = 0.09   # corner bracket vertical leg
BRACKET_WIDTH     = 0.05   # corner bracket horizontal base


# ── Marker factory helpers ────────────────────────────────────────────────────
def make_sphere(ns, marker_id, r, g, b, size=0.025):
    m = Marker()
    m.header.frame_id = BASE_FRAME
    m.ns, m.id = ns, marker_id
    m.type = Marker.SPHERE
    m.action = Marker.ADD
    m.scale.x = m.scale.y = m.scale.z = size
    m.color = ColorRGBA(r, g, b, 1.0)
    m.pose.orientation.w = 1.0
    return m

def make_text(ns, marker_id):
    m = Marker()
    m.header.frame_id = BASE_FRAME
    m.ns, m.id = ns, marker_id
    m.type = Marker.TEXT_VIEW_FACING
    m.action = Marker.ADD
    m.scale.z = 0.03
    m.color = ColorRGBA(1.0, 1.0, 1.0, 1.0)
    m.pose.orientation.w = 1.0
    m.pose.position.x, m.pose.position.z = 0.4, 0.65
    return m

def build_bowl_markers(cx, cy, cz):
    """
    Build physical sensor bowl geometry markers.
    cx, cy = horizontal centre; cz = height of flat base.
    Bowl opens upward: centre is low, outer rim is BOWL_SLOPE_HEIGHT above it.
    """
    markers = []
    uid = 100

    GREY = ColorRGBA(0.45, 0.45, 0.50, 0.90)
    WOOD = ColorRGBA(0.87, 0.80, 0.60, 1.00)

    half_outer = BOWL_OUTER_SIZE / 2
    half_base  = BOWL_BASE_SIZE  / 2
    thick      = BOWL_THICKNESS

    def cube(lx, ly, lz, sx, sy, sz, color, ns):
        nonlocal uid
        m = Marker()
        m.header.frame_id    = BASE_FRAME
        m.ns, m.id           = ns, uid;  uid += 1
        m.type               = Marker.CUBE
        m.action             = Marker.ADD
        m.pose.position.x    = float(cx + lx)
        m.pose.position.y    = float(cy + ly)
        m.pose.position.z    = float(cz + lz)
        m.pose.orientation.w = 1.0
        m.scale.x, m.scale.y, m.scale.z = float(sx), float(sy), float(sz)
        m.color = color
        return m

    def tri_list(tris, color, ns):
        nonlocal uid
        m = Marker()
        m.header.frame_id    = BASE_FRAME
        m.ns, m.id           = ns, uid;  uid += 1
        m.type               = Marker.TRIANGLE_LIST
        m.action             = Marker.ADD
        m.pose.position.x    = float(cx)
        m.pose.position.y    = float(cy)
        m.pose.position.z    = float(cz)
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = 1.0
        m.color = color
        for tri in tris:
            for v in tri:
                p = Point()
                p.x, p.y, p.z = float(v[0]), float(v[1]), float(v[2])
                m.points.append(p)
        return m

    # Flat base square
    markers.append(cube(
        lx=0, ly=0, lz=thick/2,
        sx=BOWL_BASE_SIZE, sy=BOWL_BASE_SIZE, sz=thick,
        color=GREY, ns="bowl_base"))

    # Four sloped side panels
    slope_defs = [
        dict(il=(-half_base,  half_base, 0), ir=( half_base,  half_base, 0),
             ol=(-half_outer, half_outer, BOWL_SLOPE_HEIGHT), or_=( half_outer, half_outer, BOWL_SLOPE_HEIGHT)),
        dict(il=(-half_base, -half_base, 0), ir=( half_base, -half_base, 0),
             ol=(-half_outer,-half_outer, BOWL_SLOPE_HEIGHT), or_=( half_outer,-half_outer, BOWL_SLOPE_HEIGHT)),
        dict(il=(-half_base, -half_base, 0), ir=(-half_base,  half_base, 0),
             ol=(-half_outer,-half_outer, BOWL_SLOPE_HEIGHT), or_=(-half_outer, half_outer, BOWL_SLOPE_HEIGHT)),
        dict(il=( half_base, -half_base, 0), ir=( half_base,  half_base, 0),
             ol=( half_outer,-half_outer, BOWL_SLOPE_HEIGHT), or_=( half_outer, half_outer, BOWL_SLOPE_HEIGHT)),
    ]
    slope_tris = []
    for d in slope_defs:
        il, ir, ol, or_ = d['il'], d['ir'], d['ol'], d['or_']
        slope_tris.append((il, ir, ol))
        slope_tris.append((ir, or_, ol))
    markers.append(tri_list(slope_tris, GREY, "bowl_slopes"))

    # Outer rim lip
    rim_z = BOWL_SLOPE_HEIGHT + thick / 2
    for rim in [
        dict(lx=0,          ly= half_outer, lz=rim_z, sx=BOWL_OUTER_SIZE, sy=thick, sz=thick),
        dict(lx=0,          ly=-half_outer, lz=rim_z, sx=BOWL_OUTER_SIZE, sy=thick, sz=thick),
        dict(lx=-half_outer, ly=0,          lz=rim_z, sx=thick, sy=BOWL_OUTER_SIZE, sz=thick),
        dict(lx= half_outer, ly=0,          lz=rim_z, sx=thick, sy=BOWL_OUTER_SIZE, sz=thick),
    ]:
        markers.append(cube(lx=rim['lx'], ly=rim['ly'], lz=rim['lz'],
                            sx=rim['sx'], sy=rim['sy'], sz=rim['sz'],
                            color=GREY, ns="bowl_rim"))

    # Corner support brackets
    bracket_tris = []
    for (bx, by, inward_x, inward_y) in [
        ( half_outer,  half_outer, -1,  0),
        (-half_outer,  half_outer, +1,  0),
        ( half_outer, -half_outer, -1,  0),
        (-half_outer, -half_outer, +1,  0),
    ]:
        bottom = (bx,                            by, 0.0)
        top    = (bx,                            by, BRACKET_HEIGHT)
        foot   = (bx + inward_x * BRACKET_WIDTH, by, 0.0)
        bracket_tris.append((bottom, top, foot))
    markers.append(tri_list(bracket_tris, WOOD, "bowl_brackets"))

    return markers


# ── Main teleoperation node ───────────────────────────────────────────────────
class TeleopVizNode:
    def __init__(self):
        rospy.init_node('test_touchx_teleop_viz', anonymous=False)

        # Movement and filtering
        self.position_scale     = rospy.get_param('~position_scale', 1.5)
        self.control_rate       = rospy.get_param('~control_rate',   100.0)
        self.smooth_hz          = rospy.get_param('~smooth_hz',      3.0)
        self.joint_filter_alpha = rospy.get_param('~joint_filter_alpha', 0.40)
        self.max_joint_step     = rospy.get_param('~max_joint_step',     0.10)
        self.joint_speed_ratio  = rospy.get_param('~joint_speed_ratio',  0.20)
        self.workspace_centre   = rospy.get_param('~workspace_centre', [0.70, 0.0, 0.20])

        # Adaptive XY scaling near bowl surface
        self.far_dist   = rospy.get_param('~far_dist',   0.20)
        self.near_dist  = rospy.get_param('~near_dist',  0.08)
        self.min_scale  = rospy.get_param('~min_scale',  0.25)

        # Orientation scale per axis
        self.roll_scale  = rospy.get_param('~roll_scale',  0.8)
        self.pitch_scale = rospy.get_param('~pitch_scale', 0.8)
        self.yaw_scale   = rospy.get_param('~yaw_scale',   0.8)

        # ── Data collection (Basler ×3 + uSkin tactile) ───────────────────
        self.record_enabled   = rospy.get_param('~record_enabled', True)
        self.record_rate      = rospy.get_param('~record_rate', 20.0)
        self.save_dir         = rospy.get_param('~save_dir', '/root/collected_data')
        self.xela_ws_url      = rospy.get_param('~xela_ws_url', 'ws://localhost:5000')
        self.n_taxels         = rospy.get_param('~tactile_taxels', 24)
        self.tac_hist         = rospy.get_param('~tactile_history', 5)
        self.camera_ips       = rospy.get_param('~camera_ips', {
            'image_left':  '192.168.1.130',
            'image_right': '192.168.1.120',
            'image_top':   '192.168.1.100'})
        self.cam_scale        = rospy.get_param('~camera_scale',   0.5)
        self.cam_binning      = rospy.get_param('~camera_binning', 2)
        self.cam_fps          = rospy.get_param('~camera_fps',     10)
        self.record_realsense = rospy.get_param('~record_realsense', False)

        # Bowl position: [x, y, z] of the flat base centre
        raw_bowl = rospy.get_param('~sensor_bowl_pos', [0.70, 0.0, 0.05])
        if isinstance(raw_bowl, str):
            import ast
            raw_bowl = ast.literal_eval(raw_bowl)
        self.bowl_pos = [float(v) for v in raw_bowl]

        self.home_joints = list(DEFAULT_HOME_JOINTS)
        rospy.loginfo("[teleop] Home joints: %s", self.home_joints)

        # TF
        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Joint state publisher (drives robot_state_publisher and the real robot)
        self.joint_pub       = rospy.Publisher('/joint_states', JointState, queue_size=5)
        self.current_joints  = list(self.home_joints)
        self.smoothed_joints = list(self.home_joints)

        # RelaxedIK solver
        rospy.loginfo("[teleop] Loading RelaxedIK...")
        saved_dir = os.getcwd()
        os.chdir(RELAXED_IK_PATH)
        try:
            self.rik = RelaxedIKRust(setting_file_path=RELAXED_IK_SETTINGS)
        finally:
            os.chdir(saved_dir)
        rospy.loginfo("[teleop] RelaxedIK ready")

        # Sawyer arm interface
        self.arm = intera_interface.Limb('right')
        # Lower joint position speed ratio = controller follows targets less
        # aggressively, which kills the "dud dud" on contact during pick/place.
        try:
            self.arm.set_joint_position_speed(self.joint_speed_ratio)
            rospy.loginfo("[teleop] Joint position speed ratio set to %.2f",
                          self.joint_speed_ratio)
        except Exception as e:
            rospy.logwarn("[teleop] Could not set joint position speed: %s", e)

        # Gripper (optional — falls back gracefully if not connected)
        try:
            from pyrobotiqgripper import RobotiqGripper
            self.gripper = RobotiqGripper()
            self.gripper.activate()
            rospy.sleep(0.5)
            self.gripper_ready = True
            rospy.loginfo("[teleop] Gripper activated")
        except Exception as e:
            self.gripper_ready = False
            rospy.logwarn("[teleop] Gripper not available: %s", e)
        self.gripper_open = False

        # Visualization markers
        self.viz_pub       = rospy.Publisher('/teleop_viz', Marker, queue_size=20)
        self.goal_sphere   = make_sphere("goal",   0, 0.0, 1.0, 0.0, 0.03)
        self.actual_sphere = make_sphere("actual", 1, 1.0, 0.0, 0.0, 0.03)
        self.info_text     = make_text("info", 4)

        # Workspace boundary box (transparent fill + red edges)
        self.workspace_fill = Marker()
        self.workspace_fill.header.frame_id = BASE_FRAME
        self.workspace_fill.ns, self.workspace_fill.id = "workspace_fill", 10
        self.workspace_fill.type   = Marker.CUBE
        self.workspace_fill.action = Marker.ADD
        self.workspace_fill.scale.x = 0.80
        self.workspace_fill.scale.y = 0.80
        self.workspace_fill.scale.z = 0.40
        self.workspace_fill.color   = ColorRGBA(1.0, 0.5, 0.0, 0.15)
        self.workspace_fill.pose.orientation.w = 1.0

        self.workspace_edges = Marker()
        self.workspace_edges.header.frame_id = BASE_FRAME
        self.workspace_edges.ns, self.workspace_edges.id = "workspace_edges", 11
        self.workspace_edges.type   = Marker.LINE_LIST
        self.workspace_edges.action = Marker.ADD
        self.workspace_edges.scale.x = 0.005
        self.workspace_edges.color   = ColorRGBA(1.0, 0.0, 0.0, 1.0)
        self.workspace_edges.pose.orientation.w = 1.0

        # Table markers
        self.table_top = Marker()
        self.table_top.header.frame_id = BASE_FRAME
        self.table_top.ns, self.table_top.id = "table", 20
        self.table_top.type   = Marker.CUBE
        self.table_top.action = Marker.ADD
        self.table_top.scale.x = 1.80
        self.table_top.scale.y = 1.20
        self.table_top.scale.z = 0.05
        self.table_top.color   = ColorRGBA(0.55, 0.40, 0.25, 1.0)
        self.table_top.pose.orientation.w = 1.0

        self.table_legs = []
        for i in range(4):
            leg = Marker()
            leg.header.frame_id = BASE_FRAME
            leg.ns, leg.id = "table_leg", 30 + i
            leg.type   = Marker.CYLINDER
            leg.action = Marker.ADD
            leg.scale.x = leg.scale.y = 0.06
            leg.scale.z = 0.80
            leg.color   = ColorRGBA(0.55, 0.40, 0.25, 1.0)
            leg.pose.orientation.w = 1.0
            self.table_legs.append(leg)

        self.wood_box = Marker()
        self.wood_box.header.frame_id = BASE_FRAME
        self.wood_box.ns, self.wood_box.id = "wood_box", 40
        self.wood_box.type   = Marker.CUBE
        self.wood_box.action = Marker.ADD
        self.wood_box.scale.x = 0.24
        self.wood_box.scale.y = 0.24
        self.wood_box.scale.z = 0.05
        self.wood_box.color   = ColorRGBA(0.82, 0.65, 0.40, 1.0)
        self.wood_box.pose.orientation.w = 1.0

        # Teleop state
        self.lock           = threading.Lock()
        self.haptic_pose    = None   # latest filtered haptic device pose
        self.last_stamp     = None
        self.active         = False  # True while grey button is held
        self.anchor_haptic  = None   # haptic position when button was pressed
        self.anchor_robot   = None   # robot EE position when button was pressed
        self.home_pos       = None
        self.home_quat      = None
        self.last_goal      = None
        self.anchor_yaw     = None
        self.anchor_quat    = None

        # Sensors + episode recorder (graceful if hardware/deps absent)
        self.recorder  = None
        self.recording = False
        self.ep_count  = 0
        self._init_recording()

        rospy.Subscriber('/phantom/pose',   PoseStamped,     self._on_haptic_pose, queue_size=1)
        rospy.Subscriber('/phantom/button', OmniButtonEvent, self._on_button,      queue_size=1)
        self.haptic_pub = rospy.Publisher('/phantom/force_feedback', OmniFeedback, queue_size=1)

    # ── Euler angles to quaternion [x, y, z, w] ───────────────────────────────
    @staticmethod
    def euler_to_quat(roll, pitch, yaw):
        cy, sy = math.cos(yaw   * 0.5), math.sin(yaw   * 0.5)
        cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
        cr, sr = math.cos(roll  * 0.5), math.sin(roll  * 0.5)
        return [
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy,
        ]

    # ── Data collection: start sensors + build recorder ───────────────────────
    def _init_recording(self):
        if not self.record_enabled:
            rospy.loginfo("[teleop] Recording disabled (record_enabled=false)")
            return
        if not _RECORDING_AVAILABLE:
            rospy.logwarn("[teleop] Recording deps missing (pypylon/websocket/h5py) — disabled")
            return

        # uSkin tactile (xela_server websocket)
        self.tactile = None
        try:
            self.tactile = TactileReader(ws_url=self.xela_ws_url,
                                         n_per_finger=self.n_taxels,
                                         history_len=self.tac_hist)
            self.tactile.start()
            rospy.loginfo("[teleop] Tactile reader started (%s)", self.xela_ws_url)
        except Exception as e:
            rospy.logwarn("[teleop] Tactile unavailable: %s", e)

        # Basler GigE cameras
        self.basler = None
        try:
            self.basler = BaslerCameraManager(self.camera_ips, scale=self.cam_scale,
                                              binning=self.cam_binning, fps=self.cam_fps)
            self.basler.start_bg()
            rospy.loginfo("[teleop] Basler cameras: %s", self.basler.names)
        except Exception as e:
            rospy.logwarn("[teleop] Basler cameras unavailable: %s", e)

        # RealSense (optional — off by default)
        self.realsense = None
        if self.record_realsense:
            try:
                from realsense_camera import RealSenseCamera
                self.realsense = RealSenseCamera()
                self.realsense.start_bg()
                rospy.loginfo("[teleop] RealSense started")
            except Exception as e:
                rospy.logwarn("[teleop] RealSense unavailable: %s", e)

        gripper = self.gripper if getattr(self, 'gripper_ready', False) else None
        self.recorder = DataRecorder(
            limb=self.arm, gripper=gripper, tactile=self.tactile,
            basler=self.basler, realsense=self.realsense,
            rate_hz=self.record_rate, save_dir=self.save_dir)
        rospy.loginfo("[teleop] Recorder ready -> %s  (r=record  f=finish+save  d=discard)",
                      self.save_dir)

    def _handle_record_key(self, key):
        if self.recorder is None:
            rospy.logwarn_throttle(2.0, "[teleop] recording not available")
            return
        if key == 'r' and not self.recording:
            self.recorder.start()
            self.recording = True
            rospy.loginfo("[teleop] >>> RECORDING episode")
        elif key == 'f' and self.recording:
            self.recorder.stop()
            self.recording = False
            path = self.recorder.save(tag="ep%03d" % self.ep_count)
            if path:
                self.ep_count += 1
            rospy.loginfo("[teleop] <<< saved %d frames", len(self.recorder))
        elif key == 'd' and self.recording:
            self.recorder.stop()
            self.recording = False
            rospy.loginfo("[teleop] episode discarded")

    def _shutdown_recording(self):
        if self.recording and self.recorder is not None:
            self.recorder.stop()
        for obj in (getattr(self, 'tactile', None), getattr(self, 'basler', None),
                    getattr(self, 'realsense', None)):
            try:
                if obj is not None:
                    obj.stop()
            except Exception:
                pass

    # ── Send joint angles to robot and RViz ───────────────────────────────────
    def _send_joints(self, angles, move_robot=True):
        grip = GRIPPER_OPEN_POS if self.gripper_open else GRIPPER_CLOSED_POS
        gripper_positions = [grip, grip, grip, grip, -grip, -grip]

        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        msg.name     = ARM_JOINTS + GRIPPER_JOINTS
        msg.position = list(angles) + gripper_positions
        msg.velocity = [0.0] * (7 + len(GRIPPER_JOINTS))
        msg.effort   = [0.0] * (7 + len(GRIPPER_JOINTS))
        self.joint_pub.publish(msg)

        if move_robot:
            self.arm.set_joint_positions(dict(zip(ARM_JOINTS, angles)))
        self.current_joints = list(angles)

    def _move_to_home(self):
        rospy.loginfo("[teleop] Moving to home position...")
        self.arm.move_to_joint_positions(dict(zip(ARM_JOINTS, self.home_joints)), timeout=10.0)
        self.smoothed_joints = list(self.home_joints)
        rospy.loginfo("[teleop] Reached home position")

    # ── Read end-effector pose from TF ────────────────────────────────────────
    def _get_ee_pose(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                BASE_FRAME, EE_FRAME, rospy.Time(0), rospy.Duration(1.0))
            t = tf.transform.translation
            r = tf.transform.rotation
            return [t.x, t.y, t.z], [r.x, r.y, r.z, r.w]
        except Exception:
            return None, None

    # ── Haptic pose callback: EMA low-pass filter on position ─────────────────
    def _on_haptic_pose(self, msg):
        with self.lock:
            if self.haptic_pose is None:
                self.haptic_pose = msg
                self.last_stamp  = msg.header.stamp
                return
            dt = (msg.header.stamp - self.last_stamp).to_sec()
            self.last_stamp = msg.header.stamp
            if dt <= 0:
                dt = 0.001
            tau   = 1.0 / (2.0 * math.pi * self.smooth_hz)
            alpha = dt / (tau + dt)
            filt = self.haptic_pose.pose.position
            new  = msg.pose.position
            filt.x += alpha * (new.x - filt.x)
            filt.y += alpha * (new.y - filt.y)
            filt.z += alpha * (new.z - filt.z)
            self.haptic_pose.header = msg.header
            # Slerp orientation with the same alpha so wrist jitter on the
            # stylus stops driving joints 5 / 6 directly.
            fo = self.haptic_pose.pose.orientation
            no = msg.pose.orientation
            q_old = [fo.x, fo.y, fo.z, fo.w]
            q_new = [no.x, no.y, no.z, no.w]
            try:
                qs = tft.quaternion_slerp(q_old, q_new, alpha)
                fo.x, fo.y, fo.z, fo.w = qs[0], qs[1], qs[2], qs[3]
            except Exception:
                fo.x, fo.y, fo.z, fo.w = no.x, no.y, no.z, no.w

    # ── Button callback ────────────────────────────────────────────────────────
    def _on_button(self, msg):
        if msg.grey_button == 1:
            # Reset RelaxedIK from current joint state so it continues smoothly
            self.rik.reset(list(self.current_joints))

            with self.lock:
                pose_snap = self.haptic_pose

            haptic_anchor_pos = None
            if pose_snap is not None:
                haptic_anchor_pos = [pose_snap.pose.position.x,
                                     pose_snap.pose.position.y,
                                     pose_snap.pose.position.z]

            # Robot anchor = current EE position (movement is relative to this)
            ee_pos, _ = self._get_ee_pose()
            robot_anchor_pos = ee_pos if ee_pos is not None else (
                list(self.home_pos) if self.home_pos else list(self.workspace_center))

            init_yaw  = 0.0
            init_quat = list(self.home_quat) if self.home_quat else [0.0, 0.0, 0.0, 1.0]
            if pose_snap is not None:
                q = pose_snap.pose.orientation
                _, _, init_yaw = tft.euler_from_quaternion([q.x, q.y, q.z, q.w])
                init_quat      = [q.x, q.y, q.z, q.w]

            with self.lock:
                self.active          = True
                self.anchor_haptic   = haptic_anchor_pos
                self.anchor_robot    = robot_anchor_pos
                self.anchor_yaw      = init_yaw
                self.anchor_quat     = init_quat
                # Re-seed joint smoother so the first cycle after engaging
                # doesn't snap from a stale buffer.
                self.smoothed_joints = list(self.current_joints)
            rospy.loginfo("[teleop] Grey button pressed — anchor: %.3f %.3f %.3f", *robot_anchor_pos)
        else:
            with self.lock:
                self.active        = False
                self.anchor_haptic = None
            rospy.loginfo("[teleop] Grey button released — holding position")

        if msg.white_button == 1 and self.gripper_ready:
            if self.gripper_open:
                self.gripper.close()
                self.gripper_open = False
                rospy.loginfo("[teleop] Gripper CLOSED")
            else:
                self.gripper.open()
                self.gripper_open = True
                rospy.loginfo("[teleop] Gripper OPEN")

    # ── Compute target EE position from haptic input ──────────────────────────
    def _compute_target_pos(self, haptic_pose, haptic_anchor, robot_anchor, ee_pos=None):
        pos = haptic_pose.pose.position
        dhx = pos.x - haptic_anchor[0]
        dhy = pos.y - haptic_anchor[1]
        dhz = pos.z - haptic_anchor[2]

        # XY scale reduces when EE is close to bowl (finer control near surface)
        bowl_z = float(self.bowl_pos[2]) + GRIPPER_LENGTH
        if ee_pos is not None:
            height_above_bowl = ee_pos[2] - bowl_z
            blend    = max(0.0, min(1.0, (height_above_bowl - self.near_dist) /
                                          max(self.far_dist - self.near_dist, 1e-6)))
            xy_scale = self.position_scale * (self.min_scale + blend * (1.0 - self.min_scale))
        else:
            xy_scale = self.position_scale
        rospy.loginfo_throttle(1.0, "[scale] xy=%.2f (height above bowl=%.3fm)",
                               xy_scale, (ee_pos[2] - bowl_z) if ee_pos else -1)

        # Haptic → robot axis mapping:
        #   haptic +Y (forward away from user) → robot -X (forward)
        #   haptic +X (right)                  → robot +Y (left)
        #   haptic +Z (up)                     → robot +Z (up)
        robot_dx = -dhy * xy_scale
        robot_dy =  dhx * xy_scale
        robot_dz =  dhz * self.position_scale   # Z stays at full scale

        gx = robot_anchor[0] + robot_dx
        gy = robot_anchor[1] + robot_dy
        gz = robot_anchor[2] + robot_dz

        # Clamp to workspace walls and push back with haptic force at boundaries
        home_x, home_y, _ = self.home_pos
        wall_force = 3.0
        fx = fy = fz = 0.0

        if gx < home_x - 0.40: gx = home_x - 0.40; fx =  wall_force; rospy.logwarn_throttle(1.0, "[wall] X min")
        if gx > home_x + 0.40: gx = home_x + 0.40; fx = -wall_force; rospy.logwarn_throttle(1.0, "[wall] X max")
        if gy < home_y - 0.40: gy = home_y - 0.40; fy =  wall_force; rospy.logwarn_throttle(1.0, "[wall] Y min")
        if gy > home_y + 0.40: gy = home_y + 0.40; fy = -wall_force; rospy.logwarn_throttle(1.0, "[wall] Y max")

        bowl_floor = float(self.bowl_pos[2]) + GRIPPER_LENGTH
        rospy.loginfo_throttle(1.0, "[wall] gz=%.4f  floor=%.4f", gz, bowl_floor)
        if gz < bowl_floor:
            gz = bowl_floor
            fz = wall_force
            rospy.logwarn_throttle(0.5, "[wall] Z floor gz=%.4f", gz)

        # Remap force to haptic device axes (mirrors position axis mapping)
        feedback = OmniFeedback()
        feedback.force.x =  fy
        feedback.force.y = -fx
        feedback.force.z =  fz
        self.haptic_pub.publish(feedback)
        rospy.loginfo_throttle(1.0, "[haptic] force: x=%.2f y=%.2f z=%.2f",
                               feedback.force.x, feedback.force.y, feedback.force.z)

        return [gx, gy, gz]

    # ── Publish workspace boundary box ────────────────────────────────────────
    def _publish_workspace_box(self):
        cx, cy, _ = self.home_pos
        xmin, xmax = cx - 0.40, cx + 0.40
        ymin, ymax = cy - 0.40, cy + 0.40
        zmin = float(self.bowl_pos[2])
        zmax = zmin + 0.40

        self.workspace_fill.header.stamp = rospy.Time.now()
        self.workspace_fill.pose.position.x = cx
        self.workspace_fill.pose.position.y = cy
        self.workspace_fill.pose.position.z = zmin + 0.20
        self.viz_pub.publish(self.workspace_fill)

        corners = [
            (xmin, ymin, zmin), (xmax, ymin, zmin), (xmax, ymax, zmin), (xmin, ymax, zmin),
            (xmin, ymin, zmax), (xmax, ymin, zmax), (xmax, ymax, zmax), (xmin, ymax, zmax),
        ]
        edges = [(0,1),(1,2),(2,3),(3,0),(0,4),(1,5),(2,6),(3,7)]
        self.workspace_edges.header.stamp = rospy.Time.now()
        self.workspace_edges.points = []
        for a, b in edges:
            self.workspace_edges.points.append(Point(*corners[a]))
            self.workspace_edges.points.append(Point(*corners[b]))
        self.viz_pub.publish(self.workspace_edges)

    # ── Publish physical sensor bowl ──────────────────────────────────────────
    def _publish_bowl(self):
        cx, cy, _ = self.home_pos
        cz  = float(self.bowl_pos[2])
        now = rospy.Time.now()
        for m in build_bowl_markers(cx, cy, cz):
            m.header.stamp = now
            self.viz_pub.publish(m)

    # ── Publish table, legs and wooden box ────────────────────────────────────
    def _publish_table(self):
        cx, cy, _ = self.home_pos
        now = rospy.Time.now()

        table_z        = 0.0
        table_thick    = 0.05
        table_center_z = table_z - (table_thick / 2.0)

        self.table_top.header.stamp = now
        self.table_top.pose.position.x = cx
        self.table_top.pose.position.y = cy
        self.table_top.pose.position.z = table_center_z
        self.viz_pub.publish(self.table_top)

        leg_height = 0.80
        leg_z      = table_z - table_thick - (leg_height / 2.0)
        leg_offsets = [( 0.80,  0.55), ( 0.80, -0.55),
                       (-0.80,  0.55), (-0.80, -0.55)]
        for leg, (ox, oy) in zip(self.table_legs, leg_offsets):
            leg.header.stamp = now
            leg.pose.position.x = cx + ox
            leg.pose.position.y = cy + oy
            leg.pose.position.z = leg_z
            self.viz_pub.publish(leg)

        self.wood_box.header.stamp = now
        self.wood_box.pose.position.x = cx
        self.wood_box.pose.position.y = cy
        self.wood_box.pose.position.z = table_z + (self.wood_box.scale.z / 2.0)
        self.viz_pub.publish(self.wood_box)

    # ── Publish goal / actual EE markers and info text ────────────────────────
    def _publish_ee_markers(self, goal_pos, ee_pos, is_active):
        now = rospy.Time.now()

        if goal_pos:
            self.goal_sphere.header.stamp = now
            self.goal_sphere.pose.position.x = goal_pos[0]
            self.goal_sphere.pose.position.y = goal_pos[1]
            self.goal_sphere.pose.position.z = goal_pos[2]
            self.viz_pub.publish(self.goal_sphere)

        if ee_pos:
            self.actual_sphere.header.stamp = now
            self.actual_sphere.pose.position.x = ee_pos[0]
            self.actual_sphere.pose.position.y = ee_pos[1]
            self.actual_sphere.pose.position.z = ee_pos[2] - GRIPPER_LENGTH
            self.viz_pub.publish(self.actual_sphere)

        if goal_pos and ee_pos:
            error_m = math.sqrt(sum((a - b) ** 2 for a, b in zip(goal_pos, ee_pos)))
            self.info_text.header.stamp = now
            self.info_text.text = (
                f"{'ENABLED' if is_active else 'DISABLED'}\n"
                f"Goal:   [{goal_pos[0]:.3f}, {goal_pos[1]:.3f}, {goal_pos[2]:.3f}]\n"
                f"Actual: [{ee_pos[0]:.3f}, {ee_pos[1]:.3f}, {ee_pos[2]:.3f}]\n"
                f"Error:  {error_m * 1000:.1f} mm\n"
                f"Joints: [{', '.join(f'{a:.2f}' for a in self.current_joints)}]"
            )
            self.viz_pub.publish(self.info_text)

    # ── Main loop ──────────────────────────────────────────────────────────────
    def run(self):
        import sys, tty, termios, select
        old_term = termios.tcgetattr(sys.stdin)
        try:
            tty.setcbreak(sys.stdin.fileno())
            self._main_loop()
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_term)
            self._shutdown_recording()

    def _main_loop(self):
        import sys, tty, termios, select

        self._move_to_home()

        rospy.loginfo("[teleop] Warming up TF for RViz...")
        warmup_rate = rospy.Rate(50)
        for _ in range(100):
            self._send_joints(self.home_joints, move_robot=False)
            warmup_rate.sleep()

        home_ee_pos, home_ee_quat = self._get_ee_pose()
        if home_ee_pos is None:
            rospy.logfatal("[teleop] Cannot read end-effector TF. Is the robot connected?")
            return
        self.home_pos  = home_ee_pos
        self.home_quat = home_ee_quat
        rospy.loginfo("[teleop] Home EE: [%.4f, %.4f, %.4f]", *home_ee_pos)
        rospy.loginfo("[teleop] Ready. Hold grey button to teleoperate. White button = gripper.")
        rospy.loginfo("[teleop] Keys: h=home  r=record  f=finish+save  d=discard")

        rate = rospy.Rate(self.control_rate)
        while not rospy.is_shutdown():

            # Keyboard:  h=home  r=record  f=finish+save  d=discard
            if select.select([sys.stdin], [], [], 0)[0]:
                key = sys.stdin.read(1).lower()
                if key == 'h':
                    rospy.loginfo("[teleop] 'h' pressed — returning to home")
                    self._move_to_home()
                    with self.lock:
                        self.active        = False
                        self.anchor_haptic = None
                    self.current_joints  = list(self.home_joints)
                    self.smoothed_joints = list(self.home_joints)
                elif key in ('r', 'f', 'd'):
                    self._handle_record_key(key)

            # Snapshot shared state from callbacks
            with self.lock:
                pose      = self.haptic_pose
                enabled   = self.active
                anchor_h  = self.anchor_haptic
                anchor_r  = self.anchor_robot
                anchor_q  = self.anchor_quat

            goal_pos = None
            ee_pos, _ = self._get_ee_pose()

            if enabled and pose is not None and anchor_h is not None:
                goal_pos = self._compute_target_pos(pose, anchor_h, anchor_r, ee_pos)
                self.last_goal = goal_pos

                # Orientation: scale the delta from the anchor independently per axis
                q = pose.pose.orientation
                haptic_quat = [q.x, q.y, q.z, q.w]

                if anchor_q is None:
                    anchor_q = list(self.home_quat)
                delta_quat = tft.quaternion_multiply(haptic_quat,
                                                     tft.quaternion_inverse(anchor_q))
                delta_roll, delta_pitch, delta_yaw = tft.euler_from_quaternion(delta_quat)
                home_roll,  home_pitch,  home_yaw  = tft.euler_from_quaternion(self.home_quat)

                target_quat = self.euler_to_quat(
                    home_roll  + delta_roll  * self.roll_scale,
                    home_pitch + delta_pitch * self.pitch_scale,
                    home_yaw   + delta_yaw   * self.yaw_scale)

                try:
                    joint_solution = self.rik.solve_position(
                        positions=goal_pos,
                        orientations=target_quat,
                        tolerances=[0.0] * 6)
                    if len(joint_solution) == 7 and all(math.isfinite(a) for a in joint_solution):
                        # Joint-space EMA + per-cycle rate limit. Catches IK
                        # branch flips and any residual jitter that survives
                        # the haptic pose filter — those are what the operator
                        # feels as "snaps" on the robot side.
                        a_alpha  = self.joint_filter_alpha
                        max_step = self.max_joint_step
                        smoothed = []
                        for tgt, cur in zip(joint_solution, self.smoothed_joints):
                            nxt = cur + a_alpha * (tgt - cur)
                            d = nxt - cur
                            if d >  max_step: nxt = cur + max_step
                            if d < -max_step: nxt = cur - max_step
                            smoothed.append(nxt)
                        self.smoothed_joints = smoothed
                        self._send_joints(smoothed)
                except Exception as e:
                    rospy.logwarn_throttle(2.0, "[teleop] IK failed: %s", e)
            else:
                self._send_joints(self.current_joints)

            self._publish_ee_markers(goal_pos or self.last_goal, ee_pos, enabled)
            self._publish_workspace_box()
            self._publish_bowl()
            self._publish_table()
            rate.sleep()


def main():
    node = TeleopVizNode()
    node.run()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass