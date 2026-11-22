# Code 2

#!/usr/bin/env python3
"""
ROS 2 GUI Node connected with the Forward Kinematics and Inverse Kinematics nodes.

Six simulation modes, each driveable three ways (sliders / constant entries /
signal builder):

  FK Position      joint angles  q        -> /joint_commands
  FK Velocity      joint rates   qdot     -> /joint_velocity_commands
  FK Acceleration  joint accels  qddot    -> /joint_acceleration_commands
  IK Position      target [X, Y, Z]       -> /target_pose
  IK Velocity      target [Vx, Vy, Vz]    -> /target_velocity
  IK Acceleration  target [Ax, Ay, Az]    -> /target_accel

Live feedback from the FK node (/current_ee_pose, /current_ee_velocity,
/current_ee_accel) and from the IK node (/joint_velocity_commands,
/joint_acceleration_commands) is shown at the top.

TRANSLATION ONLY: the Cartesian task space is position / velocity /
acceleration of the tool tip. Orientation is not commanded.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import PoseStamped, Vector3Stamped

import tkinter as tk
from tkinter import ttk
import threading
import time
import math

STREAM_HZ = 50.0
STREAM_DT = 1.0 / STREAM_HZ

# Joint limits from low_cost_robot_arm.xml (6th entry is the gripper)
JOINT_LIMITS = [
    (-2.20, 2.20),   # Joint 1: Base Yaw
    (-1.57, 0.60),   # Joint 2: Shoulder Pitch
    (-1.57, 1.45),   # Joint 3: Elbow Pitch
    (-2.00, 2.00),   # Joint 4: Wrist Pitch
    (-3.14, 3.14),   # Joint 5: Wrist Roll
    (-1.60, 0.032),  # Joint 6: Gripper
]

# Actuator rate limits used when the GUI integrates its own motion
QDOT_LIMIT = 3.0      # rad/s
QDDOT_LIMIT = 20.0    # rad/s^2

# Cartesian home point the IK position signal oscillates about
HOME_XYZ = (-0.18, 0.00, 0.18)

# ==============================================================================
# PURE HELPERS (no tk, no ROS - unit-testable on their own)
# ==============================================================================

def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def signal_value(center, amp, freq, phase_deg, t):
    """One channel of the signal builder: center + amp*sin(2*pi*f*t + phase)."""
    return center + amp * math.sin(2.0 * math.pi * freq * t + math.radians(phase_deg))


def integrate_velocity(q, qdot, dt, limits):
    """q_next = clamp(q + qdot*dt). Used by the FK velocity mode."""
    return [clamp(q[i] + qdot[i] * dt, limits[i][0], limits[i][1]) for i in range(len(q))]


def integrate_acceleration(q, qdot, qddot, dt, limits,
                           qdot_limit=QDOT_LIMIT):
    """
    Double integration for the FK acceleration mode:
        qdot_next = clamp(qdot + qddot*dt, +/- qdot_limit)
        q_next    = clamp(q + qdot_next*dt, joint limits)

    A joint that runs into a position limit has its rate zeroed, so it stops
    there instead of winding up and lurching back when the sign flips.
    """
    q_next, qdot_next = [], []
    for i in range(len(q)):
        v = clamp(qdot[i] + qddot[i] * dt, -qdot_limit, qdot_limit)
        p = q[i] + v * dt
        lo, hi = limits[i]
        if p <= lo or p >= hi:
            p = clamp(p, lo, hi)
            v = 0.0
        q_next.append(p)
        qdot_next.append(v)
    return q_next, qdot_next

# ==============================================================================
# MODE DEFINITIONS
# ==============================================================================
# Each channel: (key, label, min, max, init, signal_center, amp_max, init_amp,
#                init_phase_deg)
MODE_SPECS = {
    "FK_POS": {
        "label": "Forward Kinematics - Position (joint angles, rad)",
        "unit": "rad",
        "stream": False,          # published on change, like the original GUI
        "channels": [
            (f"J{i+1}", f"Joint {i+1}", JOINT_LIMITS[i][0], JOINT_LIMITS[i][1],
             0.0, 0.0, 0.5, (0.20 if i == 1 else 0.0), 0.0)
            for i in range(6)
        ],
    },
    "FK_VEL": {
        "label": "Forward Velocity Kinematics (joint rates, rad/s)",
        "unit": "rad/s",
        "stream": True,           # rates are time-dependent: stream at 50 Hz
        "channels": [
            (f"dJ{i+1}", f"Joint {i+1}", -QDOT_LIMIT, QDOT_LIMIT,
             0.0, 0.0, 1.0, (0.30 if i == 1 else 0.0), 0.0)
            for i in range(5)
        ],
    },
    "FK_ACC": {
        "label": "Forward Acceleration Kinematics (joint accelerations, rad/s^2)",
        "unit": "rad/s^2",
        "stream": True,
        "channels": [
            (f"ddJ{i+1}", f"Joint {i+1}", -10.0, 10.0,
             0.0, 0.0, 5.0, (1.00 if i == 1 else 0.0), 0.0)
            for i in range(5)
        ],
    },
    "IK_POS": {
        "label": "Inverse Kinematics - Position (target X, Y, Z, m)",
        "unit": "m",
        "stream": False,
        "channels": [
            ("X", "Target X", -0.25, -0.10, HOME_XYZ[0], HOME_XYZ[0], 0.05, 0.03, 0.0),
            ("Y", "Target Y", -0.15,  0.15, HOME_XYZ[1], HOME_XYZ[1], 0.05, 0.03, 90.0),
            ("Z", "Target Z",  0.05,  0.28, HOME_XYZ[2], HOME_XYZ[2], 0.05, 0.03, 45.0),
        ],
    },
    "IK_VEL": {
        "label": "Inverse Velocity Kinematics (target Vx, Vy, Vz, m/s)",
        "unit": "m/s",
        "stream": True,
        "channels": [
            ("VX", "Target Vx", -0.10, 0.10, 0.0, 0.0, 0.05, 0.00, 0.0),
            ("VY", "Target Vy", -0.10, 0.10, 0.0, 0.0, 0.05, 0.03, 90.0),
            ("VZ", "Target Vz", -0.10, 0.10, 0.0, 0.0, 0.05, 0.00, 45.0),
        ],
    },
    "IK_ACC": {
        "label": "Inverse Acceleration Kinematics (target Ax, Ay, Az, m/s^2)",
        "unit": "m/s^2",
        "stream": True,
        "channels": [
            ("AX", "Target Ax", -1.0, 1.0, 0.0, 0.0, 0.5, 0.00, 0.0),
            ("AY", "Target Ay", -1.0, 1.0, 0.0, 0.0, 0.5, 0.20, 90.0),
            ("AZ", "Target Az", -1.0, 1.0, 0.0, 0.0, 0.5, 0.00, 45.0),
        ],
    },
}

MODE_ORDER = ["FK_POS", "FK_VEL", "FK_ACC", "IK_POS", "IK_VEL", "IK_ACC"]

# ==============================================================================
# ROS 2 NODE
# ==============================================================================

class RobotGUINode(Node):
    def __init__(self):
        super().__init__('robot_gui_node')
        self.get_logger().info("Robot Control GUI Node Initialized.")

        # --- Publishers: the six command channels ---
        self.joint_command_pub = self.create_publisher(Float64MultiArray, '/joint_commands', 10)
        self.joint_vel_pub = self.create_publisher(Float64MultiArray, '/joint_velocity_commands', 10)
        self.joint_acc_pub = self.create_publisher(Float64MultiArray, '/joint_acceleration_commands', 10)
        self.cartesian_target_pub = self.create_publisher(PoseStamped, '/target_pose', 10)
        self.cartesian_vel_pub = self.create_publisher(Vector3Stamped, '/target_velocity', 10)
        self.cartesian_acc_pub = self.create_publisher(Vector3Stamped, '/target_accel', 10)

        # --- Subscriptions: feedback from the FK node ---
        self.create_subscription(PoseStamped, '/current_ee_pose', self.ee_pose_callback, 10)
        self.create_subscription(Vector3Stamped, '/current_ee_velocity', self.ee_vel_callback, 10)
        self.create_subscription(Vector3Stamped, '/current_ee_accel', self.ee_acc_callback, 10)
        # --- and the joint commands the IK node resolves ---
        self.create_subscription(Float64MultiArray, '/joint_velocity_commands',
                                 self.resolved_vel_callback, 10)
        self.create_subscription(Float64MultiArray, '/joint_acceleration_commands',
                                 self.resolved_acc_callback, 10)

        self.num_joints = 6
        self.joint_limits = JOINT_LIMITS

        self.current_cartesian_values = {"X": HOME_XYZ[0], "Y": HOME_XYZ[1], "Z": HOME_XYZ[2]}
        self.measured_ee_pose = {"X": 0.0, "Y": 0.0, "Z": 0.0}
        self.measured_ee_vel = {"X": 0.0, "Y": 0.0, "Z": 0.0}
        self.measured_ee_acc = {"X": 0.0, "Y": 0.0, "Z": 0.0}
        self.resolved_qdot = [0.0] * 5
        self.resolved_qddot = [0.0] * 5

        self.signal_running = False
        self.signal_thread = None

    # ---------------- feedback ----------------
    def ee_pose_callback(self, msg: PoseStamped):
        self.measured_ee_pose["X"] = msg.pose.position.x
        self.measured_ee_pose["Y"] = msg.pose.position.y
        self.measured_ee_pose["Z"] = msg.pose.position.z

    def ee_vel_callback(self, msg: Vector3Stamped):
        self.measured_ee_vel["X"] = msg.vector.x
        self.measured_ee_vel["Y"] = msg.vector.y
        self.measured_ee_vel["Z"] = msg.vector.z

    def ee_acc_callback(self, msg: Vector3Stamped):
        self.measured_ee_acc["X"] = msg.vector.x
        self.measured_ee_acc["Y"] = msg.vector.y
        self.measured_ee_acc["Z"] = msg.vector.z

    def resolved_vel_callback(self, msg: Float64MultiArray):
        if len(msg.data) >= 5:
            self.resolved_qdot = [float(v) for v in msg.data[:5]]

    def resolved_acc_callback(self, msg: Float64MultiArray):
        if len(msg.data) >= 5:
            self.resolved_qddot = [float(v) for v in msg.data[:5]]

    # ---------------- dispatch ----------------
    def dispatch_fk_command(self, joint_angles):
        """Joint positions (5 joints + gripper) -> /joint_commands."""
        msg = Float64MultiArray()
        msg.data = [float(x) for x in joint_angles]
        self.joint_command_pub.publish(msg)

    def dispatch_joint_velocities(self, qdot):
        msg = Float64MultiArray()
        msg.data = [float(v) for v in qdot[:5]]
        self.joint_vel_pub.publish(msg)

    def dispatch_joint_accelerations(self, qddot):
        msg = Float64MultiArray()
        msg.data = [float(v) for v in qddot[:5]]
        self.joint_acc_pub.publish(msg)

    def dispatch_ik_command(self, x, y, z):
        self.current_cartesian_values["X"] = x
        self.current_cartesian_values["Y"] = y
        self.current_cartesian_values["Z"] = z

        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = "frame_0"
        pose_msg.pose.position.x = float(x)
        pose_msg.pose.position.y = float(y)
        pose_msg.pose.position.z = float(z)
        pose_msg.pose.orientation.w = 1.0
        self.cartesian_target_pub.publish(pose_msg)

    def dispatch_target_velocity(self, vx, vy, vz):
        self.cartesian_vel_pub.publish(self._vec3(vx, vy, vz))

    def dispatch_target_accel(self, ax, ay, az):
        self.cartesian_acc_pub.publish(self._vec3(ax, ay, az))

    def _vec3(self, x, y, z):
        msg = Vector3Stamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "frame_0"
        msg.vector.x = float(x)
        msg.vector.y = float(y)
        msg.vector.z = float(z)
        return msg


# ==============================================================================
# GUI
# ==============================================================================

class GUIApp:
    def __init__(self, root, node: RobotGUINode):
        self.root = root
        self.node = node
        self.root.title("Low Cost Robot Arm - Kinematics GUI")
        self.root.geometry("760x980")

        self.is_ready = False
        self._suppress = False          # blocks slider callbacks during programmatic set

        # Per-mode widget and value storage
        self.sliders = {m: {} for m in MODE_ORDER}
        self.constant_entries = {m: {} for m in MODE_ORDER}
        self.signal_widgets = {m: {} for m in MODE_ORDER}
        self.values = {m: [ch[4] for ch in MODE_SPECS[m]["channels"]] for m in MODE_ORDER}

        # Integrator state for the FK velocity / acceleration modes
        self.sim_q = [0.0] * 6
        self.sim_qdot = [0.0] * 5

        # ---------------- Live feedback ----------------
        feedback_frame = ttk.LabelFrame(root, text=" Live End-Effector State (from FK node) ", padding=8)
        feedback_frame.pack(fill="x", padx=15, pady=6)
        self.lbl_ee = ttk.Label(feedback_frame, text="", font=("Helvetica", 10, "bold"))
        self.lbl_ee.pack(anchor="w")
        self.lbl_ee_vel = ttk.Label(feedback_frame, text="", font=("Helvetica", 10))
        self.lbl_ee_vel.pack(anchor="w")
        self.lbl_ee_acc = ttk.Label(feedback_frame, text="", font=("Helvetica", 10))
        self.lbl_ee_acc.pack(anchor="w")

        resolved_frame = ttk.LabelFrame(root, text=" Resolved Joint Commands (from IK node) ", padding=8)
        resolved_frame.pack(fill="x", padx=15, pady=4)
        self.lbl_qdot = ttk.Label(resolved_frame, text="", font=("Helvetica", 9))
        self.lbl_qdot.pack(anchor="w")
        self.lbl_qddot = ttk.Label(resolved_frame, text="", font=("Helvetica", 9))
        self.lbl_qddot.pack(anchor="w")

        # ---------------- Mode selector ----------------
        self.mode_var = tk.StringVar(value="FK_POS")
        mode_frame = ttk.LabelFrame(root, text=" 1. Kinematics Mode ", padding=10)
        mode_frame.pack(fill="x", padx=15, pady=6)

        grid = ttk.Frame(mode_frame)
        grid.pack(fill="x")
        for idx, mode in enumerate(MODE_ORDER):
            ttk.Radiobutton(
                grid, text=MODE_SPECS[mode]["label"], variable=self.mode_var,
                value=mode, command=self.on_mode_change
            ).grid(row=idx % 3, column=idx // 3, sticky="w", padx=4, pady=1)

        # Integration option for the FK rate modes
        opt_row = ttk.Frame(mode_frame)
        opt_row.pack(fill="x", pady=(6, 0))
        self.integrate_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(
            opt_row,
            text="FK rate modes: integrate commands into joint motion (so the arm actually moves)",
            variable=self.integrate_var
        ).pack(anchor="w")

        # ---------------- Tabs ----------------
        self.notebook = ttk.Notebook(root)
        self.notebook.pack(fill="both", expand=True, padx=15, pady=5)

        self.tab_sliders = ttk.Frame(self.notebook, padding=10)
        self.tab_constants = ttk.Frame(self.notebook, padding=10)
        self.tab_signals = ttk.Frame(self.notebook, padding=10)

        self.notebook.add(self.tab_sliders, text=" Sliders ")
        self.notebook.add(self.tab_constants, text=" Constants ")
        self.notebook.add(self.tab_signals, text=" Signal Builder ")

        self.slider_frames = {}
        self.const_frames = {}
        self.signal_frames = {}

        for mode in MODE_ORDER:
            self.build_slider_frame(mode)
            self.build_const_frame(mode)
            self.build_signal_frame(mode)

        ttk.Button(self.tab_constants, text="Apply Constant Setpoints",
                   command=self.apply_constants).pack(pady=12)

        self.signal_btn = ttk.Button(self.tab_signals, text="Start Signal Generator",
                                     command=self.toggle_signal)
        self.signal_btn.pack(pady=12)

        bottom_frame = ttk.Frame(root, padding=10)
        bottom_frame.pack(fill="x", side="bottom")
        ttk.Button(bottom_frame, text="Reset to Zero", command=self.reset_all).pack(fill="x")

        self.is_ready = True
        self.on_mode_change()
        self.update_live_display()

        # One streaming thread drives the time-dependent modes and the signal
        # generator; event callbacks handle the static position modes.
        self.stream_running = True
        self.signal_t0 = time.time()
        self.stream_thread = threading.Thread(target=self.stream_loop, daemon=True)
        self.stream_thread.start()

    # ------------------------------------------------------------------
    # Frame builders (one per mode, per tab)
    # ------------------------------------------------------------------
    def build_slider_frame(self, mode):
        frame = ttk.Frame(self.tab_sliders)
        spec = MODE_SPECS[mode]
        ttk.Label(frame, text=spec["label"], font=("Helvetica", 9, "bold")).pack(anchor="w", pady=(0, 6))

        for ch in spec["channels"]:
            key, label, lo, hi, init = ch[0], ch[1], ch[2], ch[3], ch[4]
            row = ttk.Frame(frame)
            row.pack(fill="x", pady=4)
            ttk.Label(row, text=f"{label} ({spec['unit']})", width=18).pack(side="left")
            val_lbl = ttk.Label(row, text=f"{init:+.3f}", width=9)
            val_lbl.pack(side="right")
            s = ttk.Scale(row, from_=lo, to=hi, orient="horizontal",
                          command=lambda v, m=mode, k=key: self.on_slider_change(m, k, v))
            s.set(init)
            s.pack(side="right", fill="x", expand=True, padx=6)
            self.sliders[mode][key] = (s, val_lbl)

        self.slider_frames[mode] = frame

    def build_const_frame(self, mode):
        frame = ttk.Frame(self.tab_constants)
        spec = MODE_SPECS[mode]
        ttk.Label(frame, text=f"{spec['label']}  -  values outside limits are clamped:",
                  font=("Helvetica", 9, "bold")).pack(anchor="w", pady=(0, 6))

        for ch in spec["channels"]:
            key, label, lo, hi, init = ch[0], ch[1], ch[2], ch[3], ch[4]
            row = ttk.Frame(frame)
            row.pack(fill="x", pady=4)
            ttk.Label(row, text=f"{label} [{lo:+.2f} to {hi:+.2f} {spec['unit']}]:",
                      width=34).pack(side="left")
            ent = ttk.Entry(row)
            ent.insert(0, f"{init:.3f}")
            ent.pack(side="right", fill="x", expand=True)
            self.constant_entries[mode][key] = ent

        self.const_frames[mode] = frame

    def build_signal_frame(self, mode):
        frame = ttk.Frame(self.tab_signals)
        spec = MODE_SPECS[mode]
        ttk.Label(frame, text=f"{spec['label']}  -  per-channel sine generator:",
                  font=("Helvetica", 10, "bold")).pack(anchor="w", pady=(0, 6))

        for ch in spec["channels"]:
            key, label = ch[0], ch[1]
            amp_max, init_amp, init_phase = ch[6], ch[7], ch[8]
            row = ttk.Frame(frame)
            row.pack(fill="x", pady=3)

            ttk.Label(row, text=f"{label}:", width=11,
                      font=("Helvetica", 9, "bold")).pack(side="left")

            ttk.Label(row, text="Amp:", width=4).pack(side="left")
            amp_s = ttk.Scale(row, from_=0.0, to=amp_max, orient="horizontal",
                              command=lambda v, m=mode, k=key: self.on_amp_change(m, k, v))
            amp_s.set(init_amp)
            amp_s.pack(side="left", fill="x", expand=True, padx=4)

            amp_lbl = ttk.Label(row, text=f"{init_amp:.3f}", width=7)
            amp_lbl.pack(side="left", padx=2)

            ttk.Label(row, text="Freq (Hz):", width=9).pack(side="left", padx=(6, 2))
            freq_ent = ttk.Entry(row, width=6)
            freq_ent.insert(0, "0.5")
            freq_ent.pack(side="left")

            ttk.Label(row, text="Phase (deg):", width=11).pack(side="left", padx=(6, 2))
            ph_ent = ttk.Entry(row, width=6)
            ph_ent.insert(0, f"{init_phase:.0f}")
            ph_ent.pack(side="left")

            self.signal_widgets[mode][key] = (amp_s, amp_lbl, freq_ent, ph_ent)

        note = ("Oscillates about the home point (-0.18, 0.00, 0.18) m"
                if mode == "IK_POS" else "Oscillates about zero")
        ttk.Label(frame, text=note, font=("Helvetica", 9, "italic")).pack(anchor="w", pady=4)

        self.signal_frames[mode] = frame

    # ------------------------------------------------------------------
    # Mode switching
    # ------------------------------------------------------------------
    def on_mode_change(self):
        mode = self.mode_var.get()
        for m in MODE_ORDER:
            for frames in (self.slider_frames, self.const_frames, self.signal_frames):
                frames[m].pack_forget()
        self.slider_frames[mode].pack(fill="both", expand=True)
        self.const_frames[mode].pack(fill="both", expand=True)
        self.signal_frames[mode].pack(fill="both", expand=True)

        # Entering an FK rate mode: start integrating from where the arm is now
        if mode in ("FK_VEL", "FK_ACC"):
            self.sim_q = [self.get_slider_value("FK_POS", f"J{i+1}") for i in range(6)]
            self.sim_qdot = [0.0] * 5

        if not MODE_SPECS[mode]["stream"]:
            self.publish_static(mode)

    # ------------------------------------------------------------------
    # Widget callbacks
    # ------------------------------------------------------------------
    def get_slider_value(self, mode, key):
        return float(self.sliders[mode][key][0].get())

    def on_slider_change(self, mode, key, val):
        if self._suppress or not self.is_ready:
            return
        spec = MODE_SPECS[mode]
        idx = [c[0] for c in spec["channels"]].index(key)
        self.values[mode][idx] = float(val)
        self.sliders[mode][key][1].config(text=f"{float(val):+.3f}")
        if not spec["stream"]:
            self.publish_static(mode)

    def on_amp_change(self, mode, key, val):
        if key not in self.signal_widgets[mode]:
            return
        self.signal_widgets[mode][key][1].config(text=f"{float(val):.3f}")

    def apply_constants(self):
        mode = self.mode_var.get()
        spec = MODE_SPECS[mode]
        try:
            for idx, ch in enumerate(spec["channels"]):
                key, lo, hi = ch[0], ch[2], ch[3]
                val = clamp(float(self.constant_entries[mode][key].get()), lo, hi)
                self.values[mode][idx] = val
                self.set_slider(mode, key, val)
            if not spec["stream"]:
                self.publish_static(mode)
        except ValueError:
            print("Invalid input in constants.")

    def set_slider(self, mode, key, val):
        self.root.after(0, self._set_slider_main, mode, key, val)

    def _set_slider_main(self, mode, key, val):
        self._suppress = True
        try:
            s, lbl = self.sliders[mode][key]
            s.set(val)
            lbl.config(text=f"{val:+.3f}")
        finally:
            self._suppress = False

    # ------------------------------------------------------------------
    # Publishing
    # ------------------------------------------------------------------
    def publish_static(self, mode):
        """One-shot publish for the non-streaming (position) modes."""
        vals = self.values[mode]
        if mode == "FK_POS":
            self.sim_q = list(vals)
            self.node.dispatch_fk_command(vals)
        elif mode == "IK_POS":
            self.node.dispatch_ik_command(vals[0], vals[1], vals[2])

    def publish_streaming(self, mode, vals, dt):
        """
        Per-cycle publish for the time-dependent modes.

        FK rate modes optionally integrate the command into joint motion, so the
        arm really moves and the FK node sees a posture consistent with the
        rates it is given. Position is published first, so the FK node's
        joint_vel_cmd_cb / joint_acc_cmd_cb evaluate at the current posture.
        """
        if mode == "FK_VEL":
            if self.integrate_var.get():
                self.sim_q = integrate_velocity(self.sim_q, list(vals) + [0.0],
                                                dt, JOINT_LIMITS)
                self.sim_qdot = list(vals)
                self.node.dispatch_fk_command(self.sim_q)
            self.node.dispatch_joint_velocities(vals)

        elif mode == "FK_ACC":
            if self.integrate_var.get():
                q, qd = integrate_acceleration(self.sim_q[:5], self.sim_qdot,
                                               list(vals), dt, JOINT_LIMITS[:5])
                self.sim_q = q + [self.sim_q[5]]
                self.sim_qdot = qd
                self.node.dispatch_fk_command(self.sim_q)
                self.node.dispatch_joint_velocities(self.sim_qdot)
            self.node.dispatch_joint_accelerations(vals)

        elif mode == "IK_VEL":
            self.node.dispatch_target_velocity(vals[0], vals[1], vals[2])

        elif mode == "IK_ACC":
            self.node.dispatch_target_accel(vals[0], vals[1], vals[2])

    # ------------------------------------------------------------------
    # Signal generator / streaming loop
    # ------------------------------------------------------------------
    def toggle_signal(self):
        if not self.node.signal_running:
            self.signal_t0 = time.time()
            self.node.signal_running = True
            self.signal_btn.config(text="Stop Signal Generator")
        else:
            self.node.signal_running = False
            self.signal_btn.config(text="Start Signal Generator")

    def current_signal_values(self, mode, t):
        """Evaluate every channel of the active mode's signal generator."""
        vals = []
        for ch in MODE_SPECS[mode]["channels"]:
            key, lo, hi, center = ch[0], ch[2], ch[3], ch[5]
            amp_s, _, freq_ent, ph_ent = self.signal_widgets[mode][key]
            amp = float(amp_s.get())
            try:
                freq = float(freq_ent.get())
            except ValueError:
                freq = 0.5
            try:
                phase = float(ph_ent.get())
            except ValueError:
                phase = 0.0
            vals.append(clamp(signal_value(center, amp, freq, phase, t), lo, hi))
        return vals

    def stream_loop(self):
        last = time.time()
        while self.stream_running:
            now = time.time()
            dt = min(max(now - last, 1e-4), 0.2)
            last = now

            mode = self.mode_var.get()
            spec = MODE_SPECS[mode]

            try:
                if self.node.signal_running:
                    vals = self.current_signal_values(mode, now - self.signal_t0)
                    self.values[mode] = vals
                    for idx, ch in enumerate(spec["channels"]):
                        self.set_slider(mode, ch[0], vals[idx])
                else:
                    vals = list(self.values[mode])

                if spec["stream"]:
                    self.publish_streaming(mode, vals, dt)
                elif self.node.signal_running:
                    self.publish_static(mode)
            except Exception as exc:                      # keep the GUI alive
                print(f"[GUI] stream loop: {exc}")

            time.sleep(STREAM_DT)

    # ------------------------------------------------------------------
    def update_live_display(self):
        ee = self.node.measured_ee_pose
        ev = self.node.measured_ee_vel
        ea = self.node.measured_ee_acc
        self.lbl_ee.config(
            text=f"Position      X: {ee['X']:+.4f}  Y: {ee['Y']:+.4f}  Z: {ee['Z']:+.4f}  m")
        self.lbl_ee_vel.config(
            text=f"Velocity      X: {ev['X']:+.4f}  Y: {ev['Y']:+.4f}  Z: {ev['Z']:+.4f}  m/s"
                 f"   |v| = {math.sqrt(ev['X']**2 + ev['Y']**2 + ev['Z']**2):.4f}")
        self.lbl_ee_acc.config(
            text=f"Acceleration  X: {ea['X']:+.4f}  Y: {ea['Y']:+.4f}  Z: {ea['Z']:+.4f}  m/s^2")
        self.lbl_qdot.config(
            text="qdot  (rad/s)   : " + "  ".join(f"{v:+.3f}" for v in self.node.resolved_qdot))
        self.lbl_qddot.config(
            text="qddot (rad/s^2) : " + "  ".join(f"{v:+.3f}" for v in self.node.resolved_qddot))
        self.root.after(100, self.update_live_display)

    def reset_all(self):
        self.node.signal_running = False
        self.signal_btn.config(text="Start Signal Generator")

        for mode in MODE_ORDER:
            for idx, ch in enumerate(MODE_SPECS[mode]["channels"]):
                self.values[mode][idx] = ch[4]
                self.set_slider(mode, ch[0], ch[4])

        self.sim_q = [0.0] * 6
        self.sim_qdot = [0.0] * 5

        self.node.dispatch_fk_command([0.0] * 6)
        self.node.dispatch_joint_velocities([0.0] * 5)
        self.node.dispatch_joint_accelerations([0.0] * 5)
        self.node.dispatch_target_velocity(0.0, 0.0, 0.0)
        self.node.dispatch_target_accel(0.0, 0.0, 0.0)
        self.node.dispatch_ik_command(*HOME_XYZ)


def main():
    rclpy.init()
    node = RobotGUINode()

    ros_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    ros_thread.start()

    root = tk.Tk()
    app = GUIApp(root, node)
    try:
        root.mainloop()
    finally:
        app.stream_running = False
        node.signal_running = False
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()