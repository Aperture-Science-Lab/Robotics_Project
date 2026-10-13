#!/usr/bin/env python3
"""
ROS 2 GUI Node connected with Forward Kinematics and Inverse Kinematics nodes
- FK Signal Builder: Independent amplitude slider and frequency box per joint
- IK Signal Builder: 3D spatial sine wave on X, Y, Z in meters
- Constants tab: Displays exact physical limits
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import PoseStamped

import tkinter as tk
from tkinter import ttk
import threading
import time
import math


class RobotGUINode(Node):
    def __init__(self):
        super().__init__('robot_gui_node')
        self.get_logger().info("Robot Control GUI Node Initialized.")

        self.joint_command_pub = self.create_publisher(Float64MultiArray, '/joint_commands', 10)
        self.cartesian_target_pub = self.create_publisher(PoseStamped, '/target_pose', 10)

        self.ee_pose_sub = self.create_subscription(
            PoseStamped, '/current_ee_pose', self.ee_pose_callback, 10
        )

        self.num_joints = 6
        self.joint_names = [f"Joint {i+1}" for i in range(self.num_joints)]
        self.joint_limits = [
            (-2.20, 2.20),   # Joint 1: Base Yaw
            (-1.57, 0.60),   # Joint 2: Shoulder Pitch
            (-1.57, 1.45),   # Joint 3: Elbow Pitch
            (-2.00, 2.00),   # Joint 4: Wrist Pitch
            (-3.14, 3.14),   # Joint 5: Wrist Roll
            (-1.60, 0.032)   # Joint 6: Gripper
        ]

        # Home workspace target coordinates (Negative X)
        self.current_cartesian_values = {"X": -0.18, "Y": 0.00, "Z": 0.18}
        self.measured_ee_pose = {"X": 0.0, "Y": 0.0, "Z": 0.0}

        self.signal_running = False
        self.signal_thread = None

    def ee_pose_callback(self, msg: PoseStamped):
        self.measured_ee_pose["X"] = msg.pose.position.x
        self.measured_ee_pose["Y"] = msg.pose.position.y
        self.measured_ee_pose["Z"] = msg.pose.position.z

    def dispatch_fk_command(self, joint_angles):
        msg = Float64MultiArray()
        msg.data = [float(x) for x in joint_angles]
        self.joint_command_pub.publish(msg)

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


class GUIApp:
    def __init__(self, root, node: RobotGUINode):
        self.root = root
        self.node = node
        self.root.title("Low Cost Robot Arm - Kinematics GUI")
        self.root.geometry("600x900")

        self.is_ready = False

        # Live FK Feedback Bar
        feedback_frame = ttk.LabelFrame(root, text=" Live End-Effector Position (From FK Node) ", padding=8)
        feedback_frame.pack(fill="x", padx=15, pady=6)
        self.lbl_ee = ttk.Label(feedback_frame, text="X: 0.000 m | Y: 0.000 m | Z: 0.000 m", font=("Helvetica", 11, "bold"))
        self.lbl_ee.pack()

        # Mode Selector
        self.mode_var = tk.StringVar(value="FK")
        mode_frame = ttk.LabelFrame(root, text=" 1. Kinematics Mode ", padding=10)
        mode_frame.pack(fill="x", padx=15, pady=6)

        ttk.Radiobutton(
            mode_frame, text="Forward Kinematics (Joint Space / Radians)",
            variable=self.mode_var, value="FK", command=self.on_mode_change
        ).pack(anchor="w", pady=2)

        ttk.Radiobutton(
            mode_frame, text="Inverse Kinematics (Cartesian Space / Negative X)",
            variable=self.mode_var, value="IK", command=self.on_mode_change
        ).pack(anchor="w", pady=2)

        # Notebook Tabs
        self.notebook = ttk.Notebook(root)
        self.notebook.pack(fill="both", expand=True, padx=15, pady=5)

        self.tab_sliders = ttk.Frame(self.notebook, padding=10)
        self.tab_constants = ttk.Frame(self.notebook, padding=10)
        self.tab_signals = ttk.Frame(self.notebook, padding=10)

        self.notebook.add(self.tab_sliders, text=" Sliders ")
        self.notebook.add(self.tab_constants, text=" Constants ")
        self.notebook.add(self.tab_signals, text=" Signal Builder ")

        self.sliders = {}
        self.constant_entries = {}

        # Signal builder storage for individual FK joints
        self.fk_amp_scales = []
        self.fk_amp_labels = []
        self.fk_freq_entries = []

        self.build_sliders_tab()
        self.build_constants_tab()
        self.build_signals_tab()

        bottom_frame = ttk.Frame(root, padding=10)
        bottom_frame.pack(fill="x", side="bottom")
        ttk.Button(bottom_frame, text="Reset to Zero", command=self.reset_all).pack(fill="x")

        self.is_ready = True
        self.update_live_display()

    def update_live_display(self):
        ee = self.node.measured_ee_pose
        self.lbl_ee.config(text=f"X: {ee['X']:+.4f} m | Y: {ee['Y']:+.4f} m | Z: {ee['Z']:+.4f} m")
        self.root.after(100, self.update_live_display)

    def on_mode_change(self):
        mode = self.mode_var.get()
        if mode == "FK":
            self.fk_slider_frame.pack(fill="both", expand=True)
            self.ik_slider_frame.pack_forget()

            self.fk_const_frame.pack(fill="both", expand=True)
            self.ik_const_frame.pack_forget()

            # Switch Signal Builder to Per-Joint FK view
            self.fk_signal_frame.pack(fill="both", expand=True)
            self.ik_signal_frame.pack_forget()
        else:
            self.fk_slider_frame.pack_forget()
            self.ik_slider_frame.pack(fill="both", expand=True)

            self.fk_const_frame.pack_forget()
            self.ik_const_frame.pack(fill="both", expand=True)

            # Switch Signal Builder to 3D Cartesian IK view
            self.fk_signal_frame.pack_forget()
            self.ik_signal_frame.pack(fill="both", expand=True)

            self.publish_from_sliders()

    # ---------------- TAB 1: SLIDERS ----------------
    def build_sliders_tab(self):
        self.fk_slider_frame = ttk.Frame(self.tab_sliders)
        self.fk_slider_frame.pack(fill="both", expand=True)

        for i in range(self.node.num_joints):
            name = f"Joint {i+1}"
            q_min, q_max = self.node.joint_limits[i]
            row = ttk.Frame(self.fk_slider_frame)
            row.pack(fill="x", pady=4)
            ttk.Label(row, text=name, width=12).pack(side="left")
            s = ttk.Scale(row, from_=q_min, to=q_max, orient="horizontal", command=lambda val: self.publish_from_sliders())
            s.set(0.0)
            s.pack(side="right", fill="x", expand=True)
            self.sliders[name] = s

        self.ik_slider_frame = ttk.Frame(self.tab_sliders)
        cart_axes = [
            ("X", (-0.25, -0.10, -0.18)),
            ("Y", (-0.15, 0.15, 0.00)),
            ("Z", (0.05, 0.28, 0.18))
        ]
        for axis, (min_v, max_v, init_v) in cart_axes:
            row = ttk.Frame(self.ik_slider_frame)
            row.pack(fill="x", pady=6)
            ttk.Label(row, text=f"Target {axis} (m)", width=14).pack(side="left")
            s = ttk.Scale(row, from_=min_v, to=max_v, orient="horizontal", command=lambda val: self.publish_from_sliders())
            s.set(init_v)
            s.pack(side="right", fill="x", expand=True)
            self.sliders[axis] = s

    def publish_from_sliders(self):
        if not self.is_ready:
            return

        if self.mode_var.get() == "FK":
            angles = [self.sliders[f"Joint {i+1}"].get() for i in range(self.node.num_joints)]
            self.node.dispatch_fk_command(angles)
        else:
            x = self.sliders["X"].get()
            y = self.sliders["Y"].get()
            z = self.sliders["Z"].get()
            self.node.dispatch_ik_command(x, y, z)

    # ---------------- TAB 2: CONSTANTS ----------------
    def build_constants_tab(self):
        # FK Constants
        self.fk_const_frame = ttk.Frame(self.tab_constants)
        self.fk_const_frame.pack(fill="both", expand=True)

        ttk.Label(
            self.fk_const_frame,
            text="Set Joint Angles (Values outside limits are clamped):",
            font=("Helvetica", 9, "bold")
        ).pack(anchor="w", pady=(0, 6))

        for i in range(self.node.num_joints):
            name = f"Joint {i+1}"
            q_min, q_max = self.node.joint_limits[i]
            row = ttk.Frame(self.fk_const_frame)
            row.pack(fill="x", pady=4)

            lbl_text = f"{name} [{q_min:+.2f} to {q_max:+.2f} rad]:"
            ttk.Label(row, text=lbl_text, width=28).pack(side="left")

            ent = ttk.Entry(row)
            ent.insert(0, "0.0")
            ent.pack(side="right", fill="x", expand=True)
            self.constant_entries[name] = ent

        # IK Constants
        self.ik_const_frame = ttk.Frame(self.tab_constants)

        ttk.Label(
            self.ik_const_frame,
            text="Set Cartesian Coordinates (Reachable Workspace Limits):",
            font=("Helvetica", 9, "bold")
        ).pack(anchor="w", pady=(0, 6))

        ik_configs = [
            ("X", "-0.18", "-0.25 to -0.10 m"),
            ("Y", "0.00", "-0.15 to +0.15 m"),
            ("Z", "0.18", "+0.05 to +0.28 m")
        ]
        for axis, default, lim_str in ik_configs:
            row = ttk.Frame(self.ik_const_frame)
            row.pack(fill="x", pady=4)

            lbl_text = f"Target {axis} [{lim_str}]:"
            ttk.Label(row, text=lbl_text, width=28).pack(side="left")

            ent = ttk.Entry(row)
            ent.insert(0, default)
            ent.pack(side="right", fill="x", expand=True)
            self.constant_entries[axis] = ent

        ttk.Button(self.tab_constants, text="Apply Constant Setpoints", command=self.apply_constants).pack(pady=15)

    def apply_constants(self):
        try:
            if self.mode_var.get() == "FK":
                angles = []
                for i in range(self.node.num_joints):
                    val = float(self.constant_entries[f"Joint {i+1}"].get())
                    q_min, q_max = self.node.joint_limits[i]
                    val = max(q_min, min(q_max, val))
                    angles.append(val)
                    self.sliders[f"Joint {i+1}"].set(val)
                self.node.dispatch_fk_command(angles)
            else:
                x = float(self.constant_entries["X"].get())
                y = float(self.constant_entries["Y"].get())
                z = float(self.constant_entries["Z"].get())
                x = max(-0.25, min(-0.10, x))
                y = max(-0.15, min( 0.15, y))
                z = max( 0.05, min( 0.28, z))
                self.sliders["X"].set(x)
                self.sliders["Y"].set(y)
                self.sliders["Z"].set(z)
                self.node.dispatch_ik_command(x, y, z)
        except ValueError:
            print("Invalid input in constants.")

    # ---------------- TAB 3: SIGNAL BUILDER ----------------
    def build_signals_tab(self):
        # -------------------------------------------------------------
        # 1. FK FRAME: INDIVIDUAL PER-JOINT AMPLITUDE & FREQUENCY
        # -------------------------------------------------------------
        self.fk_signal_frame = ttk.Frame(self.tab_signals)
        self.fk_signal_frame.pack(fill="both", expand=True)

        ttk.Label(
            self.fk_signal_frame,
            text="Individual Joint Signal Generator (Radians & Hz):",
            font=("Helvetica", 10, "bold")
        ).pack(anchor="w", pady=(0, 6))

        for i in range(self.node.num_joints):
            row = ttk.Frame(self.fk_signal_frame)
            row.pack(fill="x", pady=4)

            # Joint Title
            ttk.Label(row, text=f"Joint {i+1}:", width=8, font=("Helvetica", 9, "bold")).pack(side="left")

            # Amplitude Slider
            ttk.Label(row, text="Amp:", width=4).pack(side="left")
            s = ttk.Scale(
                row, from_=0.0, to=0.5, orient="horizontal",
                command=lambda val, idx=i: self.on_fk_amp_slider_change(idx, val)
            )
            # Joint 2 has initial amplitude 0.2 rad, others 0.0 by default (user can raise any joint)
            init_amp = 0.20 if i == 1 else 0.0
            s.set(init_amp)
            s.pack(side="left", fill="x", expand=True, padx=4)
            self.fk_amp_scales.append(s)

            lbl_val = ttk.Label(row, text=f"{init_amp:.2f} rad", width=8)
            lbl_val.pack(side="left", padx=2)
            self.fk_amp_labels.append(lbl_val)

            # Frequency Entry
            ttk.Label(row, text="Freq (Hz):", width=9).pack(side="left", padx=(6, 2))
            ent_freq = ttk.Entry(row, width=6)
            ent_freq.insert(0, "0.5")
            ent_freq.pack(side="left")
            self.fk_freq_entries.append(ent_freq)

        # -------------------------------------------------------------
        # 2. IK FRAME: 3D CARTESIAN AMPLITUDE & FREQUENCY
        # -------------------------------------------------------------
        self.ik_signal_frame = ttk.Frame(self.tab_signals)

        ttk.Label(
            self.ik_signal_frame,
            text="3D Cartesian Signal Generator (Meters & Hz):",
            font=("Helvetica", 10, "bold")
        ).pack(anchor="w", pady=(0, 6))

        # IK Amplitude Row
        amp_row = ttk.Frame(self.ik_signal_frame)
        amp_row.pack(fill="x", pady=6)

        ttk.Label(amp_row, text="Amplitude (m):", width=16).pack(side="left")
        self.lbl_ik_amp_val = ttk.Label(amp_row, text="0.030 m", width=10, font=("Helvetica", 9, "bold"))
        self.lbl_ik_amp_val.pack(side="right")

        self.scale_ik_amp = ttk.Scale(
            amp_row, from_=0.0, to=0.05, orient="horizontal",
            command=self.on_ik_amp_slider_change
        )
        self.scale_ik_amp.set(0.03)
        self.scale_ik_amp.pack(side="right", fill="x", expand=True, padx=8)

        # IK Frequency Row
        freq_row = ttk.Frame(self.ik_signal_frame)
        freq_row.pack(fill="x", pady=6)

        ttk.Label(freq_row, text="Frequency (Hz):", width=16).pack(side="left")
        self.entry_ik_freq = ttk.Entry(freq_row)
        self.entry_ik_freq.insert(0, "0.5")
        self.entry_ik_freq.pack(side="right", fill="x", expand=True)

        ttk.Label(
            self.ik_signal_frame,
            text="Simultaneously modulates X, Y, and Z around (-0.18, 0.00, 0.18) m",
            font=("Helvetica", 9, "italic")
        ).pack(pady=4)

        # Common Start/Stop Button
        self.signal_btn = ttk.Button(self.tab_signals, text="Start Signal Generator", command=self.toggle_signal)
        self.signal_btn.pack(pady=15)

    def on_fk_amp_slider_change(self, idx, val):
        self.fk_amp_labels[idx].config(text=f"{float(val):.2f} rad")

    def on_ik_amp_slider_change(self, val):
        self.lbl_ik_amp_val.config(text=f"{float(val):.3f} m")

    def toggle_signal(self):
        if not self.node.signal_running:
            self.node.signal_running = True
            self.signal_btn.config(text="Stop Signal Generator")
            self.node.signal_thread = threading.Thread(target=self.run_signal_loop, daemon=True)
            self.node.signal_thread.start()
        else:
            self.node.signal_running = False
            self.signal_btn.config(text="Start Signal Generator")

    def run_signal_loop(self):
        start_time = time.time()
        while self.node.signal_running:
            t = time.time() - start_time

            if self.mode_var.get() == "FK":
                # =============================================================
                # FK MODE: EACH JOINT HAS ITS OWN AMPLITUDE & FREQUENCY
                # =============================================================
                angles = []
                for i in range(self.node.num_joints):
                    amp = float(self.fk_amp_scales[i].get())
                    try:
                        freq = float(self.fk_freq_entries[i].get())
                    except ValueError:
                        freq = 0.5

                    # Compute individual joint sinusoid in radians
                    val_j = amp * math.sin(2.0 * math.pi * freq * t)

                    # Clamp inside joint physical limits
                    q_min, q_max = self.node.joint_limits[i]
                    clamped_angle = max(q_min, min(q_max, val_j))
                    angles.append(clamped_angle)

                    if self.is_ready:
                        self.sliders[f"Joint {i+1}"].set(clamped_angle)

                self.node.dispatch_fk_command(angles)

            else:
                # =============================================================
                # IK MODE: 3D CARTESIAN SINE WAVE ON X, Y, Z (METERS)
                # =============================================================
                amp = float(self.scale_ik_amp.get())
                try:
                    freq = float(self.entry_ik_freq.get())
                except ValueError:
                    freq = 0.5

                omega = 2.0 * math.pi * freq * t

                target_x = -0.18 + amp * math.sin(omega)
                target_y =  0.00 + amp * math.cos(omega)
                target_z =  0.18 + amp * math.sin(omega + math.pi / 4.0)

                target_x = max(-0.25, min(-0.11, target_x))
                target_y = max(-0.14, min( 0.14, target_y))
                target_z = max( 0.06, min( 0.27, target_z))

                if self.is_ready:
                    self.sliders["X"].set(target_x)
                    self.sliders["Y"].set(target_y)
                    self.sliders["Z"].set(target_z)

                self.node.dispatch_ik_command(target_x, target_y, target_z)

            time.sleep(0.02)  # 50 Hz control rate

    def reset_all(self):
        self.node.signal_running = False
        self.signal_btn.config(text="Start Signal Generator")
        if self.mode_var.get() == "FK":
            for i in range(self.node.num_joints):
                self.sliders[f"Joint {i+1}"].set(0.0)
            self.node.dispatch_fk_command([0.0] * self.node.num_joints)
        else:
            self.sliders["X"].set(-0.18)
            self.sliders["Y"].set(0.00)
            self.sliders["Z"].set(0.18)
            self.node.dispatch_ik_command(-0.18, 0.0, 0.18)


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
        node.signal_running = False
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()