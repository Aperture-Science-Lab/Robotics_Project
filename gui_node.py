#!/usr/bin/env python3
"""
ROS 2 GUI Node connected with Forward Kinematics and Inverse Kinematics nodes
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
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

        # 1. Joint command publisher (FK Mode)
        self.joint_command_pub = self.create_publisher(Float64MultiArray, '/joint_commands', 10)

        # 2. Cartesian target publisher (IK Mode)
        self.cartesian_target_pub = self.create_publisher(PoseStamped, '/target_pose', 10)

        # 3. Read calculated End-Effector position from FK Node
        self.ee_pose_sub = self.create_subscription(
            PoseStamped, '/current_ee_pose', self.ee_pose_callback, 10
        )

        # Robot joint specifications from XML
        self.num_joints = 6
        self.joint_names = [
            "Base Yaw", "Shoulder Pitch", "Elbow Pitch", 
            "Wrist Pitch", "Wrist Roll", "Gripper"
        ]
        self.joint_limits = [
            (-2.2, 2.2),
            (-1.57, 0.60),
            (-1.57, 1.45),
            (-2.0, 2.0),
            (-3.14, 3.14),
            (-1.60, 0.032)
        ]

        self.current_joint_values = [0.0] * self.num_joints
        self.current_cartesian_values = {"X": 0.15, "Y": 0.0, "Z": 0.15}
        self.measured_ee_pose = {"X": 0.0, "Y": 0.0, "Z": 0.0}

        # Signal Builder state
        self.signal_running = False
        self.signal_thread = None

    def ee_pose_callback(self, msg: PoseStamped):
        self.measured_ee_pose["X"] = msg.pose.position.x
        self.measured_ee_pose["Y"] = msg.pose.position.y
        self.measured_ee_pose["Z"] = msg.pose.position.z

    def dispatch_fk_command(self, joint_angles):
        self.current_joint_values = joint_angles
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
        pose_msg.pose.orientation.w = 1.0  # Neutral orientation
        self.cartesian_target_pub.publish(pose_msg)


class GUIApp:
    def __init__(self, root, node: RobotGUINode):
        self.root = root
        self.node = node
        self.root.title("Low Cost Robot Arm - Kinematics GUI")
        self.root.geometry("540x780")

        # Live FK Feedback Bar
        feedback_frame = ttk.LabelFrame(root, text=" Live End-Effector Position (From FK Node) ", padding=8)
        feedback_frame.pack(fill="x", padx=15, pady=6)
        self.lbl_ee = ttk.Label(feedback_frame, text="X: 0.000 m | Y: 0.000 m | Z: 0.000 m", font=("Helvetica", 10, "bold"))
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
            mode_frame, text="Inverse Kinematics (Cartesian Space / Meters)",
            variable=self.mode_var, value="IK", command=self.on_mode_change
        ).pack(anchor="w", pady=2)

        # Tabbed Notebook
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
        self.build_sliders_tab()
        self.build_constants_tab()
        self.build_signals_tab()

        # Bottom Bar: Emergency Stop / Reset
        bottom_frame = ttk.Frame(root, padding=10)
        bottom_frame.pack(fill="x", side="bottom")
        ttk.Button(bottom_frame, text="Reset to Zero", command=self.reset_all).pack(fill="x")

        # Periodic UI update for live feedback
        self.update_live_display()

    def update_live_display(self):
        ee = self.node.measured_ee_pose
        self.lbl_ee.config(text=f"X: {ee['X']:+.3f} m | Y: {ee['Y']:+.3f} m | Z: {ee['Z']:+.3f} m")
        self.root.after(100, self.update_live_display)

    def on_mode_change(self):
        mode = self.mode_var.get()
        if mode == "FK":
            self.fk_slider_frame.pack(fill="both", expand=True)
            self.ik_slider_frame.pack_forget()
            self.fk_const_frame.pack(fill="both", expand=True)
            self.ik_const_frame.pack_forget()
        else:
            self.fk_slider_frame.pack_forget()
            self.ik_slider_frame.pack(fill="both", expand=True)
            self.fk_const_frame.pack_forget()
            self.ik_const_frame.pack(fill="both", expand=True)

    # ---------------- TAB 1: SLIDERS ----------------
    def build_sliders_tab(self):
        self.fk_slider_frame = ttk.Frame(self.tab_sliders)
        self.fk_slider_frame.pack(fill="both", expand=True)

        for i in range(self.node.num_joints):
            name = self.node.joint_names[i]
            q_min, q_max = self.node.joint_limits[i]
            row = ttk.Frame(self.fk_slider_frame)
            row.pack(fill="x", pady=4)
            ttk.Label(row, text=name, width=15).pack(side="left")
            s = ttk.Scale(row, from_=q_min, to=q_max, orient="horizontal",
                          command=lambda val: self.publish_from_sliders())
            s.set(0.0)
            s.pack(side="right", fill="x", expand=True)
            self.sliders[name] = s

        self.ik_slider_frame = ttk.Frame(self.tab_sliders)
        # Scaled workspace bounds for desktop arm reach (~0.25 m)
        cart_axes = [("X", (-0.25, 0.25, 0.15)), ("Y", (-0.25, 0.25, 0.0)), ("Z", (0.02, 0.30, 0.15))]
        for axis, (min_v, max_v, init_v) in cart_axes:
            row = ttk.Frame(self.ik_slider_frame)
            row.pack(fill="x", pady=6)
            ttk.Label(row, text=f"Target {axis} (m)", width=15).pack(side="left")
            s = ttk.Scale(row, from_=min_v, to=max_v, orient="horizontal",
                          command=lambda val: self.publish_from_sliders())
            s.set(init_v)
            s.pack(side="right", fill="x", expand=True)
            self.sliders[axis] = s

    def publish_from_sliders(self):
        if self.mode_var.get() == "FK":
            angles = [self.sliders[name].get() for name in self.node.joint_names]
            self.node.dispatch_fk_command(angles)
        else:
            x = self.sliders["X"].get()
            y = self.sliders["Y"].get()
            z = self.sliders["Z"].get()
            self.node.dispatch_ik_command(x, y, z)

    # ---------------- TAB 2: CONSTANTS ----------------
    def build_constants_tab(self):
        self.fk_const_frame = ttk.Frame(self.tab_constants)
        self.fk_const_frame.pack(fill="both", expand=True)
        for i in range(self.node.num_joints):
            name = self.node.joint_names[i]
            row = ttk.Frame(self.fk_const_frame)
            row.pack(fill="x", pady=4)
            ttk.Label(row, text=f"{name} (rad):", width=18).pack(side="left")
            ent = ttk.Entry(row)
            ent.insert(0, "0.0")
            ent.pack(side="right", fill="x", expand=True)
            self.constant_entries[name] = ent

        self.ik_const_frame = ttk.Frame(self.tab_constants)
        for axis, default in [("X", "0.15"), ("Y", "0.00"), ("Z", "0.15")]:
            row = ttk.Frame(self.ik_const_frame)
            row.pack(fill="x", pady=4)
            ttk.Label(row, text=f"Target {axis} (m):", width=18).pack(side="left")
            ent = ttk.Entry(row)
            ent.insert(0, default)
            ent.pack(side="right", fill="x", expand=True)
            self.constant_entries[axis] = ent

        ttk.Button(self.tab_constants, text="Apply Constant Setpoints", command=self.apply_constants).pack(pady=15)

    def apply_constants(self):
        try:
            if self.mode_var.get() == "FK":
                angles = [float(self.constant_entries[name].get()) for name in self.node.joint_names]
                for i, name in enumerate(self.node.joint_names):
                    self.sliders[name].set(angles[i])
                self.node.dispatch_fk_command(angles)
            else:
                x = float(self.constant_entries["X"].get())
                y = float(self.constant_entries["Y"].get())
                z = float(self.constant_entries["Z"].get())
                self.sliders["X"].set(x)
                self.sliders["Y"].set(y)
                self.sliders["Z"].set(z)
                self.node.dispatch_ik_command(x, y, z)
        except ValueError:
            print("Invalid number entered in Constants tab.")

    # ---------------- TAB 3: SIGNAL BUILDER ----------------
    def build_signals_tab(self):
        ttk.Label(self.tab_signals, text="Sinusoidal Waveform Generator", font=("Helvetica", 11, "bold")).pack(pady=5)
        
        amp_frame = ttk.Frame(self.tab_signals)
        amp_frame.pack(fill="x", pady=4)
        ttk.Label(amp_frame, text="Amplitude (rad / m):", width=20).pack(side="left")
        self.entry_amp = ttk.Entry(amp_frame)
        self.entry_amp.insert(0, "0.05")
        self.entry_amp.pack(side="right", fill="x", expand=True)

        freq_frame = ttk.Frame(self.tab_signals)
        freq_frame.pack(fill="x", pady=4)
        ttk.Label(freq_frame, text="Frequency (Hz):", width=20).pack(side="left")
        self.entry_freq = ttk.Entry(freq_frame)
        self.entry_freq.insert(0, "0.5")
        self.entry_freq.pack(side="right", fill="x", expand=True)

        self.signal_btn = ttk.Button(self.tab_signals, text="Start Signal Generator", command=self.toggle_signal)
        self.signal_btn.pack(pady=15)

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
        try:
            amp = float(self.entry_amp.get())
            freq = float(self.entry_freq.get())
        except ValueError:
            amp, freq = 0.05, 0.5

        start_time = time.time()
        while self.node.signal_running:
            t = time.time() - start_time
            val = amp * math.sin(2 * math.pi * freq * t)

            if self.mode_var.get() == "FK":
                angles = [0.0] * self.node.num_joints
                angles[1] = val  # Oscillates Shoulder Pitch
                self.node.dispatch_fk_command(angles)
            else:
                self.node.dispatch_ik_command(0.15, 0.0, 0.15 + val)

            time.sleep(0.02)

    def reset_all(self):
        self.node.signal_running = False
        self.signal_btn.config(text="Start Signal Generator")
        for s in self.sliders.values():
            s.set(0.0)
        self.publish_from_sliders()


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
