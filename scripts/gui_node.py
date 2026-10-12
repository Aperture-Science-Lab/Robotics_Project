#!/usr/bin/env python3
"""
ROS 2 GUI Node for Robotics Project
Includes:
- Mode Switcher: Forward Kinematics (Joint Space) vs Inverse Kinematics (Cartesian Space)
- 3 Input Types: Sliders, Constants, and Signal Builder (Sine Wave)
- Clear placeholders for connecting FK and IK nodes
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
# from geometry_msgs.msg import PoseStamped # (Uncomment when you use Pose for IK)

import tkinter as tk
from tkinter import ttk
import threading
import time
import math

class RobotGUINode(Node):
    def __init__(self):
        super().__init__('robot_gui_node')
        self.get_logger().info("Robot Control GUI Node Initialized.")

        # =====================================================================
        # PLACEHOLDER 1: ROS 2 PUBLISHERS & SUBSCRIBERS
        # =====================================================================
        # 1. To publish directly to MuJoCo for testing joint angles:
        self.joint_command_pub = self.create_publisher(Float64MultiArray, '/joint_commands', 10)

        # 2. PLACEHOLDER: Connect to your Inverse Kinematics Node
        # When your IK node is ready, uncomment this to send Cartesian targets (X, Y, Z):
        # self.cartesian_target_pub = self.create_publisher(PoseStamped, '/target_pose', 10)

        # 3. PLACEHOLDER: Connect to your Forward Kinematics Node
        # When your FK node is ready, subscribe to read calculated End-Effector position:
        # self.ee_pose_sub = self.create_subscription(PoseStamped, '/current_ee_pose', self.ee_pose_callback, 10)

        # Robot state variables
        self.num_joints = 6
        self.joint_names = [f"Joint {i+1}" for i in range(self.num_joints)]
        self.current_joint_values = [0.0] * self.num_joints
        self.current_cartesian_values = {"X": 0.3, "Y": 0.0, "Z": 0.3, "Roll": 0.0, "Pitch": 0.0, "Yaw": 0.0}

        # Signal Builder state
        self.signal_running = False
        self.signal_thread = None

    # =========================================================================
    # PLACEHOLDER 2: DISPATCHING FORWARD KINEMATICS COMMANDS
    # =========================================================================
    def dispatch_fk_command(self, joint_angles):
        """
        Called whenever Sliders, Constants, or Signals update in FK (Joint) Mode.
        joint_angles: list of floats [q1, q2, ..., q6] in radians
        """
        self.current_joint_values = joint_angles
        self.get_logger().info(f"[FK MODE] Dispatched Joint Angles: {[round(x, 2) for x in joint_angles]}")

        # Currently publishing straight to /joint_commands so MuJoCo moves:
        msg = Float64MultiArray()
        msg.data = joint_angles
        self.joint_command_pub.publish(msg)

        # --- FUTURE PLACEHOLDER ---
        # If you have an FK node that computes end-effector position from joint angles:
        # Your FK node will read /joint_state or /joint_commands and compute the forward kinematics!

    # =========================================================================
    # PLACEHOLDER 3: DISPATCHING INVERSE KINEMATICS COMMANDS
    # =========================================================================
    def dispatch_ik_command(self, x, y, z, roll=0.0, pitch=0.0, yaw=0.0):
        """
        Called whenever Sliders, Constants, or Signals update in IK (Cartesian) Mode.
        Target coordinates of the end-effector in meters.
        """
        self.current_cartesian_values["X"] = x
        self.current_cartesian_values["Y"] = y
        self.current_cartesian_values["Z"] = z
        self.get_logger().info(f"[IK MODE] Target Cartesian Pose -> X:{x:.2f}, Y:{y:.2f}, Z:{z:.2f}")

        # --- FUTURE PLACEHOLDER: PUBLISH TO IK NODE ---
        # When your IK node is created, publish this target to /target_pose:
        # pose_msg = PoseStamped()
        # pose_msg.pose.position.x = x
        # pose_msg.pose.position.y = y
        # pose_msg.pose.position.z = z
        # self.cartesian_target_pub.publish(pose_msg)
        #
        # Then, your IK node will calculate q = IK(X,Y,Z) and send q to /joint_commands!


# =============================================================================
# TKINTER GUI FRONTEND
# =============================================================================
class GUIApp:
    def __init__(self, root, node):
        self.root = root
        self.node = node
        self.root.title("Robotics Project - Kinematics & Control GUI")
        self.root.geometry("520x720")

        # Top Mode Selector: FK vs IK
        self.mode_var = tk.StringVar(value="FK")
        mode_frame = ttk.LabelFrame(root, text=" 1. Select Kinematics Mode ", padding=10)
        mode_frame.pack(fill="x", padx=15, pady=8)

        ttk.Radiobutton(
            mode_frame, text="Forward Kinematics (Joint Space / Radians)",
            variable=self.mode_var, value="FK", command=self.on_mode_change
        ).pack(anchor="w", pady=2)

        ttk.Radiobutton(
            mode_frame, text="Inverse Kinematics (Cartesian Space / X, Y, Z Meters)",
            variable=self.mode_var, value="IK", command=self.on_mode_change
        ).pack(anchor="w", pady=2)

        # Tabbed Notebook for 3 Input Methods
        self.notebook = ttk.Notebook(root)
        self.notebook.pack(fill="both", expand=True, padx=15, pady=5)

        self.tab_sliders = ttk.Frame(self.notebook, padding=10)
        self.tab_constants = ttk.Frame(self.notebook, padding=10)
        self.tab_signals = ttk.Frame(self.notebook, padding=10)

        self.notebook.add(self.tab_sliders, text=" Sliders ")
        self.notebook.add(self.tab_constants, text=" Constants ")
        self.notebook.add(self.tab_signals, text=" Signal Builder ")

        # Build each tab
        self.sliders = {}
        self.constant_entries = {}
        self.build_sliders_tab()
        self.build_constants_tab()
        self.build_signals_tab()

        # Bottom Bar: Emergency Stop / Reset
        bottom_frame = ttk.Frame(root, padding=10)
        bottom_frame.pack(fill="x", side="bottom")
        ttk.Button(bottom_frame, text="Reset to Zero", command=self.reset_all).pack(fill="x")

    def on_mode_change(self):
        """Switches controls between Joint space (angles) and Cartesian space (X, Y, Z)"""
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
        # FK Sliders
        self.fk_slider_frame = ttk.Frame(self.tab_sliders)
        self.fk_slider_frame.pack(fill="both", expand=True)

        for i in range(self.node.num_joints):
            name = f"Joint {i+1}"
            row = ttk.Frame(self.fk_slider_frame)
            row.pack(fill="x", pady=4)
            ttk.Label(row, text=name, width=10).pack(side="left")
            s = ttk.Scale(row, from_=-3.14, to=3.14, orient="horizontal", command=lambda val: self.publish_from_sliders())
            s.set(0.0)
            s.pack(side="right", fill="x", expand=True)
            self.sliders[name] = s

        # IK Sliders (Cartesian Coordinates)
        self.ik_slider_frame = ttk.Frame(self.tab_sliders)
        for axis, (min_v, max_v, init_v) in [("X", (0.0, 0.8, 0.3)), ("Y", (-0.8, 0.8, 0.0)), ("Z", (0.0, 0.8, 0.3))]:
            row = ttk.Frame(self.ik_slider_frame)
            row.pack(fill="x", pady=6)
            ttk.Label(row, text=f"Target {axis} (m)", width=14).pack(side="left")
            s = ttk.Scale(row, from_=min_v, to=max_v, orient="horizontal", command=lambda val: self.publish_from_sliders())
            s.set(init_v)
            s.pack(side="right", fill="x", expand=True)
            self.sliders[axis] = s

    def publish_from_sliders(self):
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
        for i in range(self.node.num_joints):
            row = ttk.Frame(self.fk_const_frame)
            row.pack(fill="x", pady=4)
            ttk.Label(row, text=f"Joint {i+1} (rad):", width=14).pack(side="left")
            ent = ttk.Entry(row)
            ent.insert(0, "0.0")
            ent.pack(side="right", fill="x", expand=True)
            self.constant_entries[f"Joint {i+1}"] = ent

        # IK Constants
        self.ik_const_frame = ttk.Frame(self.tab_constants)
        for axis, default in [("X", "0.30"), ("Y", "0.00"), ("Z", "0.30")]:
            row = ttk.Frame(self.ik_const_frame)
            row.pack(fill="x", pady=4)
            ttk.Label(row, text=f"Target {axis} (m):", width=14).pack(side="left")
            ent = ttk.Entry(row)
            ent.insert(0, default)
            ent.pack(side="right", fill="x", expand=True)
            self.constant_entries[axis] = ent

        ttk.Button(self.tab_constants, text="Apply Constant Setpoints", command=self.apply_constants).pack(pady=15)

    def apply_constants(self):
        try:
            if self.mode_var.get() == "FK":
                angles = [float(self.constant_entries[f"Joint {i+1}"].get()) for i in range(self.node.num_joints)]
                # Sync sliders
                for i, a in enumerate(angles):
                    self.sliders[f"Joint {i+1}"].set(a)
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
        
        # Amplitude
        amp_frame = ttk.Frame(self.tab_signals)
        amp_frame.pack(fill="x", pady=4)
        ttk.Label(amp_frame, text="Amplitude (rad / m):", width=20).pack(side="left")
        self.entry_amp = ttk.Entry(amp_frame)
        self.entry_amp.insert(0, "0.5")
        self.entry_amp.pack(side="right", fill="x", expand=True)

        # Frequency
        freq_frame = ttk.Frame(self.tab_signals)
        freq_frame.pack(fill="x", pady=4)
        ttk.Label(freq_frame, text="Frequency (Hz):", width=20).pack(side="left")
        self.entry_freq = ttk.Entry(freq_frame)
        self.entry_freq.insert(0, "0.5")
        self.entry_freq.pack(side="right", fill="x", expand=True)

        # Toggle Button
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
            amp, freq = 0.5, 0.5

        start_time = time.time()
        while self.node.signal_running:
            t = time.time() - start_time
            val = amp * math.sin(2 * math.pi * freq * t)

            if self.mode_var.get() == "FK":
                # Oscillates Joint 2 as an example:
                angles = [0.0] * self.node.num_joints
                angles[1] = val  # Joint 2 oscillation
                self.node.dispatch_fk_command(angles)
            else:
                # Oscillates Z-height in Cartesian space:
                self.node.dispatch_ik_command(0.3, 0.0, 0.3 + val)

            time.sleep(0.02)  # 50 Hz loop

    def reset_all(self):
        self.node.signal_running = False
        self.signal_btn.config(text="Start Signal Generator")
        for s in self.sliders.values():
            s.set(0.0)
        self.publish_from_sliders()


def main():
    rclpy.init()
    node = RobotGUINode()

    # Run ROS 2 spin in background
    ros_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    ros_thread.start()

    # Run Tkinter GUI
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
