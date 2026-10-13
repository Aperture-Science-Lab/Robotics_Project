#!/usr/bin/env python3
"""
Forward Kinematics Node for low_cost_robot_arm
Takes:  Joint angles q1, q2, q3, q4, q5
Gives:  Reached Cartesian coordinates [X, Y, Z] (meters) of the end-effector
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import PoseStamped, Point
import numpy as np

# ==============================================================================
# DH / KINEMATICS IMPLEMENTATION (Derived from low_cost_robot_arm.xml)
# ==============================================================================

def rot_x(angle):
    c, s = np.cos(angle), np.sin(angle)
    return np.array([[1, 0, 0, 0], [0, c, -s, 0], [0, s, c, 0], [0, 0, 0, 1]])

def rot_y(angle):
    c, s = np.cos(angle), np.sin(angle)
    return np.array([[c, 0, s, 0], [0, 1, 0, 0], [-s, 0, c, 0], [0, 0, 0, 1]])

def rot_z(angle):
    c, s = np.cos(angle), np.sin(angle)
    return np.array([[c, -s, 0, 0], [s, c, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])

def trans(x, y, z):
    return np.array([[1, 0, 0, x], [0, 1, 0, y], [0, 0, 1, z], [0, 0, 0, 1]])


def forward_kinematics(q):
    """
    Computes the Forward Kinematics from base to the end-effector site (frame_ee).
    
    Parameters:
        q: list or numpy array of 5 joint angles [q1, q2, q3, q4, q5] in radians:
           q[0] -> base_rotation  (axis 0 0 -1)
           q[1] -> pitch          (axis 0 1 0)
           q[2] -> elbow          (axis 0 -1 0)
           q[3] -> wrist_pitch    (axis 0 1 0)
           q[4] -> wrist_roll     (axis 1 0 0)

    Returns:
        x, y, z : The 3D Cartesian coordinates reached by the end-effector (meters)
        T       : The full 4x4 homogeneous transformation matrix
    """
    # 1. Base to Shoulder (joint 1: axis 0 0 -1)
    T1 = trans(0.012, 0.0, 0.0409) @ rot_z(-q[0])

    # 2. Shoulder to Elbow (joint 2: axis 0 1 0)
    T2 = trans(0.0, -0.0209, 0.0154) @ rot_y(q[1])

    # 3. Elbow Extension (joint 3: axis 0 -1 0)
    T3 = trans(-0.0148, 0.0065, 0.1083) @ rot_y(-q[2])

    # 4. Wrist Extension (joint 4: axis 0 1 0)
    T4 = trans(-0.10048, 5e-05, 0.0026999) @ rot_y(q[3])

    # 5. Gripper Base (joint 5: axis 1 0 0)
    T5 = trans(-0.045, 0.013097, 0.0) @ rot_x(q[4])

    # 6. End-Effector Tool Tip (site frame_ee in XML)
    # xyaxes="0 0 1 0 1 0" -> x_axis=[0,0,1], y_axis=[0,1,0], z_axis=[-1,0,0]
    T_ee = np.array([
        [ 0.0,  0.0, -1.0, -0.065 ],
        [ 0.0,  1.0,  0.0,  0.0105],
        [ 1.0,  0.0,  0.0,  0.0005],
        [ 0.0,  0.0,  0.0,  1.0   ]
    ])

    # Cumulative chain
    T = T1 @ T2 @ T3 @ T4 @ T5 @ T_ee

    # Extract reached X, Y, Z (meters)
    x = float(T[0, 3])
    y = float(T[1, 3])
    z = float(T[2, 3])

    return x, y, z, T


def rot_to_quat(R):
    """Utility to convert 3x3 rotation matrix to quaternion [x, y, z, w]."""
    tr = np.trace(R)
    if tr > 0:
        S = np.sqrt(tr + 1.0) * 2
        w = 0.25 * S
        x = (R[2, 1] - R[1, 2]) / S
        y = (R[0, 2] - R[2, 0]) / S
        z = (R[1, 0] - R[0, 1]) / S
    elif (R[0, 0] > R[1, 1]) and (R[0, 0] > R[2, 2]):
        S = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        w = (R[2, 1] - R[1, 2]) / S
        x = 0.25 * S
        y = (R[0, 1] + R[1, 0]) / S
        z = (R[0, 2] + R[2, 0]) / S
    elif R[1, 1] > R[2, 2]:
        S = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        w = (R[0, 2] - R[2, 0]) / S
        x = (R[0, 1] + R[1, 0]) / S
        y = 0.25 * S
        z = (R[1, 2] + R[2, 1]) / S
    else:
        S = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        w = (R[1, 0] - R[0, 1]) / S
        x = (R[0, 2] + R[2, 0]) / S
        y = (R[1, 2] + R[2, 1]) / S
        z = 0.25 * S
    return [x, y, z, w]

# ==============================================================================
# ROS 2 FORWARD KINEMATICS NODE
# ==============================================================================

class ForwardKinematicsNode(Node):
    def __init__(self):
        super().__init__('forward_kinematics_node')

        # Subscriptions: reads joint positions from simulation or GUI commands
        self.create_subscription(JointState, '/joint_state', self.joint_state_cb, 10)
        self.create_subscription(Float64MultiArray, '/joint_commands', self.joint_cmd_cb, 10)

        # Publishers: publishes the reached (X, Y, Z) pose
        self.pose_pub = self.create_publisher(PoseStamped, '/current_ee_pose', 10)
        self.xyz_pub = self.create_publisher(Point, '/current_ee_xyz', 10)

        self.last_q = None
        self.get_logger().info("Forward Kinematics Node ready. Waiting for joint inputs...")

    def joint_state_cb(self, msg: JointState):
        if len(msg.position) >= 5:
            self.compute_and_report(np.array(msg.position[:5]))

    def joint_cmd_cb(self, msg: Float64MultiArray):
        if len(msg.data) >= 5:
            self.compute_and_report(np.array(msg.data[:5]))

    def compute_and_report(self, q):
        # Ignore computation if angles haven't changed
        if self.last_q is not None and np.allclose(self.last_q, q, atol=1e-4):
            return
        self.last_q = q.copy()

        # ----------------------------------------------------
        # 1. CALL FORWARD KINEMATICS -> GET (X, Y, Z)
        # ----------------------------------------------------
        x, y, z, T = forward_kinematics(q)

        # ----------------------------------------------------
        # 2. PRINT REACHED X, Y, Z TO TERMINAL
        # ----------------------------------------------------
        self.get_logger().info(
            f"[FK Reached Position] -> X: {x:+.4f} m | Y: {y:+.4f} m | Z: {z:+.4f} m"
        )

        # ----------------------------------------------------
        # 3. PUBLISH FOR GUI & OTHER NODES
        # ----------------------------------------------------
        # Publish pure (X, Y, Z) point
        pt_msg = Point()
        pt_msg.x = x
        pt_msg.y = y
        pt_msg.z = z
        self.xyz_pub.publish(pt_msg)

        # Publish full PoseStamped
        quat = rot_to_quat(T[:3, :3])
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = "frame_0"
        pose_msg.pose.position.x = x
        pose_msg.pose.position.y = y
        pose_msg.pose.position.z = z
        pose_msg.pose.orientation.x = float(quat[0])
        pose_msg.pose.orientation.y = float(quat[1])
        pose_msg.pose.orientation.z = float(quat[2])
        pose_msg.pose.orientation.w = float(quat[3])
        self.pose_pub.publish(pose_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ForwardKinematicsNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()