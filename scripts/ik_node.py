#!/usr/bin/env python3
"""
Inverse Kinematics Node for low_cost_robot_arm
Pure NumPy implementation (Damped Least Squares / Levenberg-Marquardt)
No SciPy required.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import PoseStamped
import numpy as np

# ==============================================================================
# FORWARD KINEMATICS (Pure NumPy)
# ==============================================================================

def rot_x(q):
    c, s = np.cos(q), np.sin(q)
    return np.array([[1, 0, 0, 0], [0, c, -s, 0], [0, s, c, 0], [0, 0, 0, 1]])

def rot_y(q):
    c, s = np.cos(q), np.sin(q)
    return np.array([[c, 0, s, 0], [0, 1, 0, 0], [-s, 0, c, 0], [0, 0, 0, 1]])

def rot_z(q):
    c, s = np.cos(q), np.sin(q)
    return np.array([[c, -s, 0, 0], [s, c, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])

def trans(x, y, z):
    return np.array([[1, 0, 0, x], [0, 1, 0, y], [0, 0, 1, z], [0, 0, 0, 1]])

def forward_kinematics(q):
    """Computes reached end-effector [X, Y, Z] in meters."""
    T1 = trans(0.012, 0.0, 0.0409) @ rot_z(-q[0])
    T2 = trans(0.0, -0.0209, 0.0154) @ rot_y(q[1])
    T3 = trans(-0.0148, 0.0065, 0.1083) @ rot_y(-q[2])
    T4 = trans(-0.10048, 5e-05, 0.0026999) @ rot_y(q[3])
    T5 = trans(-0.045, 0.013097, 0.0) @ rot_x(q[4])

    T_ee = np.array([
        [ 0.0,  0.0, -1.0, -0.065 ],
        [ 0.0,  1.0,  0.0,  0.0105],
        [ 1.0,  0.0,  0.0,  0.0005],
        [ 0.0,  0.0,  0.0,  1.0   ]
    ])
    T = T1 @ T2 @ T3 @ T4 @ T5 @ T_ee
    return T[:3, 3]

# Joint limits from low_cost_robot_arm.xml
Q_MIN = np.array([-2.2, -1.57, -1.57, -2.0, -3.14])
Q_MAX = np.array([ 2.2,  0.60,  1.45,  2.0,  3.14])

# ==============================================================================
# PURE NUMPY INVERSE KINEMATICS (Damped Least Squares)
# ==============================================================================

def numerical_jacobian(q, eps=1e-5):
    """Computes 3x5 position Jacobian using central differences."""
    J = np.zeros((3, 5))
    for i in range(5):
        q_p = q.copy()
        q_m = q.copy()
        q_p[i] += eps
        q_m[i] -= eps
        J[:, i] = (forward_kinematics(q_p) - forward_kinematics(q_m)) / (2.0 * eps)
    return J

def solve_ik_numpy(target_pos, q_seed, max_iter=60, tol=1e-3):
    """
    Solves IK via Damped Least Squares without external dependencies.
    """
    q = np.clip(q_seed.copy(), Q_MIN, Q_MAX)
    lambda_sq = 1e-4  # Damping factor to handle singularities

    for _ in range(max_iter):
        p_curr = forward_kinematics(q)
        error = target_pos - p_curr

        if np.linalg.norm(error) < tol:
            break

        J = numerical_jacobian(q)
        # delta_q = J^T * (J * J^T + lambda^2 * I)^(-1) * error
        JJT = J @ J.T
        damped = JJT + lambda_sq * np.eye(3)
        delta_q = J.T @ np.linalg.solve(damped, error)

        # Step clamp for numerical stability
        step_len = np.linalg.norm(delta_q)
        if step_len > 0.15:
            delta_q = delta_q * (0.15 / step_len)

        q = np.clip(q + delta_q, Q_MIN, Q_MAX)

    final_err = np.linalg.norm(target_pos - forward_kinematics(q))
    return q, final_err

# ==============================================================================
# ROS 2 NODE
# ==============================================================================

class InverseKinematicsNode(Node):
    def __init__(self):
        super().__init__('inverse_kinematics_node')

        self.q_current = np.zeros(5)
        self.gripper_pos = 0.0

        # Subscriptions
        self.create_subscription(JointState, '/joint_state', self.on_joint_state, 10)
        self.create_subscription(PoseStamped, '/target_pose', self.on_target_pose, 10)

        # Publisher for actuator commands
        self.cmd_pub = self.create_publisher(Float64MultiArray, '/joint_commands', 10)

        self.get_logger().info("=== [IK NODE] Initialized successfully with Pure NumPy Solver ===")

    def on_joint_state(self, msg: JointState):
        if len(msg.position) >= 5:
            self.q_current = np.array(msg.position[:5])
        if len(msg.position) >= 6:
            self.gripper_pos = float(msg.position[5])

    def on_target_pose(self, msg: PoseStamped):
        tx = msg.pose.position.x
        ty = msg.pose.position.y
        tz = msg.pose.position.z
        target_pos = np.array([tx, ty, tz])

        # Solve IK using current configuration as seed
        q_sol, err_meters = solve_ik_numpy(target_pos, self.q_current)
        err_mm = err_meters * 1000.0

        if err_meters < 0.010:  # Within 10 mm
            self.get_logger().info(
                f"[IK SUCCESS] Target ({tx:+.3f}, {ty:+.3f}, {tz:+.3f}) -> Err: {err_mm:.1f} mm | Joints: {[round(float(v), 2) for v in q_sol]}"
            )
        else:
            self.get_logger().warn(
                f"[IK REACH LIMIT] Target ({tx:+.3f}, {ty:+.3f}, {tz:+.3f}) closest err: {err_mm:.1f} mm"
            )

        # Publish solution to move the arm
        cmd = Float64MultiArray()
        cmd.data = [float(v) for v in q_sol] + [float(self.gripper_pos)]
        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = InverseKinematicsNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()