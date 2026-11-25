#!/usr/bin/env python3
"""
Forward kinematics node: joint space -> task space, at three levels.

    position      X      = f(q)                       DH chain
    velocity      X_dot  = J(q) q_dot
    acceleration  X_ddot = J(q) q_ddot + J_dot(q, q_dot) q_dot

Input : /joint_state from the simulator (MEASURED q and q_dot). JointState
        carries no accelerations, so q_ddot is estimated by differentiating
        q_dot (and q_dot from q, if the message has no velocities).
Output: /current_ee_pose      PoseStamped     tool-tip position (+ orientation)
        /current_ee_xyz       Point           tool-tip position only
        /current_ee_velocity  Vector3Stamped  X_dot
        /current_ee_accel     Vector3Stamped  X_ddot
        /current_jacobian     Float64MultiArray  J, 3x5 row-major

The kinematics themselves live in kinematics.py.
"""

import sys
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, MultiArrayDimension
from geometry_msgs.msg import PoseStamped, Point, Vector3Stamped

sys.path.insert(0, str(Path(__file__).resolve().parent))
from kinematics import (                                    # noqa: E402
    N_JOINTS, forward_kinematics, forward_acceleration_kinematics, rot_to_quat)


class ForwardKinematicsNode(Node):
    def __init__(self):
        super().__init__('forward_kinematics_node')

        # 'fd' = finite-difference J_dot (default), 'analytic' = closed form
        self.declare_parameter('jacobian_dot_method', 'fd')
        self.declare_parameter('fd_step', 1e-5)
        self.declare_parameter('log_period_sec', 0.5)
        self.jdot_method = self.get_parameter('jacobian_dot_method').value
        self.fd_step = float(self.get_parameter('fd_step').value)
        self.log_period = float(self.get_parameter('log_period_sec').value)

        self.create_subscription(JointState, '/joint_state', self.joint_state_cb, 10)

        self.pose_pub = self.create_publisher(PoseStamped, '/current_ee_pose', 10)
        self.xyz_pub = self.create_publisher(Point, '/current_ee_xyz', 10)
        self.vel_pub = self.create_publisher(Vector3Stamped, '/current_ee_velocity', 10)
        self.acc_pub = self.create_publisher(Vector3Stamped, '/current_ee_accel', 10)
        self.jac_pub = self.create_publisher(Float64MultiArray, '/current_jacobian', 10)

        # Last evaluated state (to skip repeats) and q_dot estimate
        self.last_q = None
        self.last_qdot = np.zeros(N_JOINTS)
        self.last_qddot = np.zeros(N_JOINTS)
        self.last_stamp = None
        # q_ddot estimate: reference sample + low-pass filter (see numeric_qddot)
        self.acc_ref_qdot = None
        self.acc_ref_stamp = None
        self.qddot_filt = np.zeros(N_JOINTS)
        self.acc_tau = 0.25      # s, low-pass time constant (command timing jitter is ~20%)

        self.get_logger().info(
            f"FK node ready (position-only task space, J_dot method "
            f"'{self.jdot_method}'). Waiting for /joint_state...")

    # ------------------------------------------------------------------
    # Input: measured joint state
    # ------------------------------------------------------------------
    def joint_state_cb(self, msg: JointState):
        if len(msg.position) < N_JOINTS:
            return
        q = np.array(msg.position[:N_JOINTS], dtype=float)
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        qdot = (np.array(msg.velocity[:N_JOINTS], dtype=float)
                if len(msg.velocity) >= N_JOINTS else self.numeric_qdot(q, stamp))
        qddot = self.numeric_qddot(qdot, stamp)

        self.last_stamp = stamp
        self.compute_and_report(q, qdot, qddot)

    def numeric_qdot(self, q, stamp):
        """q_dot ~= (q_k - q_(k-1)) / dt, when the message carries no velocities."""
        if self.last_q is None or self.last_stamp is None:
            return np.zeros(N_JOINTS)
        dt = stamp - self.last_stamp
        if dt <= 1e-6 or dt > 1.0:        # ignore stale / duplicate stamps
            return self.last_qdot
        return (q - self.last_q) / dt

    def numeric_qddot(self, qdot, stamp):
        """
        Joint accelerations from successive joint velocities, low-pass filtered.

        /joint_state arrives at 200 Hz but in kinematic mode the velocity in it
        only changes when a new command is applied (~50 Hz), so differencing
        every sample gives a spike followed by zeros. Differentiate only when
        the velocity actually changes, and smooth over acc_tau.
        """
        if self.acc_ref_stamp is None:
            self.acc_ref_qdot, self.acc_ref_stamp = qdot.copy(), stamp
            return np.zeros(N_JOINTS)
        dt = stamp - self.acc_ref_stamp
        if np.array_equal(qdot, self.acc_ref_qdot):
            if dt > 0.1:                          # velocity held constant: no acceleration
                self.qddot_filt = np.zeros(N_JOINTS)
                self.acc_ref_stamp = stamp
            return self.qddot_filt
        if 1e-6 < dt <= 1.0:
            raw = (qdot - self.acc_ref_qdot) / dt
            self.qddot_filt += dt / (self.acc_tau + dt) * (raw - self.qddot_filt)
        self.acc_ref_qdot, self.acc_ref_stamp = qdot.copy(), stamp
        return self.qddot_filt

    # ------------------------------------------------------------------
    # Forward kinematics at the three levels
    # ------------------------------------------------------------------
    def compute_and_report(self, q, qdot, qddot):
        # Skip if nothing changed since the last evaluation
        if (self.last_q is not None
                and np.allclose(self.last_q, q, atol=1e-4)
                and np.allclose(self.last_qdot, qdot, atol=1e-4)
                and np.allclose(self.last_qddot, qddot, atol=1e-4)):
            return
        self.last_q, self.last_qdot, self.last_qddot = q.copy(), qdot.copy(), qddot.copy()

        # Position:      G_T_EE = G_T_0 . 0_T_1 ... 4_T_5 . 5_T_EE
        T = forward_kinematics(q)
        x, y, z = T[:3, 3]
        # Velocity and acceleration:  X_dot = J q_dot,  X_ddot = J q_ddot + J_dot q_dot
        a, bias, J, _ = forward_acceleration_kinematics(
            q, qdot, qddot, method=self.jdot_method, step=self.fd_step)
        v = J @ qdot

        self.get_logger().info(
            f"[FK] X {x:+.4f} Y {y:+.4f} Z {z:+.4f} m | "
            f"|v| {np.linalg.norm(v):.4f} m/s | |a| {np.linalg.norm(a):.4f} m/s^2 | "
            f"|J_dot q_dot| {np.linalg.norm(bias):.4f} m/s^2",
            throttle_duration_sec=self.log_period)

        # Singularity: J loses rank when its smallest singular value -> 0
        sigma_min = float(np.linalg.svd(J, compute_uv=False)[-1])
        if sigma_min < 1e-3:
            self.get_logger().warning(
                f"Near-singular configuration (smallest singular value = {sigma_min:.2e})",
                throttle_duration_sec=self.log_period)

        # ---- publish ----
        now = self.get_clock().now().to_msg()

        pt = Point()
        pt.x, pt.y, pt.z = float(x), float(y), float(z)
        self.xyz_pub.publish(pt)

        quat = rot_to_quat(T[:3, :3])
        pose = PoseStamped()
        pose.header.stamp = now
        pose.header.frame_id = "frame_0"
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = pt.x, pt.y, pt.z
        (pose.pose.orientation.x, pose.pose.orientation.y,
         pose.pose.orientation.z, pose.pose.orientation.w) = (float(c) for c in quat)
        self.pose_pub.publish(pose)

        self.vel_pub.publish(self._vec_msg(v, now))
        self.acc_pub.publish(self._vec_msg(a, now))

        jac = Float64MultiArray()
        jac.layout.dim = [MultiArrayDimension(label="rows", size=3, stride=15),
                          MultiArrayDimension(label="cols", size=5, stride=5)]
        jac.data = J.flatten().tolist()
        self.jac_pub.publish(jac)

    @staticmethod
    def _vec_msg(vec, stamp):
        msg = Vector3Stamped()
        msg.header.stamp = stamp
        msg.header.frame_id = "frame_0"
        msg.vector.x, msg.vector.y, msg.vector.z = (float(c) for c in vec[:3])
        return msg


def main():
    rclpy.init()
    node = ForwardKinematicsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
