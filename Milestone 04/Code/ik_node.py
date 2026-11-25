#!/usr/bin/env python3
"""
Inverse kinematics node: task space -> joint space, at three levels.

    position      /target_pose      [x, y, z]        ->  q        (Newton-Raphson)
    velocity      /target_velocity  [vx, vy, vz]     ->  q_dot  = J# X_dot
    acceleration  /target_accel     [ax, ay, az]     ->  q_ddot = J# (X_ddot - J_dot q_dot)

The arm is position-controlled (/joint_commands), so the velocity and
acceleration solutions are integrated into joint positions here:

    q_(k+1) = q_k + q_dot dt                                   velocity mode
    q_dot_(k+1) = q_dot_k + q_ddot dt,  q_(k+1) = q_k + q_dot_(k+1) dt   acceleration mode

Outputs: /joint_commands               position command (5 joints + gripper)
         /joint_velocity_commands      resolved q_dot   (for the GUI readout)
         /joint_acceleration_commands  resolved q_ddot  (for the GUI readout)

The kinematics themselves live in kinematics.py.
"""

import sys
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, MultiArrayDimension
from geometry_msgs.msg import PoseStamped, Vector3Stamped

sys.path.insert(0, str(Path(__file__).resolve().parent))
from kinematics import (                                    # noqa: E402
    N_JOINTS, Q_MIN, Q_MAX, QDOT_MAX, jacobian, inverse_kinematics,
    inverse_velocity_kinematics, inverse_acceleration_kinematics)


# ==============================================================================
# WORKSPACE-BOUNDARY HANDLING (shared by the velocity and acceleration modes)
# ==============================================================================

def _leaves_singularity(q, qdot, eps=1e-4):
    """True if a small step along q_dot increases the smallest singular value of J."""
    n = float(np.linalg.norm(qdot))
    if n < 1e-12:
        return False
    sigma = lambda qq: np.linalg.svd(jacobian(qq), compute_uv=False)[-1]
    return sigma(q + eps * qdot / n) > sigma(q)


def boundary_stop(q, qdot, v_des, achieved, dt, max_off_axis=0.2):
    """
    Stop at the workspace boundary instead of drifting off-axis.

    Two cases end the motion:
      * direction: near a singularity the damped J# bends the achieved
        velocity J q_dot away from v_des; if its off-axis part exceeds
        max_off_axis of |v_des| (or it points backwards), the command is dropped.
      * joint limit: if this step would push a joint past its limit, the whole
        q_dot is scaled down uniformly (keeping its Cartesian direction) so that
        joint lands exactly on the limit; the next step then stops. Clipping
        joints one by one would let the others carry on and move the tool in
        directions that were never commanded.

    Commands that move back away from the boundary are unaffected: the
    direction check only applies while heading deeper into the singularity
    (smallest singular value shrinking), otherwise the arm could never leave.
    Returns (q_dot, reason), reason None if nothing was limited.
    """
    speed = float(np.linalg.norm(v_des))
    if speed < 1e-9:
        return qdot, None

    u = v_des / speed
    along = float(achieved @ u)
    off = float(np.linalg.norm(achieved - along * u))
    if along <= 0.0 or (off > max_off_axis * speed and not _leaves_singularity(q, qdot)):
        return np.zeros_like(qdot), (
            f"commanded direction not reachable here "
            f"(off-axis {off / speed * 100:.0f}% of command)")

    if dt > 0.0:
        step = qdot * dt
        room = np.where(step > 0.0, Q_MAX - q, Q_MIN - q)
        with np.errstate(divide='ignore', invalid='ignore'):
            frac = np.where(np.abs(step) > 1e-12, room / step, np.inf)
        s = float(np.clip(frac.min(), 0.0, 1.0))
        if s < 1.0:
            return qdot * s, f"joint {int(np.argmin(frac)) + 1} at its limit"

    return qdot, None


# ==============================================================================
# ROS 2 NODE
# ==============================================================================

class InverseKinematicsNode(Node):
    def __init__(self):
        super().__init__('inverse_kinematics_node')

        # Integrate the resolved rates into position commands, so the arm
        # actually moves when only a velocity / acceleration is streamed in
        self.declare_parameter('integrate_velocity', True)
        self.declare_parameter('integrate_accel', True)
        self.declare_parameter('k_null', 1.0)
        self.declare_parameter('fd_step', 1e-5)
        self.integrate_velocity = self.get_parameter('integrate_velocity').value
        self.integrate_accel = self.get_parameter('integrate_accel').value
        self.k_null = float(self.get_parameter('k_null').value)
        self.fd_step = float(self.get_parameter('fd_step').value)

        self.q_current = np.zeros(N_JOINTS)
        self.qdot_current = np.zeros(N_JOINTS)
        self.gripper_pos = 0.0
        self.last_vel_time = None
        self.last_acc_time = None

        self.create_subscription(JointState, '/joint_state', self.on_joint_state, 10)
        self.create_subscription(PoseStamped, '/target_pose', self.on_target_pose, 10)
        self.create_subscription(Vector3Stamped, '/target_velocity', self.on_target_velocity, 10)
        self.create_subscription(Vector3Stamped, '/target_accel', self.on_target_accel, 10)

        self.cmd_pub = self.create_publisher(Float64MultiArray, '/joint_commands', 10)
        self.vel_pub = self.create_publisher(Float64MultiArray, '/joint_velocity_commands', 10)
        self.acc_pub = self.create_publisher(Float64MultiArray, '/joint_acceleration_commands', 10)

        self.get_logger().info("IK node ready: position / velocity / acceleration "
                               "(position-only task space).")

    def on_joint_state(self, msg: JointState):
        if len(msg.position) >= N_JOINTS:
            self.q_current = np.array(msg.position[:N_JOINTS])
        if len(msg.position) > N_JOINTS:
            self.gripper_pos = float(msg.position[N_JOINTS])
        if len(msg.velocity) >= N_JOINTS:
            self.qdot_current = np.array(msg.velocity[:N_JOINTS])

    def publish_position(self, q):
        cmd = Float64MultiArray()
        cmd.data = [float(v) for v in q] + [float(self.gripper_pos)]
        self.cmd_pub.publish(cmd)

    # ------------------------------------------------------------------
    # POSITION LEVEL:  q = f^-1(X_des)   by Newton-Raphson, seeded with the
    # current posture (so the solution closest to where the arm is wins)
    # ------------------------------------------------------------------
    def on_target_pose(self, msg: PoseStamped):
        p = msg.pose.position
        x_des = np.array([p.x, p.y, p.z])

        q_sol, err = inverse_kinematics(x_des, self.q_current)

        if err < 0.010:
            self.get_logger().info(
                f"[IK] target ({p.x:+.3f}, {p.y:+.3f}, {p.z:+.3f}) -> error "
                f"{err * 1000:.1f} mm | q {[round(float(v), 2) for v in q_sol]}")
        else:
            self.get_logger().warning(
                f"[IK] target ({p.x:+.3f}, {p.y:+.3f}, {p.z:+.3f}) out of reach: "
                f"closest error {err * 1000:.1f} mm")
        self.publish_position(q_sol)

    # ------------------------------------------------------------------
    # VELOCITY LEVEL:  q_dot = J# X_dot_des + null space,  q += q_dot dt
    # ------------------------------------------------------------------
    def on_target_velocity(self, msg: Vector3Stamped):
        v_des = np.array([msg.vector.x, msg.vector.y, msg.vector.z])

        res = inverse_velocity_kinematics(self.q_current, v_des, k_null=self.k_null)
        qdot = res['qdot']

        self.get_logger().info(
            f"[IVK] X_dot ({v_des[0]:+.3f}, {v_des[1]:+.3f}, {v_des[2]:+.3f}) m/s -> "
            f"q_dot {[round(float(v), 3) for v in qdot]} rad/s | "
            f"residual {res['residual'] * 1000:.2f} mm/s | sigma_min {res['sigma_min']:.3e}")
        if res['damping'] > 0.0:
            self.get_logger().warning(
                f"[IVK] Near singularity: damping {res['damping']:.4f}, "
                f"X_dot not tracked exactly")
        if res['scale'] < 1.0:
            self.get_logger().warning(
                f"[IVK] Joint speed limit hit: command scaled to {res['scale'] * 100:.0f}%")

        now = self.get_clock().now().nanoseconds * 1e-9
        dt = 0.0 if self.last_vel_time is None else now - self.last_vel_time
        self.last_vel_time = now

        qdot, reason = boundary_stop(self.q_current, qdot, v_des, res['achieved'], dt)
        if reason:
            self.get_logger().warning(f"[IVK] Stopped at boundary: {reason}",
                                      throttle_duration_sec=1.0)

        self.vel_pub.publish(self._array_msg(qdot))
        self.qdot_current = qdot

        # Integrate:  q_(k+1) = q_k + q_dot dt
        if self.integrate_velocity and 0.0 < dt < 0.5:
            self.q_current = np.clip(self.q_current + qdot * dt, Q_MIN, Q_MAX)
            self.publish_position(self.q_current)

    # ------------------------------------------------------------------
    # ACCELERATION LEVEL:  q_ddot = J# (X_ddot_des - J_dot q_dot) + null space,
    # then double integration
    # ------------------------------------------------------------------
    def on_target_accel(self, msg: Vector3Stamped):
        a_des = np.array([msg.vector.x, msg.vector.y, msg.vector.z])

        now = self.get_clock().now().nanoseconds * 1e-9
        dt = 0.0 if self.last_acc_time is None else now - self.last_acc_time
        self.last_acc_time = now

        # The acceleration integrator keeps its own (q, q_dot) for the whole
        # stream. Reading them back from /joint_state every cycle would reset
        # the velocity to zero in kinematic mode, so the arm could never speed
        # up. It is re-seeded from the measured posture, at rest, whenever a
        # new stream starts.
        if not 0.0 < dt < 0.5:
            self.acc_q = self.q_current.copy()
            self.acc_qdot = np.zeros(N_JOINTS)
            self.acc_vref = np.zeros(3)   # Cartesian velocity built up from a_des
            dt = 0.0

        res = inverse_acceleration_kinematics(self.acc_q, self.acc_qdot, a_des,
                                              step=self.fd_step)
        qddot = res['qddot']

        self.get_logger().info(
            f"[IAK] X_ddot ({a_des[0]:+.3f}, {a_des[1]:+.3f}, {a_des[2]:+.3f}) m/s^2 -> "
            f"q_ddot {[round(float(v), 3) for v in qddot]} rad/s^2 | "
            f"|J_dot q_dot| {np.linalg.norm(res['bias']):.4f} m/s^2 | "
            f"residual {res['residual']:.4f}", throttle_duration_sec=0.5)
        if res['scale'] < 1.0:
            self.get_logger().warning(
                f"[IAK] Joint acceleration limit hit: command scaled to "
                f"{res['scale'] * 100:.0f}%", throttle_duration_sec=0.5)

        self.acc_pub.publish(self._array_msg(qddot))

        if not (self.integrate_accel and dt > 0.0):
            return

        # q_dot_(k+1) = q_dot_k + q_ddot dt   (uniformly scaled to QDOT_MAX,
        # which keeps the direction; per-joint clipping would bend it)
        qdot_next = self.acc_qdot + qddot * dt
        over = float(np.max(np.abs(qdot_next) / QDOT_MAX))
        if over > 1.0:
            qdot_next = qdot_next / over
        self.acc_vref = self.acc_vref + a_des * dt

        # Stop at the boundary, judged against the velocity built up so far
        qdot_next, reason = boundary_stop(self.acc_q, qdot_next, self.acc_vref,
                                          jacobian(self.acc_q) @ qdot_next, dt)
        # q_(k+1) = q_k + q_dot_(k+1) dt
        q_next = np.clip(self.acc_q + qdot_next * dt, Q_MIN, Q_MAX)
        if reason:
            self.get_logger().warning(f"[IAK] Stopped at boundary: {reason}",
                                      throttle_duration_sec=1.0)
            qdot_next = np.zeros(N_JOINTS)          # come to rest at the boundary
            self.acc_vref = np.zeros(3)

        self.acc_q, self.acc_qdot = q_next, qdot_next
        self.qdot_current = qdot_next
        self.vel_pub.publish(self._array_msg(qdot_next))
        self.publish_position(q_next)

    @staticmethod
    def _array_msg(vec):
        msg = Float64MultiArray()
        msg.layout.dim = [MultiArrayDimension(label="joints", size=N_JOINTS, stride=N_JOINTS)]
        msg.data = [float(v) for v in vec]
        return msg


def main():
    rclpy.init()
    node = InverseKinematicsNode()
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
