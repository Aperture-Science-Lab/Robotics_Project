#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import numpy as np
from scipy.optimize import least_squares

"""
In the setup.py, change the entry point to include this file's name without the brackets

    entry_points={
        'console_scripts': [
            'kinematics = panda_kinematics.<file_name>:main',
        ],
    },
"""
# ==========================================================
# KINEMATICS IMPLEMENTATION — EDIT ONLY THIS SECTION
# ==========================================================

# Standard DH parameters for links 1 through 7.
#
# Format: one row per link, columns are [alpha, a, d].
# theta is the joint variable and is not stored here.
DH_PARAMS = [
    [-np.pi / 2,  0.0,     0.333],   # link 1
    [ np.pi / 2,  0.0,     0.0  ],   # link 2
    [ np.pi / 2,  0.0825,  0.316],   # link 3
    [-np.pi / 2, -0.0825,  0.0  ],   # link 4
    [ np.pi / 2,  0.0,     0.384],   # link 5
    [ np.pi / 2,  0.088,   0.0  ],   # link 6
    [ 0.0,        0.0,     0.0  ],   # link 7
]


# Fixed transform from link7 to the 'hand' body.
# 4x4 numpy array.
#
# Derive this from the <body name="hand"> tag in panda.xml.
# See the guide, Section 6.
_c, _s = np.cos(-np.pi / 4), np.sin(-np.pi / 4)
T_L7_HAND = np.array([
    [_c, -_s, 0.0, 0.0],
    [_s,  _c, 0.0, 0.0],
    [0.0, 0.0, 1.0, 0.107],
    [0.0, 0.0, 0.0, 1.0],
])


# Joint limits for joints 1 through 7, in radians.
# numpy arrays of shape (7,).
#
# Read them from the <joint> tags in panda.xml.
# See the guide, Section 6.
Q_MIN = np.array([-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973])
Q_MAX = np.array([ 2.8973,  1.7628,  2.8973, -0.0698,  2.8973,  3.7525,  2.8973])

# The target pose the node will try to reach.
# 4x4 numpy array in the 'link0' frame.
#
# Pick a joint configuration inside the limits, run it through
# your forward_kinematics, and use the result. See the guide,
# Section 8.
TARGET_POSE = None
Q_TARGET = np.array([0.3, -0.4, 0.2, -1.8, 0.1, 1.6, 0.5])

def dh_transform(alpha, a, d, theta):
    ct, st = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)
    return np.array([
        [ct, -st * ca,  st * sa, a * ct],
        [st,  ct * ca, -ct * sa, a * st],
        [0.0,      sa,       ca,      d],
        [0.0,     0.0,      0.0,    1.0],
    ])

def forward_kinematics(q):
    """
    Forward kinematics for the Panda arm.
    q: numpy array of 7 joint angles (rad).
    Returns a 4x4 homogeneous transform of the 'hand' frame relative to 'link0'.
    """
    # Start with an identity matrix (base frame)
    T = np.eye(4)
    
    # Multiply the 7 joint transformations sequentially left-to-right
    for i in range(7):
        alpha, a, d = DH_PARAMS[i]
        theta = q[i]
        T_i = dh_transform(alpha, a, d, theta)
        T = T @ T_i
        
    # Apply the fixed transform to reach the hand frame
    T_hand = T @ T_L7_HAND
    
    return T_hand


TARGET_POSE = forward_kinematics(Q_TARGET)
 
# Tolerances used to decide whether an IK solution is accepted.
POS_TOL = 1e-4      # m
ROT_TOL = 1e-3      # rad
 
 
def _pose_errors(T_cur, T_target):
    """Return (position error [m], orientation error angle [rad])."""
    pos = np.linalg.norm(T_target[:3, 3] - T_cur[:3, 3])
    R_rel = T_target[:3, :3].T @ T_cur[:3, :3]
    cos_ang = np.clip((np.trace(R_rel) - 1.0) / 2.0, -1.0, 1.0)
    return pos, np.arccos(cos_ang)
 
 
def inverse_kinematics(target_pose, q0):
    """
    Inverse kinematics for the Panda arm: bounded nonlinear least squares.
 
    - Residual = position error (3) + rotation-matrix element error (9), so it
      is smooth and needs no angle wrapping.
    - Bounds are the joint limits, so every returned vector respects them
      (the 'trf' solver keeps iterates inside the box).
    - Local method: converges to the solution nearest the seed q0, which keeps
      the arm from swinging to a distant configuration.
    - At singularities (e.g. q5 = 0) the Jacobian loses rank; the trust-region
      step is damped by construction, so it stays stable instead of crashing.
    - Unreachable targets: if no attempt reaches the tolerances, return the
      (clipped) seed and False so the arm stays put.
    - If the first attempt fails (local minimum), a few perturbed seeds are tried.
    """
    target_pose = np.asarray(target_pose, dtype=float)
    # least_squares requires x0 strictly inside the bounds
    eps = 1e-6
    q_seed = np.clip(np.asarray(q0, dtype=float), Q_MIN + eps, Q_MAX - eps)
 
    def residual(q):
        T = forward_kinematics(q)
        pos_error = target_pose[:3, 3] - T[:3, 3]
        rot_error = (target_pose[:3, :3] - T[:3, :3]).flatten()
        return np.concatenate([pos_error, rot_error])
 
    rng = np.random.default_rng(0)
    starts = [q_seed] + [
        np.clip(q_seed + rng.normal(0.0, 0.5, 7), Q_MIN + eps, Q_MAX - eps)
        for _ in range(4)
    ]
 
    best_q, best_dist = None, np.inf
    for k, x0 in enumerate(starts):
        res = least_squares(residual, x0, bounds=(Q_MIN, Q_MAX),
                            xtol=1e-10, ftol=1e-10, gtol=1e-10,
                            max_nfev=150)
        pos_err, rot_err = _pose_errors(forward_kinematics(res.x), target_pose)
        if pos_err < POS_TOL and rot_err < ROT_TOL:
            dist = np.linalg.norm(res.x - q_seed)
            if dist < best_dist:
                best_q, best_dist = res.x, dist
            if k == 0:       # nearest-to-seed solution found, no need to search on
                break
 
    if best_q is None:
        return q_seed, False
    return np.clip(best_q, Q_MIN, Q_MAX), True


# =================================================================
# DO NOT MODIFY BELOW (OR MODIFY, IF YOU KNOW WHAT YOU ARE DOING)
# =================================================================

CMD_SIZE = 9
GRIPPER_CTRL = 255.0


class PandaKinematicsNode(Node):
    def __init__(self):
        super().__init__('panda_kinematics')
        self.q_current = np.zeros(7)

        self.create_subscription(
            JointState, 'joint_state', self.on_joint_state, 10)
        self.cmd_pub = self.create_publisher(
            Float64MultiArray, 'joint_commands', 10)

        self.create_timer(0.05, self.tick)

        if TARGET_POSE is not None:
            self.get_logger().info(
                f"Target pose:\n{np.round(TARGET_POSE, 4)}")
        else:
            self.get_logger().warn(
                "TARGET_POSE is None — set it before running.")

    def on_joint_state(self, msg):
        self.q_current = np.array(msg.position[:7])

    def tick(self):
        if TARGET_POSE is None:
            return

        result = inverse_kinematics(TARGET_POSE, self.q_current)
        q_sol, ok = result

        if not ok:
            self.get_logger().warn(
                "IK did not converge.",
                throttle_duration_sec=2.0)

        cmd = Float64MultiArray()
        cmd.data = q_sol.tolist() + [GRIPPER_CTRL, 0.0]
        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = PandaKinematicsNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
