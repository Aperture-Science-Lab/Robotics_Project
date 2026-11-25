#!/usr/bin/env python3
"""
Kinematics of the Low Cost Robot Arm  (5 revolute joints + a gripper).

    Joint space :  q = [q1, q2, q3, q4, q5]            (rad)
    Task space  :  X = [x, y, z] of the tool tip       (m, global frame G)

The task space is the POSITION of the tool tip only (the MuJoCo site
"frame_ee"); orientation is not part of the task. With 3 task DoF and
5 joint DoF the arm is OVER-ACTUATED (redundant):

    DoF_joints (5)  >  DoF_E.E. (3)   ->  J is 3x5, not square
                                       ->  J^-1 does not exist; use the
                                           pseudo-inverse  J# = J^T (J J^T)^-1
                                       ->  2 spare DoF (the null space of J)

Contents
    1. Homogeneous transformations           Rot_z, Rot_x, Trans
    2. DH parameters table                   (the arm's geometry, in one place)
    3. Forward position kinematics           G_T_EE = G_T_0 . 0_T_1 ... 4_T_5 . 5_T_EE
    4. Forward velocity kinematics           X_dot  = J(q) q_dot
    5. Forward acceleration kinematics       X_ddot = J q_ddot + J_dot q_dot
    6. Inverse position kinematics           Newton-Raphson
    7. Inverse velocity kinematics           q_dot  = J# X_dot  (+ null space)
    8. Inverse acceleration kinematics       q_ddot = J# (X_ddot - J_dot q_dot)
    9. Self-test                             python3 kinematics.py --selftest

Every other script (fk_node, ik_node, trajectory_node, mujoco_bridge --compare)
imports these functions, so the arm is modelled in exactly one place.
"""

import sys

import numpy as np

N_JOINTS = 5

# ==============================================================================
# 1. HOMOGENEOUS TRANSFORMATIONS
# ==============================================================================
# A 4x4 homogeneous transformation packs a rotation R (3x3) and a translation
# P (3x1):          | R  P |
#               T = |      |        p_old = T p_new   (points as [x y z 1])
#                   | 0  1 |

def rot_z(theta):
    """Pure rotation about the Z-axis by theta."""
    c, s = np.cos(theta), np.sin(theta)
    return np.array([[c, -s, 0.0, 0.0],
                     [s,  c, 0.0, 0.0],
                     [0.0, 0.0, 1.0, 0.0],
                     [0.0, 0.0, 0.0, 1.0]])


def rot_x(alpha):
    """Pure rotation about the X-axis by alpha."""
    c, s = np.cos(alpha), np.sin(alpha)
    return np.array([[1.0, 0.0, 0.0, 0.0],
                     [0.0, c, -s, 0.0],
                     [0.0, s,  c, 0.0],
                     [0.0, 0.0, 0.0, 1.0]])


def trans(x, y, z):
    """Pure translation by (x, y, z)."""
    T = np.eye(4)
    T[:3, 3] = (x, y, z)
    return T


# ==============================================================================
# 2. DH PARAMETERS TABLE
# ==============================================================================
# Axes assignment (standard DH convention):
#   Z_(i-1) : the rotation axis of joint i
#   X_i     : perpendicular to BOTH Z_(i-1) and Z_i, and intersecting both
#   E.E.    : frame 5, its Z parallel to the last joint axis Z_4, its origin
#             ON the tool tip, X_5 pointing from the roll axis to the tip
#
#   Z_0 : vertical (up), through the base-yaw axis           (joint 1)
#   Z_1 : horizontal, shoulder-pitch axis                     (joint 2)
#   Z_2 : parallel to Z_1, elbow axis                         (joint 3)
#   Z_3 : parallel to Z_1, wrist-pitch axis                   (joint 4)
#   Z_4 : along the forearm, wrist-roll axis                  (joint 5)
#
# Each frame is related to the one before it by the four DH steps:
#   theta_i : rotation    about     Z_(i-1)   (makes X_(i-1) parallel to X_i)
#   d_i     : translation along     Z_(i-1)   (brings the origins together)
#   a_i     : translation along     X_i       (makes the origins coincide)
#   alpha_i : rotation    about     X_i       (aligns Z_(i-1) with Z_i)
#
# Link geometry, taken from the body offsets in model/low_cost_robot_arm.xml:
#   base -> joint 1 axis      x = 0.012, z = 0.0409
#   joint 1 -> joint 2        z = 0.0154                (y = -0.0209)
#   upper arm  (j2 -> j3)     x = -0.0148, z = 0.1083   (y = +0.0065)
#   forearm    (j3 -> j4)     x = -0.10048, z = 0.0027  (y = +0.00005)
#   wrist      (j4 -> j5)     x = -0.045                (y = +0.013097)
#   tool tip   (j5 -> E.E.)   x = -0.065, y = 0.0105, z = 0.0005

D1 = 0.0409 + 0.0154                          # height of the shoulder axis    0.0563
A2 = np.hypot(0.0148, 0.1083)                 # upper-arm length               0.109307
A3 = np.hypot(0.10048, 0.0026999)             # forearm length                 0.100516
D4 = -0.0209 + 0.0065 + 0.00005 + 0.013097    # sideways offset of roll axis  -0.001253
D5 = -(0.045 + 0.065)                         # wrist-pitch axis -> tool, along Z_4 -0.110
A5 = np.hypot(0.0105, 0.0005)                 # tool tip offset from roll axis 0.010512

BETA2 = np.arctan2(0.0148, 0.1083)            # upper arm leans back from vertical
BETA3 = np.arctan2(0.0026999, 0.10048)        # forearm rises above horizontal
BETA5 = np.arctan2(0.0005, 0.0105)            # tool tip offset direction about Z_4

#  i |  theta_i                          |  d_i   |  a_i  |  alpha_i
# ---+-----------------------------------+--------+-------+---------
#  1 |  -q1                              |  D1    |  0    |  -pi/2
#  2 |   q2 - (pi/2 + BETA2)             |  0     |  A2   |   0
#  3 |  -q3 - (pi/2 - BETA2 - BETA3)     |  0     |  A3   |   0
#  4 |   q4 + (pi/2 - BETA3)             |  D4    |  0    |  -pi/2
#  5 |   q5 - (pi/2 - BETA5)             |  D5    |  A5   |   0
#
# theta_i = SIGN_i * q_i + OFFSET_i. A sign of -1 means the MuJoCo joint
# turns the opposite way to the DH Z-axis (joint 1's axis is "0 0 -1" in the
# model, joint 3's is "0 -1 0"); the offsets are the angles of the links at
# q = 0, where the arm stands up with the forearm pointing back over the base.
#                 SIGN   OFFSET                            d      a      alpha
DH_TABLE = [
    (-1.0,  0.0,                               D1,   0.0,  -np.pi / 2),
    (+1.0, -(np.pi / 2 + BETA2),               0.0,  A2,    0.0),
    (-1.0, -(np.pi / 2 - BETA2 - BETA3),       0.0,  A3,    0.0),
    (+1.0,  (np.pi / 2 - BETA3),               D4,   0.0,  -np.pi / 2),
    (+1.0, -(np.pi / 2 - BETA5),               D5,   A5,    0.0),
]

# Frame 0 sits on the base-yaw axis at table level, so it is the global frame
# G shifted by the 12 mm offset of that axis.
G_T_0 = trans(0.012, 0.0, 0.0)

# The tool tip IS the origin of frame 5, so the E.E. frame needs no extra
# translation. This constant rotation only turns frame 5 into the orientation
# of the MuJoCo site "frame_ee" (used when the pose is published).
T_5_EE = np.array([[np.sin(BETA5),  np.cos(BETA5),  0.0, 0.0],
                   [np.cos(BETA5), -np.sin(BETA5),  0.0, 0.0],
                   [0.0,            0.0,           -1.0, 0.0],
                   [0.0,            0.0,            0.0, 1.0]])

# Joint limits (rad) from low_cost_robot_arm.xml, and actuator rate limits
Q_MIN = np.array([-2.2, -1.57, -1.57, -2.0, -3.14])
Q_MAX = np.array([2.2, 0.60, 1.45, 2.0, 3.14])
Q_MID = 0.5 * (Q_MIN + Q_MAX)
Q_RANGE = Q_MAX - Q_MIN
QDOT_MAX = np.full(N_JOINTS, 3.0)              # rad/s
QDDOT_MAX = np.full(N_JOINTS, 20.0)            # rad/s^2


def dh_matrix(theta, d, a, alpha):
    """
    Transformation from frame (i-1) to frame i, the four DH steps in order:

        (i-1)_T_i = Rot_z(theta) . Trans_z(d) . Trans_x(a) . Rot_x(alpha)

                  | C_th  -S_th C_al   S_th S_al   a C_th |
                = | S_th   C_th C_al  -C_th S_al   a S_th |
                  |  0       S_al        C_al        d    |
                  |  0        0           0          1    |
    """
    ct, st = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)
    return np.array([[ct, -st * ca,  st * sa, a * ct],
                     [st,  ct * ca, -ct * sa, a * st],
                     [0.0,      sa,       ca,      d],
                     [0.0,     0.0,      0.0,    1.0]])


def dh_theta(q):
    """theta_i = SIGN_i * q_i + OFFSET_i for all five rows of the DH table."""
    q = np.asarray(q, dtype=float).reshape(-1)[:N_JOINTS]
    return np.array([sign * q[i] + offset
                     for i, (sign, offset, _, _, _) in enumerate(DH_TABLE)])


# ==============================================================================
# 3. FORWARD POSITION KINEMATICS     X = f(q)
# ==============================================================================

def dh_frames(q):
    """
    All DH frames expressed in the global frame:  [G_T_0, G_T_1, ..., G_T_5]

        G_T_i = G_T_0 . 0_T_1 . 1_T_2 ... (i-1)_T_i

    G_T_(i-1) carries what the Jacobian needs for joint i:
        Z_(i-1) = 3rd column of its rotation   (the joint axis)
        O_(i-1) = its translation              (a point on the joint axis)
    """
    theta = dh_theta(q)
    frames = [G_T_0]
    for i, (_, _, d, a, alpha) in enumerate(DH_TABLE):
        frames.append(frames[-1] @ dh_matrix(theta[i], d, a, alpha))
    return frames


def forward_kinematics(q):
    """
    Total homogeneous transformation of the end-effector:

        G_T_EE = G_T_0 . 0_T_1 . 1_T_2 . 2_T_3 . 3_T_4 . 4_T_5 . 5_T_EE

    Returns the 4x4 matrix; its last column is the tool-tip position [x y z].
    """
    return dh_frames(q)[-1] @ T_5_EE


def ee_position(q):
    """Tool-tip position X = [x, y, z] (m) in the global frame."""
    return dh_frames(q)[-1][:3, 3].copy()


# ==============================================================================
# 4. FORWARD VELOCITY KINEMATICS     X_dot = J(q) q_dot
# ==============================================================================
# General approach, column by column. For a REVOLUTE joint i:
#
#     J_v_i = Z_(i-1) x (O_N - O_(i-1))        (translational part, 3x1)
#     J_w_i = Z_(i-1)                          (rotational part, not used here)
#
# with O_N the end-effector origin. Only the translational rows are kept,
# since the task space is position only, so J is 3x5. The factor SIGN_i is
# d(theta_i)/d(q_i) from the DH table: a joint that turns opposite to its
# DH Z-axis moves the tool the opposite way.

def jacobian(q):
    """3x5 translational Jacobian J(q): q_dot [rad/s] -> X_dot [m/s]."""
    frames = dh_frames(q)
    O_N = frames[-1][:3, 3]                      # tool tip
    J = np.zeros((3, N_JOINTS))
    for i in range(N_JOINTS):
        Z_prev = frames[i][:3, 2]                # Z_(i-1), axis of joint i
        O_prev = frames[i][:3, 3]                # O_(i-1), on that axis
        J[:, i] = DH_TABLE[i][0] * np.cross(Z_prev, O_N - O_prev)
    return J


def numerical_jacobian(q, eps=1e-6):
    """
    Differentiation approach, done numerically: column i is dX/dq_i by a
    central difference of the forward kinematics. Kept as an independent
    check of jacobian() (see selftest).
    """
    q = np.asarray(q, dtype=float)
    J = np.zeros((3, N_JOINTS))
    for i in range(N_JOINTS):
        dq = np.zeros(N_JOINTS)
        dq[i] = eps
        J[:, i] = (ee_position(q + dq) - ee_position(q - dq)) / (2.0 * eps)
    return J


def forward_velocity_kinematics(q, qdot):
    """X_dot = J(q) q_dot.  Returns (X_dot, J)."""
    J = jacobian(q)
    return J @ np.asarray(qdot, dtype=float)[:N_JOINTS], J


# ==============================================================================
# 5. FORWARD ACCELERATION KINEMATICS     X_ddot = J q_ddot + J_dot q_dot
# ==============================================================================
# Differentiating X_dot = J q_dot treats it as a product of two functions of
# time:   X_ddot = J_dot q_dot + J q_ddot.
# The J_dot q_dot term is the acceleration the tool has even when every joint
# turns at constant speed (centripetal / Coriolis).

def jacobian_dot_fd(q, qdot, step=1e-5):
    """
    J_dot by a central finite difference along the motion:

        J_dot ~= ( J(q + h q_dot) - J(q - h q_dot) ) / (2h)

    J depends on time only through q, so by the chain rule
    dJ/dt = (dJ/dq) . q_dot, the derivative of J along q_dot.
    h = step / |q_dot| keeps the perturbation h*q_dot at a fixed length
    `step`, however fast the arm moves: a fixed h would perturb by almost
    nothing when the arm is nearly stopped (pure round-off) and by a large
    angle when it is fast (truncation error). At rest J_dot q_dot = 0 anyway.
    step = 1e-5 was measured as the best truncation/round-off trade-off.
    """
    qdot = np.asarray(qdot, dtype=float).reshape(-1)[:N_JOINTS]
    speed = float(np.linalg.norm(qdot))
    if speed < 1e-12:
        return np.zeros((3, N_JOINTS))
    h = step / speed
    q = np.asarray(q, dtype=float).reshape(-1)
    return (jacobian(q + h * qdot) - jacobian(q - h * qdot)) / (2.0 * h)


def jacobian_dot_analytic(q, qdot):
    """
    Closed-form J_dot, the reference the finite difference is checked against.
    With k_i = SIGN_i Z_(i-1) the joint axis and o_i = O_(i-1) a point on it:

        w_i     = sum_{j<i} k_j q_dot_j              angular velocity of axis i
        k_dot_i = w_i x k_i
        o_dot_i = sum_{j<i} q_dot_j k_j x (o_i - o_j)
        p_dot   = J q_dot                            tool-tip velocity
        J_dot_i = k_dot_i x (p - o_i) + k_i x (p_dot - o_dot_i)
    """
    qdot = np.asarray(qdot, dtype=float).reshape(-1)[:N_JOINTS]
    frames = dh_frames(q)
    p = frames[-1][:3, 3]
    k = [DH_TABLE[i][0] * frames[i][:3, 2] for i in range(N_JOINTS)]
    o = [frames[i][:3, 3] for i in range(N_JOINTS)]

    J = np.column_stack([np.cross(k[i], p - o[i]) for i in range(N_JOINTS)])
    p_dot = J @ qdot

    J_dot = np.zeros((3, N_JOINTS))
    w = np.zeros(3)
    for i in range(N_JOINTS):
        k_dot = np.cross(w, k[i])
        o_dot = sum((qdot[j] * np.cross(k[j], o[i] - o[j]) for j in range(i)),
                    np.zeros(3))
        J_dot[:, i] = np.cross(k_dot, p - o[i]) + np.cross(k[i], p_dot - o_dot)
        w = w + k[i] * qdot[i]
    return J_dot


def jacobian_dot(q, qdot, method='fd', step=1e-5):
    """J_dot by finite difference (default) or in closed form ('analytic')."""
    if method == 'analytic':
        return jacobian_dot_analytic(q, qdot)
    return jacobian_dot_fd(q, qdot, step=step)


def forward_acceleration_kinematics(q, qdot, qddot, method='fd', step=1e-5):
    """
    X_ddot = J q_ddot + J_dot q_dot

    Returns (X_ddot, bias, J, J_dot) with bias = J_dot q_dot.
    """
    qdot = np.asarray(qdot, dtype=float).reshape(-1)[:N_JOINTS]
    qddot = np.asarray(qddot, dtype=float).reshape(-1)[:N_JOINTS]
    J = jacobian(q)
    J_dot = jacobian_dot(q, qdot, method=method, step=step)
    bias = J_dot @ qdot
    return J @ qddot + bias, bias, J, J_dot


# ==============================================================================
# 6. INVERSE POSITION KINEMATICS     q = f^-1(X)   (Newton-Raphson)
# ==============================================================================
# The forward kinematics are non-linear in q, and with 5 joints for 3 task
# coordinates a closed-form trigonometric solution is not unique, so the
# equations are solved numerically. Writing them as roots of
#
#     F(q) = f(q) - X_des = 0
#
# the Newton-Raphson update is   q_(n+1) = q_n - F'(q_n)^-1 F(q_n),
# where the gradient F'(q) of the three functions w.r.t. the five joints is
# exactly the Jacobian J(q). J is 3x5 and has no inverse, so the damped
# pseudo-inverse is used:
#
#     q_(n+1) = q_n + J^T (J J^T + lambda^2 I)^-1 (X_des - f(q_n))
#
# The small lambda (Levenberg-Marquardt damping) keeps the step bounded near
# singularities; each step is also clamped in length and to the joint limits.
# Stopping criteria: the error converges below tol, or max_iter is reached.
# Of the infinitely many solutions, the one closest to the seed q_seed is
# found, so seeding with the current posture gives the smallest motion.

def inverse_kinematics(x_des, q_seed, max_iter=60, tol=1e-3,
                       lam=1e-2, max_step=0.15):
    """
    Newton-Raphson inverse position kinematics.

    Returns (q, final position error in m).
    """
    x_des = np.asarray(x_des, dtype=float)
    q = np.clip(np.asarray(q_seed, dtype=float).copy(), Q_MIN, Q_MAX)

    for _ in range(max_iter):
        error = x_des - ee_position(q)                    # -F(q_n)
        if np.linalg.norm(error) < tol:
            break
        J = jacobian(q)                                   # F'(q_n)
        delta_q = J.T @ np.linalg.solve(J @ J.T + lam ** 2 * np.eye(3), error)

        step_len = np.linalg.norm(delta_q)                # step clamp
        if step_len > max_step:
            delta_q *= max_step / step_len
        q = np.clip(q + delta_q, Q_MIN, Q_MAX)

    return q, float(np.linalg.norm(x_des - ee_position(q)))


# ==============================================================================
# 7. INVERSE VELOCITY KINEMATICS     q_dot = J# X_dot  (+ null space)
# ==============================================================================
# X_dot = J q_dot is LINEAR in q_dot, so it can be solved for q_dot. The
# solution fails where J has no inverse:
#   * J is not square (3x5, over-actuated)  ->  right pseudo-inverse
#         J# = J^T (J J^T)^-1        (the minimum-norm q_dot giving X_dot)
#   * det(J J^T) = 0                      ->  SINGULARITY: J# blows up.
#     Damping  J# = J^T (J J^T + lambda^2 I)^-1  keeps q_dot bounded there,
#     at the price of not tracking X_dot exactly.

def damped_pinv(J, lam_max=0.08, sigma_thresh=0.02):
    """
    Damped right pseudo-inverse  J# = J^T (J J^T + lambda^2 I)^-1.

    lambda is ADAPTIVE: zero while the arm is well conditioned (so J# is the
    exact pseudo-inverse), growing smoothly to lam_max as the smallest
    singular value sigma_min of J drops below sigma_thresh.

    Returns (J#, sigma_min, lambda).
    """
    sigma_min = float(np.linalg.svd(J, compute_uv=False)[-1])
    if sigma_min >= sigma_thresh:
        lam = 0.0
    else:
        lam = lam_max * np.sqrt(max(1.0 - (sigma_min / sigma_thresh) ** 2, 0.0))
    return J.T @ np.linalg.inv(J @ J.T + lam ** 2 * np.eye(3)), sigma_min, lam


def joint_limit_gradient(q):
    """
    Gradient of H(q) = 1/2 sum( ((q - q_mid) / q_range)^2 ).
    Moving along -grad(H) pushes the posture back toward mid-range: this is
    what the two redundant DoF are spent on.
    """
    return (np.asarray(q, dtype=float) - Q_MID) / Q_RANGE ** 2


def null_space_projector(J, J_pinv):
    """N = I - J# J : joint motions that do NOT move the tool tip (J N = 0)."""
    return np.eye(N_JOINTS) - J_pinv @ J


def inverse_velocity_kinematics(q, v_des, k_null=1.0, respect_limits=True):
    """
    Resolved-rate solution for a desired tool-tip velocity:

        q_dot = J# X_dot_des  +  (I - J# J) (-k_null grad H)
                ------------     ---------------------------
                task motion      self-motion in the null space
                                 (joint-limit avoidance, no tool motion)

    respect_limits: if any joint would exceed QDOT_MAX the WHOLE q_dot is
    scaled down uniformly, which keeps the Cartesian direction (clipping
    joints one by one would bend the path).

    Returns a dict: qdot, achieved X_dot, residual, sigma_min, damping, scale.
    """
    v_des = np.asarray(v_des, dtype=float).reshape(-1)[:3]
    J = jacobian(q)
    J_pinv, sigma_min, lam = damped_pinv(J)

    qdot = J_pinv @ v_des
    if k_null != 0.0:
        qdot = qdot + null_space_projector(J, J_pinv) @ (-k_null * joint_limit_gradient(q))

    scale = 1.0
    if respect_limits:
        over = float(np.max(np.abs(qdot) / QDOT_MAX))
        if over > 1.0:
            scale = 1.0 / over
            qdot = qdot * scale

    achieved = J @ qdot
    return {'qdot': qdot, 'achieved': achieved,
            'residual': float(np.linalg.norm(v_des - achieved)),
            'sigma_min': sigma_min, 'damping': lam, 'scale': scale}


# ==============================================================================
# 8. INVERSE ACCELERATION KINEMATICS     q_ddot = J# (X_ddot - J_dot q_dot)
# ==============================================================================

def inverse_acceleration_kinematics(q, qdot, a_des, kp_null=4.0, kd_null=2.0,
                                    respect_limits=True, step=1e-5):
    """
    From X_ddot = J q_ddot + J_dot q_dot, solved for q_ddot:

        q_ddot = J# (X_ddot_des - J_dot q_dot)  +  (I - J# J) q_ddot_0

    Subtracting J_dot q_dot removes the centripetal / Coriolis acceleration
    the tool already has from the motion in progress; leaving it out makes an
    acceleration-level command drift off its path.

    q_ddot_0 = -kp_null grad H - kd_null q_dot is a PD law in the null space:
    it pulls the posture toward mid-range and damps the internal motion, so
    the redundant joints do not wander while the task acceleration is exact.

    Returns a dict: qddot, achieved X_ddot, bias, residual, sigma_min,
    damping, scale.
    """
    a_des = np.asarray(a_des, dtype=float).reshape(-1)[:3]
    qdot = np.asarray(qdot, dtype=float).reshape(-1)[:N_JOINTS]

    J = jacobian(q)
    bias = jacobian_dot(q, qdot, step=step) @ qdot      # J_dot q_dot
    J_pinv, sigma_min, lam = damped_pinv(J)

    qddot = J_pinv @ (a_des - bias)
    if kp_null != 0.0 or kd_null != 0.0:
        qddot = qddot + null_space_projector(J, J_pinv) @ (
            -kp_null * joint_limit_gradient(q) - kd_null * qdot)

    scale = 1.0
    if respect_limits:
        over = float(np.max(np.abs(qddot) / QDDOT_MAX))
        if over > 1.0:
            scale = 1.0 / over
            qddot = qddot * scale

    achieved = J @ qddot + bias
    return {'qddot': qddot, 'achieved': achieved, 'bias': bias,
            'residual': float(np.linalg.norm(a_des - achieved)),
            'sigma_min': sigma_min, 'damping': lam, 'scale': scale}


# ==============================================================================
# UTILITIES
# ==============================================================================

def rot_to_quat(R):
    """3x3 rotation matrix -> quaternion [x, y, z, w]."""
    tr = np.trace(R)
    if tr > 0:
        S = np.sqrt(tr + 1.0) * 2
        return [(R[2, 1] - R[1, 2]) / S, (R[0, 2] - R[2, 0]) / S,
                (R[1, 0] - R[0, 1]) / S, 0.25 * S]
    if R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        S = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        return [0.25 * S, (R[0, 1] + R[1, 0]) / S,
                (R[0, 2] + R[2, 0]) / S, (R[2, 1] - R[1, 2]) / S]
    if R[1, 1] > R[2, 2]:
        S = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        return [(R[0, 1] + R[1, 0]) / S, 0.25 * S,
                (R[1, 2] + R[2, 1]) / S, (R[0, 2] - R[2, 0]) / S]
    S = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
    return [(R[0, 2] + R[2, 0]) / S, (R[1, 2] + R[2, 1]) / S,
            0.25 * S, (R[1, 0] - R[0, 1]) / S]


def print_dh_table():
    """The DH table with numbers, as used by the code."""
    print(" i |  theta_i                 |   d_i [m]  |   a_i [m]  | alpha_i")
    for i, (sign, offset, d, a, alpha) in enumerate(DH_TABLE):
        th = f"{'-' if sign < 0 else ' '}q{i + 1} {offset:+.6f}"
        print(f" {i + 1} |  {th:23s} | {d:+.6f}  | {a:+.6f}  | {alpha:+.4f}")


# ==============================================================================
# 9. SELF-TEST  (no ROS, no simulator)
# ==============================================================================

def selftest(n_samples=200, seed=0):
    """
    Over random postures, checks that
      * the Jacobian (Z x (O_N - O) rule) equals the differentiated FK
      * the finite-difference J_dot equals the closed-form J_dot
      * X_ddot = J q_ddot + J_dot q_dot equals d(X_dot)/dt
      * inverse velocity:     J (J# X_dot)            == X_dot
      * the null-space term leaves the tool velocity unchanged
      * inverse acceleration: J q_ddot + J_dot q_dot  == X_ddot
      * Newton-Raphson reaches reachable targets
    The DH model itself is checked against MuJoCo by
        python3 mujoco_bridge.py --compare
    """
    print_dh_table()
    rng = np.random.default_rng(seed)
    e_jac = e_jdot = e_acc = e_vel = e_null = e_iacc = e_ik = 0.0
    h = 1e-6
    tested = 0
    for _ in range(n_samples):
        q = rng.uniform(Q_MIN, Q_MAX)
        qdot = rng.uniform(-2.0, 2.0, N_JOINTS)
        qddot = rng.uniform(-5.0, 5.0, N_JOINTS)
        J = jacobian(q)

        e_jac = max(e_jac, np.abs(J - numerical_jacobian(q)).max())
        e_jdot = max(e_jdot, np.abs(jacobian_dot_fd(q, qdot)
                                    - jacobian_dot_analytic(q, qdot)).max())

        # X_ddot against d(X_dot)/dt along q(t) = q + t q_dot + t^2/2 q_ddot
        a, _, _, _ = forward_acceleration_kinematics(q, qdot, qddot)
        vp = jacobian(q + h * qdot + 0.5 * h * h * qddot) @ (qdot + h * qddot)
        vm = jacobian(q - h * qdot + 0.5 * h * h * qddot) @ (qdot - h * qddot)
        e_acc = max(e_acc, np.abs(a - (vp - vm) / (2 * h)).max())

        if np.linalg.svd(J, compute_uv=False)[-1] < 0.05:
            continue                                   # inverse checks: non-singular only
        tested += 1
        v_des = rng.uniform(-0.2, 0.2, 3)
        r = inverse_velocity_kinematics(q, v_des, k_null=0.0, respect_limits=False)
        e_vel = max(e_vel, np.abs(J @ r['qdot'] - v_des).max())
        rn = inverse_velocity_kinematics(q, v_des, k_null=2.0, respect_limits=False)
        e_null = max(e_null, np.abs(J @ rn['qdot'] - v_des).max())
        ra = inverse_acceleration_kinematics(q, qdot, rng.uniform(-1, 1, 3),
                                             respect_limits=False)
        e_iacc = max(e_iacc, ra['residual'])

        # Newton-Raphson from a nearby seed back to this posture's tool point
        q_sol, err = inverse_kinematics(ee_position(q), q + rng.uniform(-0.3, 0.3, 5),
                                        tol=1e-6, max_iter=200)
        e_ik = max(e_ik, err)

    print(f"Jacobian  vs differentiated FK      : {e_jac:.3e} m/rad")
    print(f"J_dot     finite diff vs closed form: {e_jdot:.3e} 1/s")
    print(f"X_ddot    vs d(X_dot)/dt            : {e_acc:.3e} m/s^2")
    print(f"inverse velocity   (postures {tested:3d})  : {e_vel:.3e} m/s")
    print(f"null-space task disturbance         : {e_null:.3e} m/s")
    print(f"inverse acceleration residual       : {e_iacc:.3e} m/s^2")
    print(f"Newton-Raphson worst final error    : {e_ik:.3e} m")
    ok = (e_jac < 1e-6 and e_jdot < 1e-4 and e_acc < 1e-4 and e_vel < 1e-9
          and e_null < 1e-9 and e_iacc < 1e-9 and e_ik < 1e-4)
    print("SELF-TEST PASSED" if ok else "SELF-TEST FAILED")
    return ok


if __name__ == '__main__':
    if '--selftest' in sys.argv:
        sys.exit(0 if selftest() else 1)
    print_dh_table()
