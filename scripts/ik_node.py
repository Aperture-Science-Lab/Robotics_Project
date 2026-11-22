# #!/usr/bin/env python3
# """
# Inverse Kinematics Node for low_cost_robot_arm
# Pure NumPy implementation (Damped Least Squares / Levenberg-Marquardt)
# No SciPy required.
# """

# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import JointState
# from std_msgs.msg import Float64MultiArray
# from geometry_msgs.msg import PoseStamped
# import numpy as np

# # ==============================================================================
# # FORWARD KINEMATICS (Pure NumPy)
# # ==============================================================================

# def rot_x(q):
#     c, s = np.cos(q), np.sin(q)
#     return np.array([[1, 0, 0, 0], [0, c, -s, 0], [0, s, c, 0], [0, 0, 0, 1]])

# def rot_y(q):
#     c, s = np.cos(q), np.sin(q)
#     return np.array([[c, 0, s, 0], [0, 1, 0, 0], [-s, 0, c, 0], [0, 0, 0, 1]])

# def rot_z(q):
#     c, s = np.cos(q), np.sin(q)
#     return np.array([[c, -s, 0, 0], [s, c, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])

# def trans(x, y, z):
#     return np.array([[1, 0, 0, x], [0, 1, 0, y], [0, 0, 1, z], [0, 0, 0, 1]])

# def forward_kinematics(q):
#     """Computes reached end-effector [X, Y, Z] in meters."""
#     T1 = trans(0.012, 0.0, 0.0409) @ rot_z(-q[0])
#     T2 = trans(0.0, -0.0209, 0.0154) @ rot_y(q[1])
#     T3 = trans(-0.0148, 0.0065, 0.1083) @ rot_y(-q[2])
#     T4 = trans(-0.10048, 5e-05, 0.0026999) @ rot_y(q[3])
#     T5 = trans(-0.045, 0.013097, 0.0) @ rot_x(q[4])

#     T_ee = np.array([
#         [ 0.0,  0.0, -1.0, -0.065 ],
#         [ 0.0,  1.0,  0.0,  0.0105],
#         [ 1.0,  0.0,  0.0,  0.0005],
#         [ 0.0,  0.0,  0.0,  1.0   ]
#     ])
#     T = T1 @ T2 @ T3 @ T4 @ T5 @ T_ee
#     return T[:3, 3]

# # Joint limits from low_cost_robot_arm.xml
# Q_MIN = np.array([-2.2, -1.57, -1.57, -2.0, -3.14])
# Q_MAX = np.array([ 2.2,  0.60,  1.45,  2.0,  3.14])

# # ==============================================================================
# # PURE NUMPY INVERSE KINEMATICS (Damped Least Squares)
# # ==============================================================================

# def numerical_jacobian(q, eps=1e-5):
#     """Computes 3x5 position Jacobian using central differences."""
#     J = np.zeros((3, 5))
#     for i in range(5):
#         q_p = q.copy()
#         q_m = q.copy()
#         q_p[i] += eps
#         q_m[i] -= eps
#         J[:, i] = (forward_kinematics(q_p) - forward_kinematics(q_m)) / (2.0 * eps)
#     return J

# def solve_ik_numpy(target_pos, q_seed, max_iter=60, tol=1e-3):
#     """
#     Solves IK via Damped Least Squares without external dependencies.
#     """
#     q = np.clip(q_seed.copy(), Q_MIN, Q_MAX)
#     lambda_sq = 1e-4  # Damping factor to handle singularities

#     for _ in range(max_iter):
#         p_curr = forward_kinematics(q)
#         error = target_pos - p_curr

#         if np.linalg.norm(error) < tol:
#             break

#         J = numerical_jacobian(q)
#         # delta_q = J^T * (J * J^T + lambda^2 * I)^(-1) * error
#         JJT = J @ J.T
#         damped = JJT + lambda_sq * np.eye(3)
#         delta_q = J.T @ np.linalg.solve(damped, error)

#         # Step clamp for numerical stability
#         step_len = np.linalg.norm(delta_q)
#         if step_len > 0.15:
#             delta_q = delta_q * (0.15 / step_len)

#         q = np.clip(q + delta_q, Q_MIN, Q_MAX)

#     final_err = np.linalg.norm(target_pos - forward_kinematics(q))
#     return q, final_err

# # ==============================================================================
# # ROS 2 NODE
# # ==============================================================================

# class InverseKinematicsNode(Node):
#     def __init__(self):
#         super().__init__('inverse_kinematics_node')

#         self.q_current = np.zeros(5)
#         self.gripper_pos = 0.0

#         # Subscriptions
#         self.create_subscription(JointState, '/joint_state', self.on_joint_state, 10)
#         self.create_subscription(PoseStamped, '/target_pose', self.on_target_pose, 10)

#         # Publisher for actuator commands
#         self.cmd_pub = self.create_publisher(Float64MultiArray, '/joint_commands', 10)

#         self.get_logger().info("=== [IK NODE] Initialized successfully with Pure NumPy Solver ===")

#     def on_joint_state(self, msg: JointState):
#         if len(msg.position) >= 5:
#             self.q_current = np.array(msg.position[:5])
#         if len(msg.position) >= 6:
#             self.gripper_pos = float(msg.position[5])

#     def on_target_pose(self, msg: PoseStamped):
#         tx = msg.pose.position.x
#         ty = msg.pose.position.y
#         tz = msg.pose.position.z
#         target_pos = np.array([tx, ty, tz])

#         # Solve IK using current configuration as seed
#         q_sol, err_meters = solve_ik_numpy(target_pos, self.q_current)
#         err_mm = err_meters * 1000.0

#         if err_meters < 0.010:  # Within 10 mm
#             self.get_logger().info(
#                 f"[IK SUCCESS] Target ({tx:+.3f}, {ty:+.3f}, {tz:+.3f}) -> Err: {err_mm:.1f} mm | Joints: {[round(float(v), 2) for v in q_sol]}"
#             )
#         else:
#             self.get_logger().warn(
#                 f"[IK REACH LIMIT] Target ({tx:+.3f}, {ty:+.3f}, {tz:+.3f}) closest err: {err_mm:.1f} mm"
#             )

#         # Publish solution to move the arm
#         cmd = Float64MultiArray()
#         cmd.data = [float(v) for v in q_sol] + [float(self.gripper_pos)]
#         self.cmd_pub.publish(cmd)


# def main(args=None):
#     rclpy.init(args=args)
#     node = InverseKinematicsNode()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()

#!/usr/bin/env python3
"""
Inverse Kinematics + Inverse Velocity + Inverse Acceleration Node
for low_cost_robot_arm.
Pure NumPy implementation (Damped Least Squares / Levenberg-Marquardt).
No SciPy required.

Position     level :  target [X, Y, Z]      ->  joint angles        q
Velocity     level :  target [Vx, Vy, Vz]   ->  joint rates         qdot
Acceleration level :  target [Ax, Ay, Az]   ->  joint accelerations qddot

TRANSLATION ONLY: the task space is the 3D position of the tool tip.
Orientation is not tracked, so every Jacobian here is 3x5.

                 .     #
                 q  =  J  v_des                            (inverse velocity)
                 ..    #         .     .
                 q  =  J ( a_des - Jdot q )                (inverse acceleration)

With 3 task rows and 5 joints the arm is kinematically redundant: J is not
square, so J# is the damped right pseudo-inverse, and the leftover
2-dimensional null space is used for joint-limit avoidance.
"""

import sys

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, MultiArrayDimension
from geometry_msgs.msg import PoseStamped, Vector3Stamped
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
Q_MID = 0.5 * (Q_MIN + Q_MAX)
Q_RANGE = Q_MAX - Q_MIN

# Actuator rate limits (rad/s, rad/s^2) - adjust to your servos
QDOT_MAX = np.array([3.0, 3.0, 3.0, 3.0, 3.0])
QDDOT_MAX = np.array([20.0, 20.0, 20.0, 20.0, 20.0])

# ==============================================================================
# GEOMETRIC JACOBIAN:  J_i = Z_(i-1) x (P_ee - P_(i-1))
# ==============================================================================

# The same chain as forward_kinematics(), split into the FIXED link offset and
# the JOINT rotation, because Z_(i-1) and P_(i-1) are needed separately. The
# products are identical to T1..T5 above; selftest() checks they agree exactly.
LINK_OFFSETS = [
    trans(0.012,    0.0,      0.0409),     # base    -> joint 1 (base_rotation)
    trans(0.0,     -0.0209,   0.0154),     # joint 1 -> joint 2 (pitch)
    trans(-0.0148,  0.0065,   0.1083),     # joint 2 -> joint 3 (elbow)
    trans(-0.10048, 5e-05,    0.0026999),  # joint 3 -> joint 4 (wrist_pitch)
    trans(-0.045,   0.013097, 0.0),        # joint 4 -> joint 5 (wrist_roll)
]

# Unit rotation axis of each joint, in the frame that PRECEDES it. This arm is
# defined in MuJoCo, where each joint names its own axis, so Z_(i-1) is that
# axis rotated into frame_0 rather than the z-column of a DH frame. The signs
# of rot_z(-q0) and rot_y(-q2) are carried here, so no sign fixes are needed.
JOINT_AXES = [
    np.array([0.0,  0.0, -1.0]),   # q1 base_rotation : axis 0 0 -1
    np.array([0.0,  1.0,  0.0]),   # q2 pitch         : axis 0 1 0
    np.array([0.0, -1.0,  0.0]),   # q3 elbow         : axis 0 -1 0
    np.array([0.0,  1.0,  0.0]),   # q4 wrist_pitch   : axis 0 1 0
    np.array([1.0,  0.0,  0.0]),   # q5 wrist_roll    : axis 1 0 0
]

T_TOOL = np.array([
    [0.0, 0.0, -1.0, -0.065 ],
    [0.0, 1.0,  0.0,  0.0105],
    [1.0, 0.0,  0.0,  0.0005],
    [0.0, 0.0,  0.0,  1.0   ]
])


def _joint_rotation(i, qi):
    """Rotation of joint i by angle qi about its own axis."""
    if i == 0:
        return rot_z(-qi)      # rotation by qi about [0, 0, -1]
    if i == 1:
        return rot_y(qi)       # rotation by qi about [0, 1, 0]
    if i == 2:
        return rot_y(-qi)      # rotation by qi about [0, -1, 0]
    if i == 3:
        return rot_y(qi)       # rotation by qi about [0, 1, 0]
    return rot_x(qi)           # rotation by qi about [1, 0, 0]


def chain_frames(q):
    """
    One sweep along the chain.

    Returns:
        p_ee    : tool-tip position P_ee, in frame_0
        origins : the 5 joint origins P_(i-1), in frame_0
        axes    : the 5 unit joint axes Z_(i-1), in frame_0
    """
    q = np.asarray(q, dtype=float).reshape(-1)
    T = np.eye(4)
    origins, axes = [], []
    for i in range(5):
        T = T @ LINK_OFFSETS[i]
        # P_(i-1) and Z_(i-1) are read BEFORE joint i turns: rotating about an
        # axis moves neither that axis nor a point lying on it.
        origins.append(T[:3, 3].copy())
        axes.append(T[:3, :3] @ JOINT_AXES[i])
        T = T @ _joint_rotation(i, q[i])
    T = T @ T_TOOL
    return T[:3, 3].copy(), origins, axes


def jacobian(q):
    """
    3x5 position Jacobian in frame_0, built column by column with

        J_i = Z_(i-1) x (P_ee - P_(i-1))

    Only the translational rows are formed - the angular rows Z_(i-1) are
    intentionally omitted, since orientation is not part of the task.

    Returns:
        J : 3x5 matrix mapping qdot [rad/s] to tool velocity [m/s]
    """
    p_ee, origins, axes = chain_frames(q)
    J = np.zeros((3, 5))
    for i in range(5):
        J[:, i] = np.cross(axes[i], p_ee - origins[i])
    return J


def jacobian_dot(q, qdot, step=1e-5):
    """
    Jdot by the central finite difference:

        Jdot  ~=  ( J(q + h*qdot) - J(q - h*qdot) ) / (2h)

    Valid because J depends on time only through q, so by the chain rule
    dJ/dt = (dJ/dq) . qdot, which is exactly the directional derivative of J
    along qdot.

    The step is scaled as h = step / ||qdot||, so the perturbation h*qdot
    always has length `step` however fast the arm is moving. A fixed h would
    perturb by almost nothing when the arm is nearly stopped (the difference
    would be pure round-off) and by a large angle when it is fast (truncation
    error). When the arm is stopped, Jdot*qdot is zero anyway, so zeros are
    returned.

    step=1e-5 measured as the best truncation/round-off tradeoff for this chain.

    Returns:
        Jdot : 3x5 matrix [1/s]
    """
    qdot = np.asarray(qdot, dtype=float).reshape(-1)[:5]
    speed = float(np.linalg.norm(qdot))
    if speed < 1e-12:
        return np.zeros((3, 5))

    h = step / speed
    q = np.asarray(q, dtype=float).reshape(-1)
    return (jacobian(q + h * qdot) - jacobian(q - h * qdot)) / (2.0 * h)


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

# ==============================================================================
# PURE NUMPY INVERSE KINEMATICS (Damped Least Squares)
# ==============================================================================

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

        J = jacobian(q)
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
# DAMPED PSEUDO-INVERSE AND NULL SPACE
# ==============================================================================

def damped_pinv(J, lam_max=0.08, sigma_thresh=0.02):
    """
    Damped right pseudo-inverse  J# = J^T (J J^T + lam^2 I)^-1.

    The right inverse (inverting the 3x3 J J^T, not the 5x5 J^T J) is the one
    to use when the task has fewer rows than joints; with lam = 0 it returns
    the minimum-norm solution of J qdot = v.

    The damping is adaptive: zero while the task is well conditioned, growing
    smoothly to lam_max as the smallest singular value drops below
    sigma_thresh. That trades a little tracking accuracy for bounded joint
    rates near singularities, instead of the blow-up an undamped inverse gives.

    Returns (J#, sigma_min, lam).
    """
    sigma_min = float(np.linalg.svd(J, compute_uv=False)[-1])
    if sigma_min >= sigma_thresh:
        lam = 0.0
    else:
        lam = lam_max * np.sqrt(max(1.0 - (sigma_min / sigma_thresh) ** 2, 0.0))
    return J.T @ np.linalg.inv(J @ J.T + (lam ** 2) * np.eye(3)), sigma_min, lam


def joint_limit_gradient(q):
    """
    Gradient of H(q) = 1/2 * sum(((q - q_mid)/q_range)^2). Moving along -grad(H)
    pushes the posture back toward mid-range; this is what the redundant
    (null-space) degrees of freedom are spent on.
    """
    return (np.asarray(q, dtype=float) - Q_MID) / (Q_RANGE ** 2)

# ==============================================================================
# INVERSE VELOCITY KINEMATICS
# ==============================================================================

def inverse_velocity_kinematics(q, v_des, k_null=1.0, respect_limits=True):
    """
    Maps a desired tool-tip velocity to joint rates (resolved-rate control):

        qdot = J# v_des  +  (I - J# J) qdot_0

    Parameters:
        q      : 5 current joint angles [rad]
        v_des  : desired [Vx, Vy, Vz] of the tool tip [m/s], in frame_0
        k_null : gain of the null-space joint-limit-avoidance term (0 disables)
        respect_limits : scale qdot down uniformly if any joint exceeds
                 QDOT_MAX, so the Cartesian DIRECTION is preserved (clamping
                 joints one by one would bend the path)

    Returns dict: qdot, achieved velocity, residual, sigma_min, damping, scale.
    """
    v_des = np.asarray(v_des, dtype=float).reshape(-1)[:3]
    J = jacobian(q)
    J_pinv, sigma_min, lam = damped_pinv(J)

    qdot = J_pinv @ v_des

    # Null-space term: drift toward mid-range without disturbing the task
    if k_null != 0.0:
        N = np.eye(5) - J_pinv @ J
        qdot = qdot + N @ (-k_null * joint_limit_gradient(q))

    scale = 1.0
    if respect_limits:
        over = np.abs(qdot) / QDOT_MAX
        if over.max() > 1.0:
            scale = 1.0 / over.max()
            qdot = qdot * scale

    achieved = J @ qdot
    return {
        'qdot': qdot,
        'achieved': achieved,
        'residual': float(np.linalg.norm(v_des - achieved)),
        'sigma_min': sigma_min,
        'damping': lam,
        'scale': scale,
    }

# ==============================================================================
# INVERSE ACCELERATION KINEMATICS
# ==============================================================================

def inverse_acceleration_kinematics(q, qdot, a_des, kp_null=4.0, kd_null=2.0,
                                    respect_limits=True, step=1e-5):
    """
    Maps a desired tool-tip acceleration to joint accelerations:

        qddot = J# ( a_des - Jdot qdot )  +  (I - J# J) qddot_0

    The -Jdot*qdot term removes the centripetal / Coriolis acceleration the
    tool already carries from the motion in progress; dropping it is the usual
    reason an acceleration-level controller drifts off its path. Jdot comes
    from the finite-difference rule in jacobian_dot().

    The null-space term is a PD law that damps the internal motion and pulls
    the posture toward mid-range, so the redundant joints do not wander while
    the task acceleration is tracked exactly.

    Returns dict: qddot, achieved acceleration, bias, residual, sigma_min,
    damping, scale.
    """
    a_des = np.asarray(a_des, dtype=float).reshape(-1)[:3]
    qdot = np.asarray(qdot, dtype=float).reshape(-1)[:5]

    J = jacobian(q)
    Jdot = jacobian_dot(q, qdot, step=step)
    bias = Jdot @ qdot                       # centripetal / Coriolis part

    J_pinv, sigma_min, lam = damped_pinv(J)
    qddot = J_pinv @ (a_des - bias)

    if kp_null != 0.0 or kd_null != 0.0:
        N = np.eye(5) - J_pinv @ J
        qddot = qddot + N @ (-kp_null * joint_limit_gradient(q) - kd_null * qdot)

    scale = 1.0
    if respect_limits:
        over = np.abs(qddot) / QDDOT_MAX
        if over.max() > 1.0:
            scale = 1.0 / over.max()
            qddot = qddot * scale

    achieved = J @ qddot + bias
    return {
        'qddot': qddot,
        'achieved': achieved,
        'bias': bias,
        'residual': float(np.linalg.norm(a_des - achieved)),
        'sigma_min': sigma_min,
        'damping': lam,
        'scale': scale,
    }

# ==============================================================================
# ROS 2 NODE
# ==============================================================================

class InverseKinematicsNode(Node):
    def __init__(self):
        super().__init__('inverse_kinematics_node')

        # Integrate resolved joint rates into position commands, so the arm
        # actually servos when only a velocity is streamed in
        self.declare_parameter('integrate_velocity', True)
        self.declare_parameter('k_null', 1.0)
        self.declare_parameter('fd_step', 1e-5)
        self.integrate_velocity = self.get_parameter('integrate_velocity').value
        self.k_null = float(self.get_parameter('k_null').value)
        self.fd_step = float(self.get_parameter('fd_step').value)

        self.q_current = np.zeros(5)
        self.qdot_current = np.zeros(5)
        self.gripper_pos = 0.0
        self.last_vel_time = None

        # Subscriptions
        self.create_subscription(JointState, '/joint_state', self.on_joint_state, 10)
        self.create_subscription(PoseStamped, '/target_pose', self.on_target_pose, 10)
        self.create_subscription(Vector3Stamped, '/target_velocity', self.on_target_velocity, 10)
        self.create_subscription(Vector3Stamped, '/target_accel', self.on_target_accel, 10)

        # Publishers for actuator commands
        self.cmd_pub = self.create_publisher(Float64MultiArray, '/joint_commands', 10)
        self.vel_pub = self.create_publisher(Float64MultiArray, '/joint_velocity_commands', 10)
        self.acc_pub = self.create_publisher(Float64MultiArray, '/joint_acceleration_commands', 10)

        self.get_logger().info(
            "=== [IK NODE] Position / Velocity / Acceleration solver ready "
            "(Pure NumPy, position-only task space) ===")

    # ------------------------------------------------------------------
    def on_joint_state(self, msg: JointState):
        if len(msg.position) >= 5:
            self.q_current = np.array(msg.position[:5])
        if len(msg.position) >= 6:
            self.gripper_pos = float(msg.position[5])
        if len(msg.velocity) >= 5:
            self.qdot_current = np.array(msg.velocity[:5])

    # ------------------------------------------------------------------
    # POSITION LEVEL
    # ------------------------------------------------------------------
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

    # ------------------------------------------------------------------
    # VELOCITY LEVEL
    # ------------------------------------------------------------------
    def on_target_velocity(self, msg: Vector3Stamped):
        v_des = np.array([msg.vector.x, msg.vector.y, msg.vector.z])

        res = inverse_velocity_kinematics(self.q_current, v_des, k_null=self.k_null)
        qdot = res['qdot']

        self.get_logger().info(
            f"[IVK] v_des ({v_des[0]:+.3f}, {v_des[1]:+.3f}, {v_des[2]:+.3f}) m/s -> "
            f"qdot {[round(float(v), 3) for v in qdot]} rad/s | "
            f"residual {res['residual']*1000:.2f} mm/s | sigma_min {res['sigma_min']:.3e}"
        )
        if res['damping'] > 0.0:
            self.get_logger().warn(
                f"[IVK] Near singularity: damping {res['damping']:.4f} applied, "
                f"task velocity not tracked exactly"
            )
        if res['scale'] < 1.0:
            self.get_logger().warn(
                f"[IVK] Joint speed limit hit: command scaled to {res['scale']*100:.0f}%"
            )

        self.vel_pub.publish(self._array_msg(qdot))
        self.qdot_current = qdot

        # Resolved-rate servoing: integrate qdot into a position command
        if self.integrate_velocity:
            now = self.get_clock().now().nanoseconds * 1e-9
            dt = 0.0 if self.last_vel_time is None else now - self.last_vel_time
            self.last_vel_time = now
            if 0.0 < dt < 0.5:
                q_next = np.clip(self.q_current + qdot * dt, Q_MIN, Q_MAX)
                self.q_current = q_next
                cmd = Float64MultiArray()
                cmd.data = [float(v) for v in q_next] + [float(self.gripper_pos)]
                self.cmd_pub.publish(cmd)

    # ------------------------------------------------------------------
    # ACCELERATION LEVEL
    # ------------------------------------------------------------------
    def on_target_accel(self, msg: Vector3Stamped):
        a_des = np.array([msg.vector.x, msg.vector.y, msg.vector.z])

        res = inverse_acceleration_kinematics(self.q_current, self.qdot_current,
                                              a_des, step=self.fd_step)
        qddot = res['qddot']

        self.get_logger().info(
            f"[IAK] a_des ({a_des[0]:+.3f}, {a_des[1]:+.3f}, {a_des[2]:+.3f}) m/s^2 -> "
            f"qddot {[round(float(v), 3) for v in qddot]} rad/s^2 | "
            f"bias |Jdot*qdot| {np.linalg.norm(res['bias']):.4f} m/s^2 | "
            f"residual {res['residual']:.4f}"
        )
        if res['scale'] < 1.0:
            self.get_logger().warn(
                f"[IAK] Joint accel limit hit: command scaled to {res['scale']*100:.0f}%"
            )

        self.acc_pub.publish(self._array_msg(qddot))

    # ------------------------------------------------------------------
    @staticmethod
    def _array_msg(vec):
        msg = Float64MultiArray()
        msg.layout.dim = [MultiArrayDimension(label="joints", size=5, stride=5)]
        msg.data = [float(v) for v in vec]
        return msg


# ==============================================================================
# SELF-TEST
# ==============================================================================

def selftest(n_samples=150, seed=0):
    """
    Checks, over random non-singular postures:
      * chain_frames() reproduces forward_kinematics() exactly
      * the Z x (P_ee - P) Jacobian against the finite-difference one
      * inverse velocity:     J (J# v_des)          == v_des
      * the null-space term does not disturb the task
      * inverse acceleration: J qddot + Jdot qdot   == a_des
      * the commanded qddot really produces a_des   (vs d(v)/dt)
    Run with:  python3 inverse_kinematics_node.py --selftest
    """
    rng = np.random.default_rng(seed)
    e_fk = e_jac = e_vel = e_null = e_acc = e_fd = 0.0
    tested = 0
    h = 1e-6

    for _ in range(n_samples):
        q = rng.uniform(Q_MIN, Q_MAX)
        J = jacobian(q)
        if np.linalg.svd(J, compute_uv=False)[-1] < 0.05:
            continue                      # skip near-singular draws
        tested += 1

        p_ee, _, _ = chain_frames(q)
        e_fk = max(e_fk, np.abs(p_ee - forward_kinematics(q)).max())
        e_jac = max(e_jac, np.abs(J - numerical_jacobian(q)).max())

        v_des = rng.uniform(-0.2, 0.2, 3)
        r = inverse_velocity_kinematics(q, v_des, k_null=0.0, respect_limits=False)
        e_vel = max(e_vel, np.abs(J @ r['qdot'] - v_des).max())

        rn = inverse_velocity_kinematics(q, v_des, k_null=2.0, respect_limits=False)
        e_null = max(e_null, np.abs(J @ rn['qdot'] - v_des).max())

        qdot = rng.uniform(-1.0, 1.0, 5)
        a_des = rng.uniform(-1.0, 1.0, 3)
        ra = inverse_acceleration_kinematics(q, qdot, a_des, respect_limits=False)
        e_acc = max(e_acc, np.abs(ra['achieved'] - a_des).max())

        qddot = ra['qddot']
        vp = jacobian(q + h*qdot + 0.5*h*h*qddot) @ (qdot + h*qddot)
        vm = jacobian(q - h*qdot + 0.5*h*h*qddot) @ (qdot - h*qddot)
        e_fd = max(e_fd, np.abs((vp - vm) / (2*h) - a_des).max())

    print(f"postures tested                   : {tested}")
    print(f"max chain_frames vs forward_kin   : {e_fk:.3e} m")
    print(f"max Jacobian error (vs numerical) : {e_jac:.3e} m/rad")
    print(f"max inverse-velocity error        : {e_vel:.3e} m/s")
    print(f"max null-space task disturbance   : {e_null:.3e} m/s")
    print(f"max inverse-acceleration error    : {e_acc:.3e} m/s^2")
    print(f"max realized-acceleration error   : {e_fd:.3e} m/s^2")
    ok = (e_fk < 1e-12 and e_jac < 1e-6 and e_vel < 1e-9
          and e_null < 1e-9 and e_acc < 1e-9 and e_fd < 1e-4)
    print("SELF-TEST PASSED" if ok else "SELF-TEST FAILED")
    return ok


def main(args=None):
    if args is None and '--selftest' in sys.argv:
        sys.exit(0 if selftest() else 1)

    rclpy.init(args=args)
    node = InverseKinematicsNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()