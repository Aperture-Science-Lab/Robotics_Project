# #!/usr/bin/env python3
# """
# Forward Kinematics Node for low_cost_robot_arm
# Takes:  Joint angles q1, q2, q3, q4, q5
# Gives:  Reached Cartesian coordinates [X, Y, Z] (meters) of the end-effector
# """

# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import JointState
# from std_msgs.msg import Float64MultiArray
# from geometry_msgs.msg import PoseStamped, Point
# import numpy as np

# # ==============================================================================
# # DH / KINEMATICS IMPLEMENTATION (Derived from low_cost_robot_arm.xml)
# # ==============================================================================

# def rot_x(angle):
#     c, s = np.cos(angle), np.sin(angle)
#     return np.array([[1, 0, 0, 0], [0, c, -s, 0], [0, s, c, 0], [0, 0, 0, 1]])

# def rot_y(angle):
#     c, s = np.cos(angle), np.sin(angle)
#     return np.array([[c, 0, s, 0], [0, 1, 0, 0], [-s, 0, c, 0], [0, 0, 0, 1]])

# def rot_z(angle):
#     c, s = np.cos(angle), np.sin(angle)
#     return np.array([[c, -s, 0, 0], [s, c, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])

# def trans(x, y, z):
#     return np.array([[1, 0, 0, x], [0, 1, 0, y], [0, 0, 1, z], [0, 0, 0, 1]])


# def forward_kinematics(q):
#     """
#     Computes the Forward Kinematics from base to the end-effector site (frame_ee).
    
#     Parameters:
#         q: list or numpy array of 5 joint angles [q1, q2, q3, q4, q5] in radians:
#            q[0] -> base_rotation  (axis 0 0 -1)
#            q[1] -> pitch          (axis 0 1 0)
#            q[2] -> elbow          (axis 0 -1 0)
#            q[3] -> wrist_pitch    (axis 0 1 0)
#            q[4] -> wrist_roll     (axis 1 0 0)

#     Returns:
#         x, y, z : The 3D Cartesian coordinates reached by the end-effector (meters)
#         T       : The full 4x4 homogeneous transformation matrix
#     """
#     # 1. Base to Shoulder (joint 1: axis 0 0 -1)
#     T1 = trans(0.012, 0.0, 0.0409) @ rot_z(-q[0])

#     # 2. Shoulder to Elbow (joint 2: axis 0 1 0)
#     T2 = trans(0.0, -0.0209, 0.0154) @ rot_y(q[1])

#     # 3. Elbow Extension (joint 3: axis 0 -1 0)
#     T3 = trans(-0.0148, 0.0065, 0.1083) @ rot_y(-q[2])

#     # 4. Wrist Extension (joint 4: axis 0 1 0)
#     T4 = trans(-0.10048, 5e-05, 0.0026999) @ rot_y(q[3])

#     # 5. Gripper Base (joint 5: axis 1 0 0)
#     T5 = trans(-0.045, 0.013097, 0.0) @ rot_x(q[4])

#     # 6. End-Effector Tool Tip (site frame_ee in XML)
#     # xyaxes="0 0 1 0 1 0" -> x_axis=[0,0,1], y_axis=[0,1,0], z_axis=[-1,0,0]
#     T_ee = np.array([
#         [ 0.0,  0.0, -1.0, -0.065 ],
#         [ 0.0,  1.0,  0.0,  0.0105],
#         [ 1.0,  0.0,  0.0,  0.0005],
#         [ 0.0,  0.0,  0.0,  1.0   ]
#     ])

#     # Cumulative chain
#     T = T1 @ T2 @ T3 @ T4 @ T5 @ T_ee

#     # Extract reached X, Y, Z (meters)
#     x = float(T[0, 3])
#     y = float(T[1, 3])
#     z = float(T[2, 3])

#     return x, y, z, T


# def rot_to_quat(R):
#     """Utility to convert 3x3 rotation matrix to quaternion [x, y, z, w]."""
#     tr = np.trace(R)
#     if tr > 0:
#         S = np.sqrt(tr + 1.0) * 2
#         w = 0.25 * S
#         x = (R[2, 1] - R[1, 2]) / S
#         y = (R[0, 2] - R[2, 0]) / S
#         z = (R[1, 0] - R[0, 1]) / S
#     elif (R[0, 0] > R[1, 1]) and (R[0, 0] > R[2, 2]):
#         S = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
#         w = (R[2, 1] - R[1, 2]) / S
#         x = 0.25 * S
#         y = (R[0, 1] + R[1, 0]) / S
#         z = (R[0, 2] + R[2, 0]) / S
#     elif R[1, 1] > R[2, 2]:
#         S = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
#         w = (R[0, 2] - R[2, 0]) / S
#         x = (R[0, 1] + R[1, 0]) / S
#         y = 0.25 * S
#         z = (R[1, 2] + R[2, 1]) / S
#     else:
#         S = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
#         w = (R[1, 0] - R[0, 1]) / S
#         x = (R[0, 2] + R[2, 0]) / S
#         y = (R[1, 2] + R[2, 1]) / S
#         z = 0.25 * S
#     return [x, y, z, w]

# # ==============================================================================
# # ROS 2 FORWARD KINEMATICS NODE
# # ==============================================================================

# class ForwardKinematicsNode(Node):
#     def __init__(self):
#         super().__init__('forward_kinematics_node')

#         # Subscriptions: reads joint positions from simulation or GUI commands
#         self.create_subscription(JointState, '/joint_state', self.joint_state_cb, 10)
#         self.create_subscription(Float64MultiArray, '/joint_commands', self.joint_cmd_cb, 10)

#         # Publishers: publishes the reached (X, Y, Z) pose
#         self.pose_pub = self.create_publisher(PoseStamped, '/current_ee_pose', 10)
#         self.xyz_pub = self.create_publisher(Point, '/current_ee_xyz', 10)

#         self.last_q = None
#         self.get_logger().info("Forward Kinematics Node ready. Waiting for joint inputs...")

#     def joint_state_cb(self, msg: JointState):
#         if len(msg.position) >= 5:
#             self.compute_and_report(np.array(msg.position[:5]))

#     def joint_cmd_cb(self, msg: Float64MultiArray):
#         if len(msg.data) >= 5:
#             self.compute_and_report(np.array(msg.data[:5]))

#     def compute_and_report(self, q):
#         # Ignore computation if angles haven't changed
#         if self.last_q is not None and np.allclose(self.last_q, q, atol=1e-4):
#             return
#         self.last_q = q.copy()

#         # ----------------------------------------------------
#         # 1. CALL FORWARD KINEMATICS -> GET (X, Y, Z)
#         # ----------------------------------------------------
#         x, y, z, T = forward_kinematics(q)

#         # ----------------------------------------------------
#         # 2. PRINT REACHED X, Y, Z TO TERMINAL
#         # ----------------------------------------------------
#         self.get_logger().info(
#             f"[FK Reached Position] -> X: {x:+.4f} m | Y: {y:+.4f} m | Z: {z:+.4f} m"
#         )

#         # ----------------------------------------------------
#         # 3. PUBLISH FOR GUI & OTHER NODES
#         # ----------------------------------------------------
#         # Publish pure (X, Y, Z) point
#         pt_msg = Point()
#         pt_msg.x = x
#         pt_msg.y = y
#         pt_msg.z = z
#         self.xyz_pub.publish(pt_msg)

#         # Publish full PoseStamped
#         quat = rot_to_quat(T[:3, :3])
#         pose_msg = PoseStamped()
#         pose_msg.header.stamp = self.get_clock().now().to_msg()
#         pose_msg.header.frame_id = "frame_0"
#         pose_msg.pose.position.x = x
#         pose_msg.pose.position.y = y
#         pose_msg.pose.position.z = z
#         pose_msg.pose.orientation.x = float(quat[0])
#         pose_msg.pose.orientation.y = float(quat[1])
#         pose_msg.pose.orientation.z = float(quat[2])
#         pose_msg.pose.orientation.w = float(quat[3])
#         self.pose_pub.publish(pose_msg)


# def main(args=None):
#     rclpy.init(args=args)
#     node = ForwardKinematicsNode()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()

#!/usr/bin/env python3
"""
Forward Kinematics + Velocity + Acceleration Node for low_cost_robot_arm

Takes:  Joint angles q1..q5, joint rates dq1..dq5, joint accelerations ddq1..ddq5
Gives:  Reached Cartesian position [X, Y, Z]       (m)
        End-effector linear velocity     [Vx, Vy, Vz]  (m/s)
        End-effector linear acceleration [Ax, Ay, Az]  (m/s^2)

TRANSLATION ONLY: the task space here is the 3D position of the tool tip.
Orientation / angular velocity / angular acceleration are deliberately NOT
computed, so every Jacobian in this file is 3x5.

    v = J(q) qdot
    a = J(q) qddot + Jdot(q, qdot) qdot
"""

import sys

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, MultiArrayDimension
from geometry_msgs.msg import PoseStamped, Point, Vector3Stamped
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


# ==============================================================================
# FORWARD VELOCITY KINEMATICS  (3x5 POSITION JACOBIAN)
# ==============================================================================

# The same chain as forward_kinematics(), but split into the FIXED link offset
# and the JOINT rotation, because the Jacobian needs each joint's origin o_i
# and its axis k_i separately. The products are identical to T1..T5 above;
# selftest() checks that the two agree to machine precision.
LINK_OFFSETS = [
    trans(0.012,    0.0,      0.0409),     # base    -> joint 1 (base_rotation)
    trans(0.0,     -0.0209,   0.0154),     # joint 1 -> joint 2 (pitch)
    trans(-0.0148,  0.0065,   0.1083),     # joint 2 -> joint 3 (elbow)
    trans(-0.10048, 5e-05,    0.0026999),  # joint 3 -> joint 4 (wrist_pitch)
    trans(-0.045,   0.013097, 0.0),        # joint 4 -> joint 5 (wrist_roll)
]

# Unit rotation axis of each joint, in the frame that PRECEDES it. The signs of
# rot_z(-q0) and rot_y(-q2) are carried here, so no sign fixes are needed later.
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
        p_ee    : tool-tip position [X, Y, Z] in frame_0
        origins : the 5 joint origins o_i, in frame_0
        axes    : the 5 unit joint axes k_i, in frame_0
    """
    q = np.asarray(q, dtype=float).reshape(-1)
    T = np.eye(4)
    origins, axes = [], []
    for i in range(5):
        T = T @ LINK_OFFSETS[i]
        # o_i and k_i are read BEFORE joint i turns: rotating about an axis
        # moves neither that axis nor a point lying on it.
        origins.append(T[:3, 3].copy())
        axes.append(T[:3, :3] @ JOINT_AXES[i])
        T = T @ _joint_rotation(i, q[i])
    T = T @ T_TOOL
    return T[:3, 3].copy(), origins, axes


def jacobian(q):
    """
    3x5 position Jacobian of the tool tip, in the base frame (frame_0).

    For a revolute joint i with unit axis k_i and origin o_i:

        J_i = k_i x (p_ee - o_i)

    i.e. the classic  Z_(i-1) x (P_ee - P_(i-1))  rule, with the arm's actual
    joint axes in place of the DH z-axes. Only the translational rows are
    built - the angular rows are intentionally omitted.

    Returns:
        J : 3x5 matrix mapping qdot [rad/s] to tool velocity [m/s]
    """
    p_ee, origins, axes = chain_frames(q)
    J = np.zeros((3, 5))
    for i in range(5):
        J[:, i] = np.cross(axes[i], p_ee - origins[i])
    return J


def forward_velocity_kinematics(q, qdot):
    """
    Forward velocity kinematics (translation only):   v = J(q) * qdot

    Parameters:
        q    : 5 joint angles     [rad]
        qdot : 5 joint velocities [rad/s]

    Returns:
        v : [Vx, Vy, Vz] of the tool tip, in m/s, expressed in frame_0
        J : the 3x5 Jacobian used
    """
    qdot = np.asarray(qdot, dtype=float).reshape(-1)[:5]
    J = jacobian(q)
    return J @ qdot, J


# ==============================================================================
# FORWARD ACCELERATION KINEMATICS  (JACOBIAN DERIVATIVE)
# ==============================================================================

def jacobian_dot_fd(q, qdot, step=1e-5):
    """
    Jdot by the central finite difference requested:

        Jdot  ~=  ( J(q + h*qdot) - J(q - h*qdot) ) / (2h)

    This is valid because J depends on time only through q, so by the chain
    rule dJ/dt = (dJ/dq) . qdot, which is exactly the directional derivative
    of J along qdot. Evaluating it costs two extra chain sweeps.

    The step is scaled as h = step / ||qdot||, so the actual perturbation
    h*qdot always has length `step` regardless of how fast the arm is moving.
    A fixed h would perturb by almost nothing when the arm is nearly stopped
    (the difference would be pure round-off) and by a large angle when it is
    fast (truncation error). When the arm is not moving at all, Jdot*qdot is
    zero anyway, so Jdot is returned as zeros.

    Parameters:
        q    : 5 joint angles     [rad]
        qdot : 5 joint velocities [rad/s]
        step : length of the perturbation along qdot [rad]. 1e-5 measured as
               the best truncation/round-off tradeoff for this chain.

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


def jacobian_dot_analytic(q, qdot):
    """
    Closed-form Jdot, kept as the reference the finite difference is checked
    against (selftest compares the two).

        omega_i  = sum_{j<i} k_j qdot_j        kdot_i = omega_i x k_i
        odot_i   = sum_{j<i} qdot_j (k_j x (o_i - o_j))
        pdot_ee  = J qdot
        Jdot_i   = kdot_i x (p_ee - o_i) + k_i x (pdot_ee - odot_i)
    """
    qdot = np.asarray(qdot, dtype=float).reshape(-1)[:5]
    p_ee, origins, axes = chain_frames(q)

    J = np.zeros((3, 5))
    for i in range(5):
        J[:, i] = np.cross(axes[i], p_ee - origins[i])

    axes_dot, omega = [], np.zeros(3)
    for i in range(5):
        axes_dot.append(np.cross(omega, axes[i]))
        omega = omega + axes[i] * qdot[i]

    origins_dot = []
    for i in range(5):
        od = np.zeros(3)
        for j in range(i):
            od = od + qdot[j] * np.cross(axes[j], origins[i] - origins[j])
        origins_dot.append(od)
    p_ee_dot = J @ qdot

    Jdot = np.zeros((3, 5))
    for i in range(5):
        Jdot[:, i] = (np.cross(axes_dot[i], p_ee - origins[i])
                      + np.cross(axes[i], p_ee_dot - origins_dot[i]))
    return Jdot


def jacobian_dot(q, qdot, method='fd', step=1e-5):
    """Jdot, by finite difference (default) or in closed form."""
    if method == 'analytic':
        return jacobian_dot_analytic(q, qdot)
    return jacobian_dot_fd(q, qdot, step=step)


def forward_acceleration_kinematics(q, qdot, qddot, method='fd', step=1e-5):
    """
    Forward acceleration kinematics (translation only):

        a = J(q) * qddot  +  Jdot(q, qdot) * qdot

    The second term is the bias (centripetal / Coriolis) acceleration: what the
    tool tip accelerates with even when every joint runs at constant speed.

    Parameters:
        q     : 5 joint angles        [rad]
        qdot  : 5 joint velocities    [rad/s]
        qddot : 5 joint accelerations [rad/s^2]

    Returns:
        a    : [Ax, Ay, Az] of the tool tip, in m/s^2, in frame_0
        bias : the 3-vector Jdot*qdot on its own
        J    : 3x5 Jacobian
        Jdot : 3x5 Jacobian derivative
    """
    qdot = np.asarray(qdot, dtype=float).reshape(-1)[:5]
    qddot = np.asarray(qddot, dtype=float).reshape(-1)[:5]

    J = jacobian(q)
    Jdot = jacobian_dot(q, qdot, method=method, step=step)
    bias = Jdot @ qdot
    return J @ qddot + bias, bias, J, Jdot


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

        # 'fd' = finite-difference Jdot (default), 'analytic' = closed form
        self.declare_parameter('jacobian_dot_method', 'fd')
        self.declare_parameter('fd_step', 1e-5)
        # JointState carries no acceleration field, and may carry no velocity
        # field either: differentiate from message stamps when it does not.
        self.declare_parameter('estimate_velocity', True)
        self.declare_parameter('estimate_acceleration', True)
        self.jdot_method = self.get_parameter('jacobian_dot_method').value
        self.fd_step = float(self.get_parameter('fd_step').value)
        self.estimate_velocity = self.get_parameter('estimate_velocity').value
        self.estimate_acceleration = self.get_parameter('estimate_acceleration').value

        # Subscriptions: reads joint positions from simulation or GUI commands
        self.create_subscription(JointState, '/joint_state', self.joint_state_cb, 10)
        self.create_subscription(Float64MultiArray, '/joint_commands', self.joint_cmd_cb, 10)
        # Optional dedicated joint-rate / joint-acceleration streams
        self.create_subscription(Float64MultiArray, '/joint_velocity_commands',
                                 self.joint_vel_cmd_cb, 10)
        self.create_subscription(Float64MultiArray, '/joint_acceleration_commands',
                                 self.joint_acc_cmd_cb, 10)

        # Publishers: publishes the reached (X, Y, Z) pose ...
        self.pose_pub = self.create_publisher(PoseStamped, '/current_ee_pose', 10)
        self.xyz_pub = self.create_publisher(Point, '/current_ee_xyz', 10)
        # ... and the resulting linear velocity / acceleration / Jacobian
        self.vel_pub = self.create_publisher(Vector3Stamped, '/current_ee_velocity', 10)
        self.acc_pub = self.create_publisher(Vector3Stamped, '/current_ee_accel', 10)
        self.jac_pub = self.create_publisher(Float64MultiArray, '/current_jacobian', 10)

        self.last_q = None
        self.last_qdot = np.zeros(5)
        self.last_qddot = np.zeros(5)
        self.last_stamp = None   # seconds, for numeric differentiation

        self.get_logger().info(
            f"FK + Velocity + Acceleration Node ready (position-only task space, "
            f"Jdot method '{self.jdot_method}'). Waiting for joint inputs...")

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------
    def joint_state_cb(self, msg: JointState):
        if len(msg.position) < 5:
            return
        q = np.array(msg.position[:5], dtype=float)
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        if len(msg.velocity) >= 5:
            qdot = np.array(msg.velocity[:5], dtype=float)
        elif self.estimate_velocity:
            qdot = self.numeric_qdot(q, stamp)
        else:
            qdot = np.zeros(5)

        qddot = self.numeric_qddot(qdot, stamp) if self.estimate_acceleration else np.zeros(5)

        self.last_stamp = stamp
        self.compute_and_report(q, qdot, qddot)

    def joint_cmd_cb(self, msg: Float64MultiArray):
        """
        Position commands. With 10 values, entries 5..9 are the commanded joint
        velocities; with 15, entries 10..14 are the commanded accelerations.
        """
        if len(msg.data) < 5:
            return
        q = np.array(msg.data[:5], dtype=float)
        qdot = np.array(msg.data[5:10], dtype=float) if len(msg.data) >= 10 else self.last_qdot
        qddot = np.array(msg.data[10:15], dtype=float) if len(msg.data) >= 15 else np.zeros(5)
        self.compute_and_report(q, qdot, qddot)

    def joint_vel_cmd_cb(self, msg: Float64MultiArray):
        """Joint rates evaluated at the most recent known posture."""
        if len(msg.data) < 5:
            return
        q = self.last_q if self.last_q is not None else np.zeros(5)
        self.compute_and_report(q, np.array(msg.data[:5], dtype=float), np.zeros(5))

    def joint_acc_cmd_cb(self, msg: Float64MultiArray):
        """Joint accelerations, at the latest posture and joint rates."""
        if len(msg.data) < 5:
            return
        q = self.last_q if self.last_q is not None else np.zeros(5)
        self.compute_and_report(q, self.last_qdot, np.array(msg.data[:5], dtype=float))

    def numeric_qdot(self, q, stamp):
        if self.last_q is None or self.last_stamp is None:
            return np.zeros(5)
        dt = stamp - self.last_stamp
        if dt <= 1e-6 or dt > 1.0:        # ignore stale / duplicate stamps
            return self.last_qdot
        return (q - self.last_q) / dt

    def numeric_qddot(self, qdot, stamp):
        if self.last_stamp is None:
            return np.zeros(5)
        dt = stamp - self.last_stamp
        if dt <= 1e-6 or dt > 1.0:
            return self.last_qddot
        return (qdot - self.last_qdot) / dt

    # ------------------------------------------------------------------
    # Core computation
    # ------------------------------------------------------------------
    def compute_and_report(self, q, qdot=None, qddot=None):
        qdot = np.zeros(5) if qdot is None else np.asarray(qdot, dtype=float)
        qddot = np.zeros(5) if qddot is None else np.asarray(qddot, dtype=float)

        # Ignore computation if angles, rates and accelerations are unchanged
        if (self.last_q is not None
                and np.allclose(self.last_q, q, atol=1e-4)
                and np.allclose(self.last_qdot, qdot, atol=1e-4)
                and np.allclose(self.last_qddot, qddot, atol=1e-4)):
            return
        self.last_q = np.asarray(q, dtype=float).copy()
        self.last_qdot = qdot.copy()
        self.last_qddot = qddot.copy()

        # ----------------------------------------------------
        # 1. FK -> (X, Y, Z),  then velocity and acceleration
        # ----------------------------------------------------
        x, y, z, T = forward_kinematics(q)
        a, bias, J, _ = forward_acceleration_kinematics(
            q, qdot, qddot, method=self.jdot_method, step=self.fd_step)
        v = J @ qdot

        # ----------------------------------------------------
        # 2. PRINT REACHED X, Y, Z AND ITS DERIVATIVES
        # ----------------------------------------------------
        self.get_logger().info(
            f"[FK Reached Position] -> X: {x:+.4f} m | Y: {y:+.4f} m | Z: {z:+.4f} m"
        )
        self.get_logger().info(
            f"[FK Linear Velocity ] -> Vx: {v[0]:+.4f} | Vy: {v[1]:+.4f} | "
            f"Vz: {v[2]:+.4f} m/s  (|v| = {np.linalg.norm(v):.4f} m/s)"
        )
        self.get_logger().info(
            f"[FK Linear Accel    ] -> Ax: {a[0]:+.4f} | Ay: {a[1]:+.4f} | "
            f"Az: {a[2]:+.4f} m/s^2  (|a| = {np.linalg.norm(a):.4f} m/s^2)"
        )
        self.get_logger().info(
            f"[Bias (Jdot*qdot)   ] -> {np.linalg.norm(bias):.4f} m/s^2"
        )

        # Warn when the posture is close to a velocity-level singularity
        sigma_min = float(np.linalg.svd(J, compute_uv=False)[-1])
        if sigma_min < 1e-3:
            self.get_logger().warn(
                f"Near-singular configuration (smallest singular value = {sigma_min:.2e})"
            )

        # ----------------------------------------------------
        # 3. PUBLISH FOR GUI & OTHER NODES
        # ----------------------------------------------------
        now = self.get_clock().now().to_msg()

        # Publish pure (X, Y, Z) point
        pt_msg = Point()
        pt_msg.x = x
        pt_msg.y = y
        pt_msg.z = z
        self.xyz_pub.publish(pt_msg)

        # Publish full PoseStamped
        quat = rot_to_quat(T[:3, :3])
        pose_msg = PoseStamped()
        pose_msg.header.stamp = now
        pose_msg.header.frame_id = "frame_0"
        pose_msg.pose.position.x = x
        pose_msg.pose.position.y = y
        pose_msg.pose.position.z = z
        pose_msg.pose.orientation.x = float(quat[0])
        pose_msg.pose.orientation.y = float(quat[1])
        pose_msg.pose.orientation.z = float(quat[2])
        pose_msg.pose.orientation.w = float(quat[3])
        self.pose_pub.publish(pose_msg)

        # Publish linear velocity and linear acceleration
        self.vel_pub.publish(self._vec_msg(v, now))
        self.acc_pub.publish(self._vec_msg(a, now))

        # Publish the 3x5 Jacobian (row-major) for controllers / IK nodes
        jac_msg = Float64MultiArray()
        jac_msg.layout.dim = [MultiArrayDimension(label="rows", size=3, stride=15),
                              MultiArrayDimension(label="cols", size=5, stride=5)]
        jac_msg.data = J.flatten().tolist()
        self.jac_pub.publish(jac_msg)

    @staticmethod
    def _vec_msg(vec, stamp):
        msg = Vector3Stamped()
        msg.header.stamp = stamp
        msg.header.frame_id = "frame_0"
        msg.vector.x = float(vec[0])
        msg.vector.y = float(vec[1])
        msg.vector.z = float(vec[2])
        return msg


# ==============================================================================
# SELF-TEST
# ==============================================================================

def selftest(n_samples=200, seed=0):
    """
    Checks, over random configurations:
      * chain_frames() reproduces forward_kinematics() exactly
      * J against a central finite difference of the FK position
      * the finite-difference Jdot against the closed-form one
      * the acceleration a = J qddot + Jdot qdot against d(v)/dt
    Run with:  python3 forward_kinematics_node.py --selftest
    """
    rng = np.random.default_rng(seed)
    e_fk = e_jac = e_jdot = e_acc = 0.0
    h = 1e-6

    for _ in range(n_samples):
        q = rng.uniform(-np.pi, np.pi, 5)
        qdot = rng.uniform(-2.0, 2.0, 5)
        qddot = rng.uniform(-5.0, 5.0, 5)

        # chain_frames vs the original forward_kinematics
        x, y, z, _ = forward_kinematics(q)
        p_ee, _, _ = chain_frames(q)
        e_fk = max(e_fk, np.abs(p_ee - np.array([x, y, z])).max())

        # J vs d(FK)/dq
        J = jacobian(q)
        J_fd = np.zeros((3, 5))
        for i in range(5):
            dq = np.zeros(5)
            dq[i] = h
            J_fd[:, i] = (np.array(forward_kinematics(q + dq)[:3])
                          - np.array(forward_kinematics(q - dq)[:3])) / (2 * h)
        e_jac = max(e_jac, np.abs(J - J_fd).max())

        # finite-difference Jdot vs closed form
        e_jdot = max(e_jdot, np.abs(jacobian_dot_fd(q, qdot)
                                    - jacobian_dot_analytic(q, qdot)).max())

        # acceleration vs d(v)/dt along q(t) = q + t qdot + t^2/2 qddot
        a, _, _, _ = forward_acceleration_kinematics(q, qdot, qddot)
        vp = jacobian(q + h*qdot + 0.5*h*h*qddot) @ (qdot + h*qddot)
        vm = jacobian(q - h*qdot + 0.5*h*h*qddot) @ (qdot - h*qddot)
        e_acc = max(e_acc, np.abs(a - (vp - vm) / (2*h)).max())

    print(f"max chain_frames vs forward_kinematics : {e_fk:.3e} m")
    print(f"max Jacobian error  (vs d(FK)/dq)      : {e_jac:.3e} m/rad")
    print(f"max Jdot error      (fd vs analytic)   : {e_jdot:.3e} 1/s")
    print(f"max acceleration error (vs dv/dt)      : {e_acc:.3e} m/s^2")
    ok = e_fk < 1e-12 and e_jac < 1e-6 and e_jdot < 1e-4 and e_acc < 1e-4
    print("SELF-TEST PASSED" if ok else "SELF-TEST FAILED")
    return ok


def main(args=None):
    if args is None and '--selftest' in sys.argv:
        sys.exit(0 if selftest() else 1)

    rclpy.init(args=args)
    node = ForwardKinematicsNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()