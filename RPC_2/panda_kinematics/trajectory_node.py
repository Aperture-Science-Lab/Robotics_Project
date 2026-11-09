#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from scipy.optimize import least_squares
import numpy as np


# ==========================================================
#  EDIT ONLY THIS SECTION
# ==========================================================

# Copy from Challenge 1.
DH_PARAMS = [
    [-np.pi / 2,  0.0,     0.333],   # link 1
    [ np.pi / 2,  0.0,     0.0  ],   # link 2
    [ np.pi / 2,  0.0825,  0.316],   # link 3
    [-np.pi / 2, -0.0825,  0.0  ],   # link 4
    [ np.pi / 2,  0.0,     0.384],   # link 5
    [ np.pi / 2,  0.088,   0.0  ],   # link 6
    [ 0.0,        0.0,     0.0  ],   # link 7
]

_c, _s = np.cos(-np.pi / 4), np.sin(-np.pi / 4)
T_L7_HAND = np.array([
    [_c, -_s, 0.0, 0.0],
    [_s,  _c, 0.0, 0.0],
    [0.0, 0.0, 1.0, 0.107],
    [0.0, 0.0, 0.0, 1.0],
])

Q_MIN = np.array([-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973])
Q_MAX = np.array([ 2.8973,  1.7628,  2.8973, -0.0698,  2.8973,  3.7525,  2.8973])

TARGET_POSE = None
Q_TARGET = np.array([0.3, -0.4, 0.2, -1.8, 0.0, 1.6, 0.5])

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
TARGET_POSE[:3, 3] = [2, 0, 1]


 
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


def compute_jacobian(q):
    """
    Linear geometric Jacobian of the 'hand' frame in the base frame.

    q: numpy array of 7 joint angles (rad).

    Returns a 3x7 numpy array mapping joint velocities to
    linear end-effector velocities (x, y, z) in the base frame.
    """
    J = np.zeros((3, 7))
    
    # Store intermediate transforms; start with base frame (Identity) the thingy that has the dignal diggers
    T = np.eye(4)
    transforms = [T.copy()]
    
    # Forward kinematics pass accumulating intermediate matrices 
    for i in range(7):
        alpha, a, d = DH_PARAMS[i]
        theta = q[i]
        T_i = dh_transform(alpha, a, d, theta)
        T = T @ T_i
        transforms.append(T.copy())
        
    # Apply final hand transform to get end-effector position
    T_hand = T @ T_L7_HAND
    p_ee = T_hand[:3, 3] 
    
    # Compute each column of the linear Jacobian using the cross product
    for i in range(7):
        #transforms[i] represents the frame before joint i (T_{i-1})
        z_axis = transforms[i][:3, 2]  # Third column is the Z rotation axis
        p_joint = transforms[i][:3, 3] # Last column is the translation origin
        
        J[:, i] = np.cross(z_axis, p_ee - p_joint)
        
    return J

EPS = 1e-5

def compute_jacobian_dot(q, dq):
    """
    Time derivative of the linear geometric Jacobian.

    q:  numpy array of 7 joint angles (rad).
    dq: numpy array of 7 joint velocities (rad/s).

    Returns a 3x7 numpy array. Analytical or numerical,
    your choice.
    """
    J_plus = compute_jacobian(q + EPS * dq)
    J_minus = compute_jacobian(q - EPS * dq)
    return (J_plus - J_minus) / (2 * EPS)


def plan_trajectory(q_start, q_end, T, dt):
    """
    Generate a trajectory from q_start to q_end over time T.

    Method is up to you. See the guide.

    q_start: numpy array of 7 joint angles (start).
    q_end:   numpy array of 7 joint angles (end).
    T:       total time of the trajectory (seconds).
    dt:      time step between samples (seconds).

    Returns:
        t_array:  1D array of times, length N.
        q_array:  (N, 7) array of joint positions.
        dq_array: (N, 7) array of joint velocities.
        ddq_array:(N, 7) array of joint accelerations.
    """
    #like in the tutorial 06 so that is why i am a lazzy person so i will use it teach a man how to fish ts this is joint space my diggers
    #q(t) = c_0 + c_1 t + c_2 t^2 + c_3 t^3 + c_4 t^4 + c_5 t^5
    #dot{q}(t) = c_1 + 2c_2 t + 3c_3 t^2 + 4c_4 t^3 + 5c_5 t^4
    #ddot{q}(t) = 2c_2 + 6c_3 t + 12c_4 t^2 + 20c_5 t^3
    #since q(0) = q_start, dot{q}(0) = 0, ddot{q}(0) = 0, q(T) = q_end, dot{q}(T) = 0, ddot{q}(T) = 0
    #we can solve for the coefficients c_0, c_1, c_2 like my diggers c_0 = q_start, c_1 = 0, c_2 = 0,
    #c_3 = 10*(q_end - q_start)/T^3, c_4 = -15*(q_end - q_start)/T^4, c_5 = 6*(q_end - q_start)/T^5
    t = np.arange(0, T + dt / 2, dt)
    delta_q = q_end - q_start
    
    c3 = 10 * delta_q / (T ** 3)
    c4 = -15 * delta_q / (T ** 4)
    c5 = 6 * delta_q / (T ** 5)
    
    #Reshape t to (N, 1) for clean broadcasting with (7,) coefficients
    t_col = t[:, np.newaxis]
    
    q_array = q_start + c3 * (t_col**3) + c4 * (t_col**4) + c5 * (t_col**5)
    dq_array = 3 * c3 * (t_col**2) + 4 * c4 * (t_col**3) + 5 * c5 * (t_col**4)
    ddq_array = 6 * c3 * t_col + 12 * c4 * (t_col**2) + 20 * c5 * (t_col**3)
    
    return t, q_array, dq_array, ddq_array


# ==========================================================
# INFRASTRUCTURE — DO NOT MODIFY BELOW
# ==========================================================

CMD_SIZE = 9
GRIPPER_CTRL = 255.0

# UPDATE THESE BASED ON YOUR ASSIGNED VARIANT
# You must choose reachable configurations within joint limits.
Q_START = np.zeros(7)
Q_END   = np.zeros(7)

TRAJ_DURATION = 4.0
TRAJ_DT       = 0.05


class TrajectoryNode(Node):
    def __init__(self):
        super().__init__('trajectory_node')
        self.q_current = np.zeros(7)
        self.dq_current = np.zeros(7)

        self.create_subscription(
            JointState, 'joint_state', self.on_joint_state, 10)
        self.cmd_pub = self.create_publisher(
            Float64MultiArray, 'joint_commands', 10)

        self.t_array = None
        self.q_traj = None
        self.dq_traj = None
        self.ddq_traj = None
        self.step_idx = 0

        # Wait 0.5s before starting to ensure the sim is up
        self.create_timer(0.5, self.start_trajectory_once)

    def on_joint_state(self, msg):
        self.q_current = np.array(msg.position[:7])
        if hasattr(msg, 'velocity') and len(msg.velocity) >= 7:
            self.dq_current = np.array(msg.velocity[:7])

    def start_trajectory_once(self):
        if self.t_array is not None:
            return

        result = plan_trajectory(Q_START, Q_END, TRAJ_DURATION, TRAJ_DT)
        if result is None:
            return

        t_arr, q_arr, dq_arr, ddq_arr = result

        if q_arr.shape[1] != 7 or dq_arr.shape[1] != 7 or ddq_arr.shape[1] != 7:
            self.get_logger().error(
                "plan_trajectory returned wrong shape. Expected (N, 7).")
            return

        self.t_array = t_arr
        self.q_traj = q_arr
        self.dq_traj = dq_arr
        self.ddq_traj = ddq_arr

        self.get_logger().info(
            f"Trajectory generated: {len(t_arr)} samples over "
            f"{TRAJ_DURATION:.1f} s at {1.0/TRAJ_DT:.0f} Hz.")

        self.create_timer(TRAJ_DT, self.tick)

    def tick(self):
        if self.step_idx >= len(self.t_array):
            # Hold final target
            cmd = Float64MultiArray()
            
            # ==================================================
            # DEPENDING ON YOUR BRIDGE MODE, UNCOMMENT ONE BELOW:
            # ==================================================
            
            # -- POSITION MODE (Default) --
            cmd.data = Q_END.tolist() + [GRIPPER_CTRL, 0.0]
            
            # -- VELOCITY MODE --
            # cmd.data = [0.0] * 7 + [GRIPPER_CTRL, 0.0]
            
            self.cmd_pub.publish(cmd)
            return

        cmd = Float64MultiArray()
        
        # ==================================================
        # DEPENDING ON YOUR BRIDGE MODE, UNCOMMENT ONE BELOW:
        # ==================================================
        
        # -- POSITION MODE (Default) --
        q_target = self.q_traj[self.step_idx]
        
        # -- VELOCITY MODE --
        # q_target = self.dq_traj[self.step_idx]
        
        cmd.data = q_target.tolist() + [GRIPPER_CTRL, 0.0]
        self.cmd_pub.publish(cmd)

        self.step_idx += 1


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
