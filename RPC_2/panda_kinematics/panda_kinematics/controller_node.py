#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from scipy.optimize import least_squares
import numpy as np


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

#helper function 
def quat_to_rot(quat):
    w, x, y, z = quat / np.linalg.norm(quat)
    return np.array([
        [1 - 2*(y*y + z*z),     2*(x*y - z*w),     2*(x*z + y*w)],
        [    2*(x*y + z*w), 1 - 2*(x*x + z*z),     2*(y*z - x*w)],
        [    2*(x*z - y*w),     2*(y*z + x*w), 1 - 2*(x*x + y*y)],
    ])

def body_transform(pos, quat, theta=0.0):
    T = np.eye(4)
    T[:3, :3] = quat_to_rot(quat)
    T[:3, 3] = pos
    if theta != 0.0:
        ct, st = np.cos(theta), np.sin(theta)
        Rz = np.array([[ct, -st, 0, 0], [st, ct, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
        T = T @ Rz
    return T

def inertia_tensor(full):
    ixx, iyy, izz, ixy, ixz, iyz = full
    return np.array([
        [ixx, ixy, ixz],
        [ixy, iyy, iyz],
        [ixz, iyz, izz],
    ])

#(pos, quat, mass, com_local, fullinertia, active_joints_count)
PANDA_BODIES = [
    ([0.0, 0.0, 0.333],    [1, 0, 0, 0],  4.970684, [0.003875, 0.002081, -0.04762],   [0.70337, 0.70661, 0.009117, -0.000139, 0.006772, 0.019169], 1),
    ([0.0, 0.0, 0.0],      [1, -1, 0, 0], 0.646926, [-0.003141, -0.02872, 0.003495],  [0.007962, 0.02811, 0.025995, -0.003925, 0.010254, 0.000704], 2),
    ([0.0, -0.316, 0.0],   [1, 1, 0, 0],  3.228604, [0.027518, 0.039252, -0.066502],  [0.037242, 0.036155, 0.01083, -0.004761, -0.011396, -0.012805], 3),
    ([0.0825, 0.0, 0.0],   [1, 1, 0, 0],  3.587895, [-0.05317, 0.104419, 0.027454],   [0.025853, 0.019552, 0.028323, 0.007796, -0.001332, 0.008641], 4),
    ([-0.0825, 0.384, 0.0],[1, -1, 0, 0], 1.225946, [-0.011953, 0.041065, -0.038437], [0.035549, 0.029474, 0.008627, -0.002117, -0.004037, 0.000229], 5),
    ([0.0, 0.0, 0.0],      [1, 1, 0, 0],  1.666555, [0.060149, -0.014117, -0.010517], [0.001964, 0.004354, 0.005433, 0.000109, -0.001158, 0.000341], 6),
    ([0.088, 0.0, 0.0],    [1, 1, 0, 0],  0.735522, [0.010517, -0.004252, 0.061597],  [0.012516, 0.010027, 0.004815, -0.000428, -0.001196, -0.000741], 7),
    #fixed hand and open fingers attached to link 7 theta = 0
    ([0.0, 0.0, 0.107],    [0.9238795, 0, 0, -0.3826834], 0.73, [-0.01, 0.0, 0.03], [0.001, 0.0025, 0.0017, 0.0, 0.0, 0.0], 7),
    ([0.0, 0.04, 0.0584],  [1, 0, 0, 0],  0.015, [0.0, 0.0, 0.0], [2.375e-6, 2.375e-6, 7.5e-7, 0.0, 0.0, 0.0], 7),
    ([0.0, -0.04, 0.0584], [0, 0, 0, 1],  0.015, [0.0, 0.0, 0.0], [2.375e-6, 2.375e-6, 7.5e-7, 0.0, 0.0, 0.0], 7),
]

_PANDA_PRECOMP = [
    (body_transform(pos, quat, 0.0), mass, np.append(com, 1.0), inertia_tensor(full_I), n_joints)
    for (pos, quat, mass, com, full_I, n_joints) in PANDA_BODIES
]

def _compute_M_and_g(q, compute_g=True):
    M = np.eye(7) * 0.1
    g = np.zeros(7) if compute_g else None
    T = np.eye(4)
    origins = np.empty((7, 3))
    axes = np.empty((7, 3))

    for idx, (T_fixed, mass, com_h, I_local, n_joints) in enumerate(_PANDA_PRECOMP):
        T = T @ T_fixed
        if idx < 7:
            ct, st = np.cos(q[idx]), np.sin(q[idx])
            Rz = np.array([
                [ct, -st, 0.0, 0.0],
                [st,  ct, 0.0, 0.0],
                [0.0, 0.0, 1.0, 0.0],
                [0.0, 0.0, 0.0, 1.0],
            ])
            T = T @ Rz
            origins[idx] = T[:3, 3]
            axes[idx] = T[:3, 2]

        R = T[:3, :3]
        p_com = (T @ com_h)[:3]
        I_world = R @ I_local @ R.T

        ax = axes[:n_joints]
        Jv = np.cross(ax, p_com - origins[:n_joints]).T
        Jw = ax.T

        M[:n_joints, :n_joints] += mass * (Jv.T @ Jv) + (Jw.T @ I_world @ Jw)
        if compute_g:
            g[:n_joints] += Jv[2] * (mass * 9.81)

    return M, g

def compute_dynamics(q, dq, ddq):
    """
    q:   numpy array of 7 joint angles (rad).
    dq:  numpy array of 7 joint velocities (rad/s).
    ddq: numpy array of 7 joint accelerations (rad/s^2).
    
    Returns:
        M_matrix: 7x7 numpy array (Mass/Inertia matrix).
        C_g_vector: 7x1 numpy array (Coriolis + Gravity forces).
    """
    M_matrix, g_vec = _compute_M_and_g(q, compute_g=True)

    eps = 1e-6
    dM = np.empty((7, 7, 7))
    for k in range(7):
        q_plus = q.copy()
        q_plus[k] += eps
        dM[k] = (_compute_M_and_g(q_plus, False)[0] - M_matrix) / eps

    c_vec = np.einsum('kij,j,k->i', dM, dq, dq) - 0.5 * np.einsum('ijk,j,k->i', dM, dq, dq)
    return M_matrix, c_vec + g_vec

DERR_PEAKS = [-0.5, -0.25, 0.0, 0.25, 0.5]
TAU_PEAKS  = [-10.0, -5.0, 0.0, 5.0, 10.0]

ERR_PEAKS     = [-0.1, -0.05, 0.0, 0.05, 0.1]

#rows: error (LN, MN, Z, MP, LP) amd cols: delta_error (LN, MN, Z, MP, LP) twin i am cooked
RULE_TABLE = [
    [0, 0, 0, 1, 2],
    [0, 0, 1, 2, 3],
    [0, 1, 2, 3, 4],
    [1, 2, 3, 4, 4],
    [2, 3, 4, 4, 4],
]

def _fuzzify(x, peaks):
    mu = np.zeros(5)
    if x <= peaks[0]:
        mu[0] = 1.0
    elif x < peaks[1]:
        mu[0] = (peaks[1] - x) / (peaks[1] - peaks[0])

    for i in range(1, 4):
        if peaks[i - 1] < x <= peaks[i]:
            mu[i] = (x - peaks[i - 1]) / (peaks[i] - peaks[i - 1])
        elif peaks[i] < x < peaks[i + 1]:
            mu[i] = (peaks[i + 1] - x) / (peaks[i + 1] - peaks[i])

    if x >= peaks[4]:
        mu[4] = 1.0
    elif x > peaks[3]:
        mu[4] = (x - peaks[3]) / (peaks[4] - peaks[3])
    return mu

U_GRID = np.linspace(-10.0, 10.0, 201)
OUT_MFS = np.array([_fuzzify(u, TAU_PEAKS) for u in U_GRID]).T  # shape (5, 201)

#this ts helps in the robot not dancing 
JOINT_MAX_TAU = np.array([25.0, 25.0, 20.0, 20.0, 6.0, 6.0, 4.0])

def fuzzy_controller(error, delta_error, dt):
    """
    error:       7x1 numpy array (desired_q - current_q).

    delta_error: 7x1 numpy array (desired_dq - current_dq).
    dt:          Time step in seconds.
    
    Returns:
        tau_correction: 7x1 numpy array of correction torques.
    """
    tau_correction = np.zeros(7)
    for j in range(7):
        mu_e = _fuzzify(error[j], ERR_PEAKS)
        mu_de = _fuzzify(delta_error[j], DERR_PEAKS)

        #inference: AND = min, OR aggregation = max
        out_strength = np.zeros(5)
        for r in range(5):
            for c in range(5):
                w = min(mu_e[r], mu_de[c])
                idx = RULE_TABLE[r][c]
                if w > out_strength[idx]:
                    out_strength[idx] = w

        #clip output triangles and compute Centroid of Area
        shaded = np.max(np.minimum(out_strength[:, None], OUT_MFS), axis=0)
        area = np.sum(shaded)
        centroid = np.sum(U_GRID * shaded) / area if area > 1e-9 else 0.0
        tau_correction[j] = (centroid / 10.0) * JOINT_MAX_TAU[j]
    
    return tau_correction

_log_step = 0
_log_file = None

def _log_sample(q, q_des):
    global _log_step, _log_file
    if _log_file is None:
        _log_file = open("traj_log.csv", "w")
        cols = ["t"] + [f"q{i}" for i in range(7)] + [f"qdes{i}" for i in range(7)]
        _log_file.write(",".join(cols) + "\n")
    row = [_log_step * 0.01] + q.tolist() + q_des.tolist()
    _log_file.write(",".join(f"{v:.6f}" for v in row) + "\n")
    _log_file.flush()
    _log_step += 1


def compute_torques(q, dq, q_des, dq_des, ddq_des):
    """
    q:       current joint positions (7x1).
    dq:      current joint velocities (7x1).
    q_des:   desired joint positions (7x1).
    dq_des:  desired joint velocities (7x1).
    ddq_des: desired joint accelerations (7x1).
    
    Returns:
        final_torques: 7x1 numpy array to send to the motors.
    """
    M, Cg = compute_dynamics(q, dq, ddq_des)
    tau_model = np.dot(M, ddq_des) + Cg
    
    # 2. Calculate the error for the FLC
    error = q_des - q
    delta_error = dq_des - dq  # (Can also be (error - prev_error) / dt)
    
    # 3. Get the correction torque from your FLC
    tau_correction = fuzzy_controller(error, delta_error, 0.01) # 0.01 is dt
    
    # 4. Combine them (You may need to tune how they are added)
    final_torques = tau_model + tau_correction
    _log_sample(q, q_des)
    return final_torques



# ==========================================================
# DO NOT MODIFY BELOW
# ==========================================================

CMD_SIZE = 9
GRIPPER_CTRL = 255.0
TRAJ_DURATION = 4.0
TRAJ_DT = 0.01

# Team-specific start and end configurations.
Q_START = np.array([0.0, 0.0, 0.0, -0.1, 0.0, 0.0, 0.0])
T_start = forward_kinematics(Q_START)

T_end = T_start.copy()
T_end[0, 3] += 0.35   #reach 35 cm forward in X
T_end[2, 3] -= 0.45   #lower 45 cm in Z into a horizontal arm pose
Q_END, _ = inverse_kinematics(T_end, Q_START)


class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')
        self.q_current = np.zeros(7)
        self.dq_current = np.zeros(7)
        
        self.q_traj = None
        self.dq_traj = None
        self.ddq_traj = None
        self.step_idx = 0

        self.create_subscription(JointState, 'joint_state', self.on_joint_state, 10)
        self.cmd_pub = self.create_publisher(Float64MultiArray, 'joint_commands', 10)

        self.init_timer = self.create_timer(0.5, self.start_trajectory_once)

    def on_joint_state(self, msg):
        self.q_current = np.array(msg.position[:7])
        if hasattr(msg, 'velocity') and len(msg.velocity) >= 7:
            self.dq_current = np.array(msg.velocity[:7])

    def start_trajectory_once(self):
        if self.q_traj is not None:
            return

        result = plan_trajectory(Q_START, Q_END, TRAJ_DURATION, TRAJ_DT)
        if result is None:
            return

        _, self.q_traj, self.dq_traj, self.ddq_traj = result
        self.get_logger().info("Trajectory generated. Starting Controller.")
        
        # Switch to the control loop timer
        #self.destroy_timer(self.timers[0])
        self.destroy_timer(self.init_timer)
        self.create_timer(TRAJ_DT, self.control_loop)

    def control_loop(self):
        if self.step_idx >= len(self.q_traj):
            # Hold final position by sending zero velocity and holding torque
            cmd = Float64MultiArray()
            M, Cg = compute_dynamics(self.q_current, np.zeros(7), np.zeros(7))
            cmd.data = Cg.tolist() + [GRIPPER_CTRL, 0.0]
            self.cmd_pub.publish(cmd)
            return

        q_des = self.q_traj[self.step_idx]
        dq_des = self.dq_traj[self.step_idx]
        ddq_des = self.ddq_traj[self.step_idx]

        torques = compute_torques(
            self.q_current, self.dq_current, 
            q_des, dq_des, ddq_des
        )

        cmd = Float64MultiArray()
        cmd.data = torques.tolist() + [GRIPPER_CTRL, 0.0]
        self.cmd_pub.publish(cmd)

        self.step_idx += 1


def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()