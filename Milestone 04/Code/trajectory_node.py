#!/usr/bin/env python3
"""
Trajectory planning + closed-loop control for the Low Cost Robot Arm.

A PATH is only the points the tool passes through; a TRAJECTORY adds the time
at which each point is reached. Two planners, one output (/joint_commands):

  JOINT-SPACE  q_start -> q_end over T
      planner    quintic polynomial per joint  ->  q_des(t), q_dot_des(t)
      controller 5 SISO fuzzy logic controllers, one per joint
      loop       q_e = q_des - q_act
                 q_dot_cmd = q_dot_des + FLC(q_e, q_e_dot)
                 q_cmd     = integral of q_dot_cmd

  TASK-SPACE   X_start -> X_end over T, straight line
      planner    X_des(t) = X_start + s(t) (X_end - X_start),  s(t) quintic
      controller MIMO fuzzy logic controller on the Cartesian error
      loop       X_e = X_des - f(q_act)                (forward kinematics)
                 X_dot_cmd = X_dot_des + FLC(X_e, X_e_dot)
                 q_dot_cmd = J# X_dot_cmd + null space  (inverse velocity kin.)
                 q_cmd     = integral of q_dot_cmd

In both loops the controller output is a VELOCITY added to the planned
(feed-forward) velocity, and the sum is integrated into the position command
the arm's servos follow. The integration also gives integral action, which
removes the gravity sag in --dynamics mode. After T the reference holds at
the goal while the error settles.

Feedback is the MEASURED state from /joint_state; the task-space error uses
the forward kinematics of the measured joints, never of the last command.

Run the bridge with --dynamics to see the controllers work. In kinematic mode
the arm lands exactly on each command, the error stays near zero and the plan
is followed by the feed-forward alone - useful for checking the plan.

Commands arrive on /trajectory_command as JSON:
    {"mode": "joint",   "q_end": [0.3, -0.5, 0.4, 0.2, 0.0], "T": 3.0}
    {"mode": "task",    "x_end": [-0.18, 0.0, 0.07],          "T": 3.0}
    {"mode": "home",    "T": 3.0}
    {"mode": "gripper", "value": -0.6}
    {"mode": "stop"}
Status is published on /trajectory/status (JSON) for the GUI and sequencer.
Every executed trajectory is logged to traj_<mode>_<time>.csv.
"""

import json
import os
import sys
import time
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String
from geometry_msgs.msg import Vector3Stamped

sys.path.insert(0, str(Path(__file__).resolve().parent))
from kinematics import (                                    # noqa: E402
    N_JOINTS, Q_MIN, Q_MAX, QDOT_MAX, ee_position, jacobian, damped_pinv,
    null_space_projector, joint_limit_gradient, inverse_kinematics)

# ==============================================================================
# CONFIGURATION
# ==============================================================================

CONTROL_HZ = 200.0
DT = 1.0 / CONTROL_HZ          # sampling time Ts. The nominal value is used for
                               # integration, not wall-clock: timer jitter would
                               # otherwise feed straight into q_cmd.

# Gripper joint: 0 is CLOSED (pads 7 mm apart), more negative opens it.
# -0.6 opens the pads to 39 mm, enough to straddle a 20 mm board.
GRIP_OPEN = -0.6
GRIP_CLOSED = 0.0

# How far the integrated command may lead the measured joint before it is
# held. Without this, a blocked joint lets q_cmd run away and the stored
# error discharges as a lurch when the obstruction clears.
MAX_LAG = 0.08                 # rad
# A position servo (kp = 100, kv = 10 in low_cost_robot_arm.xml) needs its
# target to lead the joint by kv/kp * q_dot just to move at q_dot, so that
# much extra lead is allowed on top of MAX_LAG.
SERVO_LEAD = 0.1               # s

LIMIT_EPS = 1e-3               # rad; margin for "at a joint limit"
MIN_TOOL_Z = 0.02              # m; warn when a joint-space plan dips the tool below this

# After T the reference holds at the goal and the loop keeps running until the
# error is below tolerance, so the reported final error is the settled one.
SETTLE_MAX = 1.5               # s
SETTLE_TOL_Q = 2e-3            # rad
SETTLE_TOL_X = 1e-3            # m

HOME_Q = np.zeros(N_JOINTS)

# ==============================================================================
# TRAJECTORY PLANNERS
# ==============================================================================
# Quintic (5th-order) polynomial between two rest positions:
#
#     q(t) = C0 + C1 t + C2 t^2 + C3 t^3 + C4 t^4 + C5 t^5
#
# Six coefficients need six boundary conditions; the arm starts and stops at
# rest with zero acceleration:
#
#     q(0) = q0      q_dot(0) = 0      q_ddot(0) = 0
#     q(T) = qf      q_dot(T) = 0      q_ddot(T) = 0
#
# The three conditions at t = 0 give C0 = q0, C1 = 0, C2 = 0 directly. The
# three at t = T, with dq = qf - q0, leave three equations in C3, C4, C5:
#
#     |   T^3     T^4     T^5 | |C3|   | dq |
#     |  3T^2    4T^3    5T^4 | |C4| = |  0 |
#     |  6T     12T^2   20T^3 | |C5|   |  0 |
#
#  =>  C3 = 10 dq / T^3,   C4 = -15 dq / T^4,   C5 = 6 dq / T^5
#
# A cubic can also start and stop at rest, but its acceleration jumps from
# zero to a finite value at t = 0 and t = T. The quintic's acceleration is
# zero at both ends, so the motors see no step in torque.

def quintic_coefficients(q0, qf, T):
    """C0..C5 of the rest-to-rest quintic (works element-wise on arrays)."""
    q0 = np.asarray(q0, dtype=float)
    dq = np.asarray(qf, dtype=float) - q0
    zero = np.zeros_like(q0)
    return (q0, zero, zero,
            10.0 * dq / T ** 3,
            -15.0 * dq / T ** 4,
            6.0 * dq / T ** 5)


def plan_joint_space_trajectory(q_start, q_end, T, dt):
    """
    JOINT-SPACE planning: every joint follows its own quintic from q_start
    to q_end, all finishing together at T. Smooth in the joints; the tool
    path in Cartesian space is whatever curve those joint motions produce.

    Returns (t, q, q_dot, q_ddot): t is (N,), the others (N, 5).
    """
    t = np.arange(0.0, T + dt / 2, dt)[:, None]        # sample times, column
    C0, C1, C2, C3, C4, C5 = quintic_coefficients(q_start, q_end, T)

    q = C0 + C1 * t + C2 * t**2 + C3 * t**3 + C4 * t**4 + C5 * t**5
    q_dot = C1 + 2 * C2 * t + 3 * C3 * t**2 + 4 * C4 * t**3 + 5 * C5 * t**4
    q_ddot = 2 * C2 + 6 * C3 * t + 12 * C4 * t**2 + 20 * C5 * t**3
    return t[:, 0], q, q_dot, q_ddot


def plan_task_space_trajectory(x_start, x_end, T, dt):
    """
    TASK-SPACE planning: the tool tip moves on the straight line between the
    two points. Each coordinate is

        X(t) = X_start + s(t) (X_end - X_start)

    where s(t) is the quintic above from 0 to 1 (written with tau = t/T):

        s     = 10 tau^3 - 15 tau^4 + 6 tau^5
        s_dot = (30 tau^2 - 60 tau^3 + 30 tau^4) / T
        s_ddot= (60 tau - 180 tau^2 + 120 tau^3) / T^2

    Because all three coordinates share the same s(t), every sample lies on
    the line; s(t) only sets how fast the tool moves along it (from rest, to
    rest). The matching joint values come from the inverse kinematics, done
    on-line by the controller through q_dot = J# X_dot.

    Returns (t, X, X_dot, X_ddot): t is (N,), the others (N, 3).
    """
    t = np.arange(0.0, T + dt / 2, dt)
    tau = np.clip(t / T, 0.0, 1.0)[:, None]
    s = 10 * tau**3 - 15 * tau**4 + 6 * tau**5
    s_dot = (30 * tau**2 - 60 * tau**3 + 30 * tau**4) / T
    s_ddot = (60 * tau - 180 * tau**2 + 120 * tau**3) / T**2

    d = np.asarray(x_end, dtype=float) - np.asarray(x_start, dtype=float)
    return t, x_start + s * d, s_dot * d, s_ddot * d


# ==============================================================================
# FUZZY LOGIC CONTROLLER
# ==============================================================================

class FuzzyLogicController:
    """
    Fuzzy logic controller for ONE axis (one joint, or one Cartesian axis).

        inputs : e      position error on this axis
                 e_dot  its rate
        output : u      velocity correction for this axis [rad/s or m/s]

    The four stages:
      1. Fuzzification   crisp e, e_dot -> membership degrees of the five
                         linguistic sets LN, MN, S, MP, LP (triangular
                         membership functions, read off by interpolation)
      2. Rule base       25 IF-THEN rules, the table RULES below:
                         IF e is A AND e_dot is B THEN u is C
      3. Inference       firing strength of each rule:  AND -> minimum
      4. Defuzzification centre of gravity of the fired output sets, whose
                         peaks (singletons) sit at the set centres:
                             u = sum(w_r * c_r) / sum(w_r)

    Scaling factors: e and e_dot are normalised to [-1, 1] by Ke and Kde
    (the error / rate that counts as "large"), and u is scaled by Ku (the
    largest correction). Near zero the controller behaves like
    u ~ Ku (e/Ke + e_dot/Kde): Ku/Ke is a proportional gain [1/s] and
    Ku/Kde a derivative gain. Because u is integrated by the loop, there is
    integral action as well.
    """

    LABELS = ("LN", "MN", "S", "MP", "LP")     # large/medium negative, small, medium/large positive
    CENTERS = np.array([-1.0, -0.5, 0.0, 0.5, 1.0])
    HALF_WIDTH = 0.5                           # neighbouring triangles overlap by half

    # Rule base: rows = e, columns = e_dot, entries = index of the output set.
    # Along the leading diagonal error and rate cancel (u is S); the output
    # grows away from it.
    #                    e_dot:  LN  MN  S   MP  LP
    RULES = np.array([[0,  0,  0,  1,  2],    # e = LN
                      [0,  0,  1,  2,  3],    # e = MN
                      [0,  1,  2,  3,  4],    # e = S
                      [1,  2,  3,  4,  4],    # e = MP
                      [2,  3,  4,  4,  4]])   # e = LP

    def __init__(self, Ke, Kde, Ku):
        self.Ke, self.Kde, self.Ku = float(Ke), float(Kde), float(Ku)
        self._rule_outputs = self.CENTERS[self.RULES]      # c_r of every rule

    def fuzzify(self, x):
        """Degree of membership of normalised x in each of the five sets.
        The triangles cross at 0.5, so the degrees always sum to 1."""
        return np.maximum(0.0, 1.0 - np.abs(x - self.CENTERS) / self.HALF_WIDTH)

    def __call__(self, e, e_dot):
        # 1. Fuzzification (inputs normalised and saturated to [-1, 1])
        mu_e = self.fuzzify(np.clip(e / self.Ke, -1.0, 1.0))
        mu_de = self.fuzzify(np.clip(e_dot / self.Kde, -1.0, 1.0))
        # 2.+3. Rule base and inference: w_r = min(mu_e(A), mu_e_dot(B))
        w = np.minimum(mu_e[:, None], mu_de[None, :])
        total = w.sum()
        if total < 1e-12:
            return 0.0
        # 4. Defuzzification: centre of gravity, scaled to engineering units
        return self.Ku * float((w * self._rule_outputs).sum() / total)

    def describe(self):
        return f"Ke={self.Ke:.4f} Kde={self.Kde:.4f} Ku={self.Ku:.4f}"


class TaskSpaceFLC:
    """
    MIMO fuzzy controller for task space:

        inputs : X_e = [x_e, y_e, z_e]  and its rate
        outputs: X_dot correction = [x_dot, y_dot, z_dot]

    Its rule base is decoupled by axis: the rules for x_dot only look at x_e
    (and its rate), and so on. Coupling between the axes is real, but it is
    a coupling between JOINTS, and J# already resolves it - so the fuzzy
    layer does not duplicate it, and it stays at 25 rules per output instead
    of the 5^6 a fully coupled 6-input rule base would need.
    """

    def __init__(self, Ke, Kde, Ku):
        self.axes = [FuzzyLogicController(Ke, Kde, Ku) for _ in range(3)]

    def __call__(self, X_e, X_e_dot):
        return np.array([flc(X_e[k], X_e_dot[k]) for k, flc in enumerate(self.axes)])

    def describe(self):
        return self.axes[0].describe()


# ==============================================================================
# ROS 2 NODE
# ==============================================================================

class TrajectoryNode(Node):

    def __init__(self):
        super().__init__('trajectory_node')

        # ---- controller scaling factors ----
        # Joint space (one FLC per joint): P ~ Ku/Ke = 10 1/s, D ~ Ku/Kde = 0.25.
        # The D gain stays well below 1 because in kinematic mode the measured
        # rate IS the commanded rate one step later.
        self.declare_parameter('flc_joint_ke', 0.05)    # rad    : error at full scale
        self.declare_parameter('flc_joint_kde', 2.0)    # rad/s  : rate at full scale
        self.declare_parameter('flc_joint_ku', 0.5)     # rad/s  : max output
        # Task space (one FLC per Cartesian axis): P ~ 8 1/s, D ~ 0.27
        self.declare_parameter('flc_ke', 0.01)          # m      : error at full scale
        self.declare_parameter('flc_kde', 0.30)         # m/s    : rate at full scale
        self.declare_parameter('flc_ku', 0.08)          # m/s    : max output
        self.declare_parameter('k_null', 1.0)           # null-space joint-limit avoidance gain
        self.declare_parameter('log_dir', '')

        p = lambda name: float(self.get_parameter(name).value)
        self.flc_joint = [FuzzyLogicController(p('flc_joint_ke'), p('flc_joint_kde'),
                                               p('flc_joint_ku')) for _ in range(N_JOINTS)]
        self.flc_task = TaskSpaceFLC(p('flc_ke'), p('flc_kde'), p('flc_ku'))
        self.k_null = p('k_null')

        # ---- measured state ----
        self.q_act = np.zeros(N_JOINTS)
        self.qdot_act = np.zeros(N_JOINTS)
        self.have_state = False

        # ---- execution state ----
        self.mode = None            # 'joint' | 'task' | None
        self.traj = None            # planned arrays
        self.t0 = None
        self.q_cmd = np.zeros(N_JOINTS)
        self.have_cmd = False       # q_cmd holds a real command from a trajectory
        self.grip_cmd = GRIP_OPEN
        self.log_rows = []
        self.log_dir = self.get_parameter('log_dir').value or str(Path.cwd())

        # ---- interfaces ----
        self.create_subscription(JointState, '/joint_state', self.on_joint_state, 10)
        self.create_subscription(String, '/trajectory_command', self.on_command, 10)

        self.cmd_pub = self.create_publisher(Float64MultiArray, '/joint_commands', 10)
        self.status_pub = self.create_publisher(String, '/trajectory/status', 10)
        self.qref_pub = self.create_publisher(Float64MultiArray, '/traj/q_reference', 10)
        self.xref_pub = self.create_publisher(Vector3Stamped, '/traj/x_reference', 10)
        self.xdes_pub = self.create_publisher(Vector3Stamped, '/traj/x_desired', 10)
        self.xmeas_pub = self.create_publisher(Vector3Stamped, '/traj/x_measured', 10)

        self.create_timer(DT, self.control_tick)
        self.create_timer(0.2, self.publish_status)

        self.get_logger().info(
            f"Trajectory node ready at {CONTROL_HZ:.0f} Hz.\n"
            f"  Joint FLC  {self.flc_joint[0].describe()} (same on all five joints)\n"
            f"  Task  FLC  {self.flc_task.describe()} (same on all three axes)\n"
            f"  Run mujoco_bridge.py with --dynamics to see the controllers work; "
            f"in kinematic mode the error stays near zero.")

    # ------------------------------------------------------------------
    # Feedback: the measured joint state
    # ------------------------------------------------------------------
    def on_joint_state(self, msg: JointState):
        if len(msg.position) >= N_JOINTS:
            self.q_act = np.array(msg.position[:N_JOINTS], dtype=float)
            self.have_state = True
        if len(msg.velocity) >= N_JOINTS:
            self.qdot_act = np.array(msg.velocity[:N_JOINTS], dtype=float)

    # ------------------------------------------------------------------
    # Commands
    # ------------------------------------------------------------------
    def on_command(self, msg: String):
        try:
            cmd = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.get_logger().error(f"Bad /trajectory_command payload: {exc}")
            return

        mode = cmd.get('mode', '')

        if mode == 'stop':
            self.abort("stopped by request")
            return

        if mode == 'gripper':
            self.grip_cmd = float(cmd.get('value', GRIP_OPEN))
            self.get_logger().info(f"Gripper -> {self.grip_cmd:+.3f} rad")
            # Act now when idle, holding the arm where it is. (During a
            # trajectory the next control tick sends it.) Keep the last
            # trajectory command if the arm is still there: in --dynamics mode
            # it includes the lead that holds the arm up against gravity.
            if self.mode is None and self.have_state:
                if not (self.have_cmd and np.abs(self.q_cmd - self.q_act).max() < 0.05):
                    self.q_cmd = self.q_act.copy()
                self.publish_command()
            return

        if not self.have_state:
            self.get_logger().warning("No /joint_state yet - is mujoco_bridge.py running?")
            return

        T = float(cmd.get('T', 3.0))
        if T <= 0.0:
            self.get_logger().error("T must be positive")
            return

        if mode == 'home':
            self.start_joint_space(HOME_Q, T)
        elif mode == 'joint':
            q_end = np.asarray(cmd.get('q_end', HOME_Q), dtype=float).reshape(-1)
            if q_end.size != N_JOINTS:
                self.get_logger().error(f"q_end needs {N_JOINTS} values, got {q_end.size}")
                return
            self.start_joint_space(q_end, T)
        elif mode == 'task':
            x_end = np.asarray(cmd.get('x_end', [0, 0, 0]), dtype=float).reshape(-1)
            if x_end.size != 3:
                self.get_logger().error(f"x_end needs 3 values, got {x_end.size}")
                return
            self.start_task_space(x_end, T)
        else:
            self.get_logger().error(f"Unknown mode '{mode}'")

    # ------------------------------------------------------------------
    def start_joint_space(self, q_end, T):
        q_end = np.clip(q_end, Q_MIN, Q_MAX)
        q_start = self.q_act.copy()                 # start from where the arm IS

        t, q, q_dot, q_ddot = plan_joint_space_trajectory(q_start, q_end, T, DT)

        # Joint angles say nothing obvious about where the tool goes. A plan
        # through the table is invisible in kinematic mode (the arm passes
        # through it) and ends in a collision in --dynamics mode.
        z_min = min(ee_position(q[i])[2] for i in range(0, len(t), max(1, len(t) // 50)))
        if z_min < MIN_TOOL_Z:
            self.get_logger().warning(
                f"Joint-space path drives the tool tip down to z = {z_min * 1000:.0f} mm "
                f"(table at 0). In --dynamics mode the arm will stop on contact "
                f"and the trajectory will not reach its goal.")

        self.mode = 'joint'
        self.traj = {'t': t, 'q': q, 'q_dot': q_dot, 'q_ddot': q_ddot, 'T': T, 'n': len(t)}
        self.begin_execution()
        self.get_logger().info(
            f"JOINT-SPACE: {np.round(q_start, 3)} -> {np.round(q_end, 3)} "
            f"over {T:.2f}s, {len(t)} samples")

    def start_task_space(self, x_end, T):
        # The line starts at the measured tool position, so there is no step
        # between where the arm is and where the reference begins.
        x_start = ee_position(self.q_act)

        t, X, X_dot, X_ddot = plan_task_space_trajectory(x_start, x_end, T, DT)

        # Before committing, solve the inverse position kinematics along the
        # line (Newton-Raphson, each solution seeding the next) to check that
        # every point is reachable and clear of singularities. A path that is
        # straight in the plan and curved in execution is worse than one known
        # to be curved.
        worst_err, worst_sigma = 0.0, np.inf
        q_probe = self.q_act.copy()
        for i in range(0, len(t), max(1, len(t) // 25)):
            q_probe, err = inverse_kinematics(X[i], q_probe)
            worst_err = max(worst_err, err)
            worst_sigma = min(worst_sigma,
                              float(np.linalg.svd(jacobian(q_probe), compute_uv=False)[-1]))
        if worst_err > 0.005:
            self.get_logger().warning(
                f"Task-space path not fully reachable: worst IK residual "
                f"{worst_err * 1000:.1f} mm. The tool will leave the line.")
        if worst_sigma < 0.02:
            self.get_logger().warning(
                f"Task-space path passes near a singularity: sigma_min {worst_sigma:.4f}. "
                f"Damping will engage and tracking will degrade there.")

        self.mode = 'task'
        self.traj = {'t': t, 'X': X, 'X_dot': X_dot, 'X_ddot': X_ddot, 'T': T, 'n': len(t)}
        self.begin_execution()
        self.get_logger().info(
            f"TASK-SPACE: {np.round(x_start, 4)} -> {np.round(x_end, 4)} m "
            f"over {T:.2f}s, {len(t)} samples, worst IK residual "
            f"{worst_err * 1000:.2f} mm, sigma_min {worst_sigma:.4f}")

    def begin_execution(self):
        self.q_cmd = self.q_act.copy()             # anchor the integrator
        self.have_cmd = True
        self.log_rows = []
        self.t0 = time.time()

    def abort(self, reason):
        if self.mode is not None:
            self.get_logger().info(f"Trajectory {reason}.")
        self.finish()

    def finish(self):
        if self.log_rows:
            self.write_log()
            self.log_rows = []
        self.mode = None
        self.traj = None
        self.t0 = None

    # ------------------------------------------------------------------
    # Control loop (Ts = DT)
    # ------------------------------------------------------------------
    def control_tick(self):
        if self.mode is None or self.traj is None or not self.have_state:
            return

        elapsed = time.time() - self.t0
        k = min(int(round(elapsed / DT)), self.traj['n'] - 1)   # sample index; holds at the goal after T

        row = (self.tick_joint_space(k, elapsed) if self.mode == 'joint'
               else self.tick_task_space(k, elapsed))
        self.log_rows.append(row)
        self.publish_command()

        # Past the end of the plan the reference holds at the goal; finish
        # once the arm has settled there (or give up after SETTLE_MAX).
        t_over = elapsed - self.traj['t'][-1]
        if t_over >= 0.0:
            if self.mode == 'joint':
                err = float(np.abs(self.traj['q'][-1] - self.q_act).max())
                tol, unit = SETTLE_TOL_Q, 'rad'
            else:
                err = float(np.abs(self.traj['X'][-1] - ee_position(self.q_act)).max())
                tol, unit = SETTLE_TOL_X, 'm'
            if err < tol or t_over >= SETTLE_MAX:
                how = (f"settled in {t_over:.2f}s" if err < tol
                       else f"NOT settled after {SETTLE_MAX:.1f}s")
                self.get_logger().info(
                    f"Trajectory complete ({how}). Final |error| = {err:.5f} {unit}")
                self.finish()

    # ------------------------------------------------------------------
    def integrate_command(self, qdot_cmd, freeze_whole_step):
        """
        The integrator of the loop:   q_cmd(k+1) = q_cmd(k) + q_dot_cmd * Ts

          * Speed limit: q_dot_cmd is scaled uniformly, not per joint;
            clamping joints one at a time bends the path.
          * Joint limits: task space freezes the whole step if ANY joint would
            saturate, which holds the tool on its line; joint space has no
            line to keep, so it simply clips each joint.
          * Lead bound: the command may lead the measured arm by at most
            MAX_LAG plus the lead the position servo needs to move at that
            speed (kv/kp * q_dot), so a blocked joint cannot store up a lurch.
            Only components that would grow the lead further are held.

        Returns (q_dot_cmd, frozen, scale).
        """
        over = float(np.max(np.abs(qdot_cmd) / QDOT_MAX))
        scale = 1.0
        if over > 1.0:
            scale = 1.0 / over
            qdot_cmd = qdot_cmd * scale

        q_new = self.q_cmd + qdot_cmd * DT
        frozen = False
        if freeze_whole_step:
            frozen = bool(((q_new <= Q_MIN + LIMIT_EPS) | (q_new >= Q_MAX - LIMIT_EPS)).any())
            if frozen:
                qdot_cmd = np.zeros(N_JOINTS)
                q_new = self.q_cmd
        self.q_cmd = q_new

        lag = self.q_cmd - self.q_act
        allowed = MAX_LAG + SERVO_LEAD * np.abs(qdot_cmd)
        blocked = (np.abs(lag) > allowed) & (np.sign(qdot_cmd) == np.sign(lag))
        if blocked.any():
            self.q_cmd = np.where(blocked, self.q_cmd - qdot_cmd * DT, self.q_cmd)
        self.q_cmd = np.clip(self.q_cmd, Q_MIN, Q_MAX)
        return qdot_cmd, frozen, scale

    # ------------------------------------------------------------------
    def tick_joint_space(self, k, elapsed):
        """
        Joint-space loop: five SISO loops, one per joint. The error is formed
        in joint space, so the controller never sees Cartesian coordinates.

            q_e       = q_des - q_act
            q_e_dot   = q_dot_des - q_dot_act
            q_dot_cmd = q_dot_des + [FLC_1(q_e1, q_e1_dot), ..., FLC_5(q_e5, q_e5_dot)]
            q_cmd     = q_cmd + q_dot_cmd Ts
        """
        q_des = self.traj['q'][k]
        qdot_des = self.traj['q_dot'][k]

        q_e = q_des - self.q_act
        q_e_dot = qdot_des - self.qdot_act
        u = np.array([self.flc_joint[i](q_e[i], q_e_dot[i]) for i in range(N_JOINTS)])

        _, _, scale = self.integrate_command(qdot_des + u, freeze_whole_step=False)
        if scale < 1.0:
            self.get_logger().warning(f"Joint rate limit hit: command scaled to "
                                      f"{scale * 100:.0f}%", throttle_duration_sec=1.0)

        # f(q_des): the Cartesian curve the joint plan implies, for comparison
        # with f(q_act), the curve the tool actually followed.
        x_des = ee_position(q_des)
        x_act = ee_position(self.q_act)
        self.publish_references(q_ref=q_des, x_ref=None, x_des=x_des, x_meas=x_act)

        return dict(t=elapsed, mode='joint', q_ref=q_des.copy(), q_meas=self.q_act.copy(),
                    q_cmd=self.q_cmd.copy(), qdot_meas=self.qdot_act.copy(),
                    x_ref=x_des, x_des=x_des, x_meas=x_act, frozen=False)

    # ------------------------------------------------------------------
    def tick_task_space(self, k, elapsed):
        """
        Task-space loop (the kinematic loop closed through the inverse
        velocity kinematics):

            X_act     = f(q_act)                       forward position kinematics
            X_e       = X_des - X_act
            X_e_dot   = X_dot_des - J(q_act) q_dot_act
            X_dot_cmd = X_dot_des + FLC(X_e, X_e_dot)  feed-forward + feedback
            q_dot_cmd = J# X_dot_cmd + (I - J# J)(-k_null grad H)
            q_cmd     = q_cmd + q_dot_cmd Ts           (frozen at joint limits)
        """
        X_des = self.traj['X'][k]
        Xdot_des = self.traj['X_dot'][k]

        X_act = ee_position(self.q_act)
        J = jacobian(self.q_act)
        Xdot_act = J @ self.qdot_act

        X_e = X_des - X_act
        X_e_dot = Xdot_des - Xdot_act
        Xdot_cmd = Xdot_des + self.flc_task(X_e, X_e_dot)

        # Inverse velocity kinematics with adaptive damping, and the two
        # redundant DoF spent on staying away from the joint limits
        J_pinv, sigma_min, lam = damped_pinv(J)
        qdot_cmd = J_pinv @ Xdot_cmd
        if self.k_null != 0.0:
            qdot_cmd += null_space_projector(J, J_pinv) @ (
                -self.k_null * joint_limit_gradient(self.q_act))

        qdot_cmd, frozen, scale = self.integrate_command(qdot_cmd, freeze_whole_step=True)

        # f(q_cmd), for the same desired-vs-achieved comparison as joint space
        x_cmd = ee_position(self.q_cmd)
        self.publish_references(q_ref=self.q_cmd, x_ref=X_des, x_des=x_cmd, x_meas=X_act)

        if frozen:
            self.get_logger().warning("Integration frozen: joint at limit",
                                      throttle_duration_sec=1.0)
        if lam > 0.0:
            self.get_logger().warning(
                f"Damping {lam:.4f} active (sigma_min {sigma_min:.4f}); "
                f"the tool is not tracking the line exactly", throttle_duration_sec=1.0)
        if scale < 1.0:
            self.get_logger().warning(f"Joint rate limit hit: command scaled to "
                                      f"{scale * 100:.0f}%", throttle_duration_sec=1.0)

        return dict(t=elapsed, mode='task', q_ref=self.q_cmd.copy(), q_meas=self.q_act.copy(),
                    q_cmd=self.q_cmd.copy(), qdot_meas=self.qdot_act.copy(),
                    x_ref=X_des.copy(), x_des=x_cmd, x_meas=X_act, frozen=frozen)

    # ------------------------------------------------------------------
    # Output
    # ------------------------------------------------------------------
    def publish_command(self):
        msg = Float64MultiArray()
        msg.data = [float(v) for v in self.q_cmd] + [float(self.grip_cmd)]
        self.cmd_pub.publish(msg)

    def publish_references(self, q_ref, x_ref, x_des, x_meas):
        m = Float64MultiArray()
        m.data = [float(v) for v in q_ref]
        self.qref_pub.publish(m)
        stamp = self.get_clock().now().to_msg()
        if x_ref is not None:
            self.xref_pub.publish(self._vec(x_ref, stamp))
        self.xdes_pub.publish(self._vec(x_des, stamp))
        self.xmeas_pub.publish(self._vec(x_meas, stamp))

    @staticmethod
    def _vec(v, stamp):
        msg = Vector3Stamped()
        msg.header.stamp = stamp
        msg.header.frame_id = 'frame_0'
        msg.vector.x, msg.vector.y, msg.vector.z = (float(c) for c in v[:3])
        return msg

    def publish_status(self):
        running = self.mode is not None
        progress = 0.0
        if running and self.t0 is not None:
            progress = min(1.0, (time.time() - self.t0) / self.traj['T'])
        msg = String()
        msg.data = json.dumps({'running': running, 'mode': self.mode or 'idle',
                               'progress': round(progress, 3),
                               'gripper': round(float(self.grip_cmd), 4)})
        self.status_pub.publish(msg)

    # ------------------------------------------------------------------
    def write_log(self):
        """CSV of the reference, command and measurement, for the analysis plots.

        Columns: t, frozen, q_ref*, q_cmd*, q_meas*, qdot_meas*, x_ref_*, x_des_*, x_meas_*
          joint space: q_ref = q_des(t); x_ref = x_des = f(q_des); x_meas = f(q_act)
          task space : q_ref = q_cmd;   x_ref = X_des(t); x_des = f(q_cmd); x_meas = f(q_act)
        """
        path = os.path.join(self.log_dir,
                            f"traj_{self.mode}_{time.strftime('%Y%m%d_%H%M%S')}.csv")
        cols = (['t', 'frozen']
                + [f'q_ref{i}' for i in range(N_JOINTS)]
                + [f'q_cmd{i}' for i in range(N_JOINTS)]
                + [f'q_meas{i}' for i in range(N_JOINTS)]
                + [f'qdot_meas{i}' for i in range(N_JOINTS)]
                + ['x_ref_x', 'x_ref_y', 'x_ref_z']
                + ['x_des_x', 'x_des_y', 'x_des_z']
                + ['x_meas_x', 'x_meas_y', 'x_meas_z'])
        try:
            with open(path, 'w') as fh:
                fh.write(','.join(cols) + '\n')
                for r in self.log_rows:
                    vals = ([f"{r['t']:.6f}", '1' if r['frozen'] else '0']
                            + [f"{v:.6f}" for key in ('q_ref', 'q_cmd', 'q_meas', 'qdot_meas',
                                                      'x_ref', 'x_des', 'x_meas')
                               for v in r[key]])
                    fh.write(','.join(vals) + '\n')
            self.get_logger().info(f"Logged {len(self.log_rows)} samples -> {path}")
        except OSError as exc:
            self.get_logger().error(f"Could not write log: {exc}")


# ==============================================================================
# SELF-TEST (no ROS, no simulator)
# ==============================================================================

def selftest():
    """
    Checks the planners against their boundary conditions and the fuzzy
    controller for the properties its rule base must have.
    Run with:  python3 trajectory_node.py --selftest
    """
    ok = True

    # --- joint-space quintic: boundary conditions ---
    q0 = np.array([0.1, -0.3, 0.5, -0.2, 0.4])
    q1 = np.array([-0.4, 0.2, -0.6, 0.8, -0.1])
    t, q, dq, ddq = plan_joint_space_trajectory(q0, q1, 3.0, 0.005)
    e_pos = max(np.abs(q[0] - q0).max(), np.abs(q[-1] - q1).max())
    e_vel = max(np.abs(dq[0]).max(), np.abs(dq[-1]).max())
    e_acc = max(np.abs(ddq[0]).max(), np.abs(ddq[-1]).max())
    print(f"joint samples / last t : {len(t)} / {t[-1]:.6f} s  (T = 3.0)")
    print(f"joint q(0), q(T)       : {e_pos:.3e} rad")
    print(f"joint q_dot(0), q_dot(T)   : {e_vel:.3e} rad/s")
    print(f"joint q_ddot(0), q_ddot(T) : {e_acc:.3e} rad/s^2")
    ok &= e_pos < 1e-9 and e_vel < 1e-9 and e_acc < 1e-9

    # q_dot and q_ddot are the derivatives of q
    e_dq = np.abs(dq - np.gradient(q, t, axis=0))[2:-2].max()
    e_ddq = np.abs(ddq - np.gradient(dq, t, axis=0))[2:-2].max()
    print(f"q_dot vs dq/dt, q_ddot vs dq_dot/dt : {e_dq:.3e}, {e_ddq:.3e}")
    ok &= e_dq < 1e-4 and e_ddq < 1e-3

    # --- task-space line: every sample on the line, endpoints exact ---
    x0 = np.array([-0.18, 0.00, 0.07])
    x1 = np.array([-0.10, -0.13, 0.03])
    t, x, xdot, _ = plan_task_space_trajectory(x0, x1, 2.0, 0.005)
    d_hat = (x1 - x0) / np.linalg.norm(x1 - x0)
    rel = x - x0
    e_line = np.abs(rel - np.outer(rel @ d_hat, d_hat)).max()
    e_end = max(np.abs(x[0] - x0).max(), np.abs(x[-1] - x1).max())
    print(f"task off-line distance : {e_line:.3e} m")
    print(f"task endpoints         : {e_end:.3e} m")
    ok &= e_line < 1e-12 and e_end < 1e-12
    # peak speed of a quintic is 1.875 x the average speed
    peak = np.linalg.norm(xdot, axis=1).max()
    expect = 1.875 * np.linalg.norm(x1 - x0) / 2.0
    print(f"task peak speed        : {peak:.4f} vs {expect:.4f} m/s")
    ok &= abs(peak - expect) < 1e-3

    # --- fuzzy controller ---
    flc = FuzzyLogicController(0.02, 0.10, 0.08)
    z = flc(0.0, 0.0)
    print(f"FLC at origin          : {z:+.3e}")
    ok &= abs(z) < 1e-12
    # odd symmetry: the rule base is antisymmetric, so u(-e,-e_dot) = -u(e,e_dot)
    worst = max(abs(flc(e, ed) + flc(-e, -ed))
                for e in np.linspace(-0.05, 0.05, 21) for ed in np.linspace(-0.2, 0.2, 21))
    print(f"FLC odd symmetry       : {worst:.3e}")
    ok &= worst < 1e-12
    # monotone in e (at e_dot = 0) and bounded by Ku
    vals = [flc(e, 0.0) for e in np.linspace(-0.05, 0.05, 101)]
    mono = all(b >= a - 1e-12 for a, b in zip(vals, vals[1:]))
    print(f"FLC monotone in e      : {mono}")
    print(f"FLC output bound       : {max(abs(v) for v in vals):.4f} <= 0.08")
    ok &= mono and max(abs(v) for v in vals) <= 0.08 + 1e-12

    print("SELF-TEST PASSED" if ok else "SELF-TEST FAILED")
    return ok


def main():
    if '--selftest' in sys.argv:
        sys.exit(0 if selftest() else 1)

    rclpy.init()
    node = TrajectoryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.finish()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
