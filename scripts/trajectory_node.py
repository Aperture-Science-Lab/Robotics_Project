#!/usr/bin/env python3
"""
Trajectory planning + closed-loop control node for the Low Cost Robot Arm.

Two planners, one output topic:

  JOINT-SPACE   q_start -> q_end          quintic in joint space
                reference q_ref, qdot_ref, qddot_ref
                controller: 5 decoupled SISO PID loops, one per joint
                output:     q_cmd = q_ref + PID(e, edot)

  TASK-SPACE    x_start -> x_end          straight line, quintic time profile
                reference x_ref, xdot_ref, xddot_ref
                controller: 3 decoupled SISO fuzzy controllers, one per
                            Cartesian axis, (e, edot) -> velocity command
                output:     xdot_cmd -> J# -> qdot_cmd -> integrate -> q_cmd

Both publish /joint_commands (6 values: 5 arm joints + gripper), which
mujoco_bridge.py writes into data.ctrl.

IMPORTANT: run the bridge with --dynamics. In its default kinematic mode the
arm lands exactly on the command every step, so the error signal is always
zero and neither controller has anything to do.

Feedback is taken from /joint_state, i.e. from MEASURED state. Forward
kinematics for the error signal is computed from q_meas, never from the last
command.

Commands arrive on /trajectory_command as JSON, e.g.
    {"mode": "joint", "q_end": [0.3, -0.5, 0.4, 0.2, 0.0], "T": 3.0}
    {"mode": "task",  "x_end": [-0.18, 0.0, 0.07],          "T": 3.0}
    {"mode": "stop"}
    {"mode": "home",  "T": 3.0}

Status is published on /trajectory/status, also JSON, for the GUI.
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

# Reuse the kinematics already derived and self-tested in ik_node.py rather
# than duplicating the chain. Both files live in scripts/.
sys.path.insert(0, str(Path(__file__).resolve().parent))
from ik_node import (                      # noqa: E402
    forward_kinematics,
    jacobian,
    damped_pinv,
    joint_limit_gradient,
    solve_ik_numpy,
    Q_MIN,
    Q_MAX,
    QDOT_MAX,
)

# ==============================================================================
# CONFIGURATION
# ==============================================================================

CONTROL_HZ = 200.0
DT = 1.0 / CONTROL_HZ          # nominal, NOT wall-clock: timer jitter would
                               # otherwise feed straight into the integration

N_JOINTS = 5                   # arm joints; the gripper is commanded separately
GRIP_OPEN = 0.0
GRIP_CLOSED = -1.2

# How far the integrated command may lead the measured state before it is
# clamped. Without this, a blocked joint lets q_cmd run away and the stored
# error discharges as a lurch when the obstruction clears.
MAX_LAG = 0.08                 # rad

# Largest position correction the outer PID may add to q_ref. This bounds
# integral windup WITHOUT referencing q_meas in the output path: clamping q_cmd
# against the measured state couples the 200 Hz control loop to the 50 Hz
# /joint_state staircase, so the command rides a window that only moves every
# 20 ms and jumps when it does - measured at 25% of steps reversing direction,
# which is what makes the arm flicker.
CORR_MAX = 0.30                # rad

LIMIT_EPS = 1e-3               # rad; margin for "at a joint limit"

HOME_Q = np.zeros(N_JOINTS)

# ==============================================================================
# TRAJECTORY PLANNERS
# ==============================================================================
# These two are the project's own implementations, kept verbatim. Both evaluate
# the quintic on the raw sample times rather than clipping tau to [0, 1], so a
# final sample landing past T extrapolates slightly. The error is negligible
# because s, sdot and sddot all vanish at tau = 1 (s(1+eps) ~ 1 + 10 eps^3), but
# the two functions use different stop bounds (T + dt/2 vs T + dt), so for the
# same T and dt they can return arrays of different length. control_tick()
# indexes each trajectory by its own 'n', so that is handled.

def plan_joint_space_trajectory(q_start, q_end, T, dt):
    """
    Generate a trajectory from q_start to q_end over time T.

    Method is up to you. See the guide.

    q_start: numpy array of 5 joint angles (start).
    q_end:   numpy array of 5 joint angles (end).
    T:       total time of the trajectory (seconds).
    dt:      time step between samples (seconds).

    Returns:
        t_array:  1D array of times, length N.
        q_array:  (N, 5) array of joint positions.
        dq_array: (N, 5) array of joint velocities.
        ddq_array:(N, 5) array of joint accelerations.
    """
    t = np.arange(0, T + dt / 2, dt)
    delta_q = q_end - q_start
    
    c3 = 10 * delta_q / (T ** 3)
    c4 = -15 * delta_q / (T ** 4)
    c5 = 6 * delta_q / (T ** 5)
    
    #Reshape t to (N, 1) for clean broadcasting with (5,) coefficients
    t_col = t[:, np.newaxis]
    
    q_array = q_start + c3 * (t_col**3) + c4 * (t_col**4) + c5 * (t_col**5)
    dq_array = 3 * c3 * (t_col**2) + 4 * c4 * (t_col**3) + 5 * c5 * (t_col**4)
    ddq_array = 6 * c3 * t_col + 12 * c4 * (t_col**2) + 20 * c5 * (t_col**3)
    
    return t, q_array, dq_array, ddq_array

def task_space_trajectory(x_start, x_end, T, dt):
    t = np.arange(0, T + dt, dt)
    tau = t / T
    s    = 10*tau**3 - 15*tau**4 + 6*tau**5          # 0 → 1
    sd   = (30*tau**2 - 60*tau**3 + 30*tau**4) / T
    sdd  = (60*tau - 180*tau**2 + 120*tau**3) / T**2

    d = x_end - x_start
    x    = x_start + np.outer(s, d)
    xdot = np.outer(sd, d)
    xddot= np.outer(sdd, d)
    return t, x, xdot, xddot

# ==============================================================================
# CONTROLLERS
# ==============================================================================

class JointPID:
    """
    Five decoupled SISO PID loops. Each joint sees only its own error; coupling
    between joints is a disturbance each loop rejects on its own.

    Output is a POSITION correction added to q_ref, because the MuJoCo
    <position kp kv> actuators take a target angle. This is a cascade: an outer
    position loop (here) around an inner PD servo (the actuator model).
    """

    def __init__(self, kp, ki, kd, n=N_JOINTS):
        self.kp = np.broadcast_to(np.asarray(kp, dtype=float), (n,)).copy()
        self.ki = np.broadcast_to(np.asarray(ki, dtype=float), (n,)).copy()
        self.kd = np.broadcast_to(np.asarray(kd, dtype=float), (n,)).copy()
        self.integral = np.zeros(n)

    def reset(self):
        self.integral[:] = 0.0

    def __call__(self, e, edot, dt, freeze=False):
        if not freeze:
            self.integral += e * dt
        return self.kp * e + self.ki * self.integral + self.kd * edot


class FuzzyAxisController:
    """
    Zero-order Sugeno fuzzy controller for ONE Cartesian axis.

    Inputs  : e (position error on this axis), edot (its rate)
    Output  : a velocity command for this axis [m/s]

    Three of these, one per axis, instead of one MIMO controller: a 3-input
    MIMO rule base with 5 membership functions per input needs 5^3 = 125 rules
    per output surface and cannot be tuned by hand. The coupling that matters
    here is between JOINTS, and J# handles that - the fuzzy layer does not need
    to duplicate it.

    Ke, Kde, Ku are the normalisation and output scaling factors. They behave
    exactly like Kp, Kd and an output gain, so they are reported alongside the
    PID gains rather than hidden: an FLC tuned on these against an untuned PID
    is not a comparison of control laws.
    """

    LABELS = ("NB", "NS", "ZE", "PS", "PB")
    CENTERS = np.array([-1.0, -0.5, 0.0, 0.5, 1.0])
    HALF_WIDTH = 0.5

    # Rows indexed by e, columns by edot; entries index CENTERS for the output
    # singleton. The classic diagonal rule base: the leading diagonal is ZE
    # (error and its rate cancel), magnitude grows away from it.
    #            edot: NB NS ZE PS PB
    RULES = np.array([[0, 0, 0, 1, 2],     # e = NB
                      [0, 0, 1, 2, 3],     # e = NS
                      [0, 1, 2, 3, 4],     # e = ZE
                      [1, 2, 3, 4, 4],     # e = PS
                      [2, 3, 4, 4, 4]])    # e = PB

    def __init__(self, Ke, Kde, Ku):
        self.Ke, self.Kde, self.Ku = float(Ke), float(Kde), float(Ku)
        self._consequents = self.CENTERS[self.RULES]

    def _membership(self, x):
        """Triangular MFs on [-1, 1]; they partition unity, so they sum to 1."""
        return np.maximum(0.0, 1.0 - np.abs(x - self.CENTERS) / self.HALF_WIDTH)

    def __call__(self, e, edot):
        en = np.clip(e / self.Ke, -1.0, 1.0)
        edn = np.clip(edot / self.Kde, -1.0, 1.0)

        w = np.outer(self._membership(en), self._membership(edn))
        total = w.sum()
        if total < 1e-12:
            return 0.0
        return self.Ku * float((w * self._consequents).sum() / total)

    def describe(self):
        return f"Ke={self.Ke:.4f} Kde={self.Kde:.4f} Ku={self.Ku:.4f}"

# ==============================================================================
# ROS 2 NODE
# ==============================================================================

class TrajectoryNode(Node):

    def __init__(self):
        super().__init__('trajectory_node')

        # ---- parameters ----
        self.declare_parameter('kp', [2.0, 2.0, 2.0, 1.5, 1.5])
        self.declare_parameter('ki', [1.0, 1.0, 1.0, 0.5, 0.5])
        self.declare_parameter('kd', [0.05, 0.05, 0.05, 0.02, 0.02])
        self.declare_parameter('flc_ke', 0.02)      # m      : error at full scale
        self.declare_parameter('flc_kde', 0.10)     # m/s    : rate at full scale
        self.declare_parameter('flc_ku', 0.08)      # m/s    : max output
        self.declare_parameter('k_null', 1.0)
        self.declare_parameter('log_dir', '')

        kp = np.array(self.get_parameter('kp').value, dtype=float)
        ki = np.array(self.get_parameter('ki').value, dtype=float)
        kd = np.array(self.get_parameter('kd').value, dtype=float)
        self.k_null = float(self.get_parameter('k_null').value)

        self.pid = JointPID(kp, ki, kd)
        ke = float(self.get_parameter('flc_ke').value)
        kde = float(self.get_parameter('flc_kde').value)
        ku = float(self.get_parameter('flc_ku').value)
        self.flc = [FuzzyAxisController(ke, kde, ku) for _ in range(3)]

        # ---- measured state ----
        self.q_meas = np.zeros(N_JOINTS)
        self.qdot_meas = np.zeros(N_JOINTS)
        self.gripper_meas = 0.0
        self.have_state = False
        # The control loop runs faster than /joint_state arrives. Recomputing
        # the feedback term every tick lets the error grow while q_meas is
        # frozen, so the command runs ahead of the reference and snaps back when
        # fresh feedback lands - a sawtooth at the feedback rate, which the arm
        # shows as flicker (measured: 25% of command steps reversing direction
        # at 200 Hz control against 50 Hz feedback). The feedback term is
        # therefore recomputed only when the measurement changes, and held in
        # between, while q_ref keeps advancing every tick.
        self.fb_seq = 0

        # ---- execution state ----
        self.mode = None            # 'joint' | 'task' | None
        self.traj = None            # dict of planned arrays
        self.t0 = None
        self.q_cmd = np.zeros(N_JOINTS)
        self.cmd_saturated = False  # previous step hit an anti-windup clamp
        self.correction = np.zeros(N_JOINTS)   # held between feedback updates
        self._fb_seq_used = -1
        self._fb_time = None
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
            f"  PID  kp={kp} ki={ki} kd={kd}\n"
            f"  FLC  {self.flc[0].describe()} (same on all three axes)\n"
            f"  Run mujoco_bridge.py with --dynamics, or the error signal is "
            f"identically zero and neither controller does anything.")

    # ------------------------------------------------------------------
    # Feedback
    # ------------------------------------------------------------------
    def on_joint_state(self, msg: JointState):
        if len(msg.position) >= N_JOINTS:
            self.q_meas = np.array(msg.position[:N_JOINTS], dtype=float)
            self.have_state = True
            self.fb_seq += 1
        if len(msg.position) >= N_JOINTS + 1:
            self.gripper_meas = float(msg.position[N_JOINTS])
        if len(msg.velocity) >= N_JOINTS:
            self.qdot_meas = np.array(msg.velocity[:N_JOINTS], dtype=float)

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
            return

        if not self.have_state:
            self.get_logger().warn(
                "No /joint_state yet - is mujoco_bridge.py running?")
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
                self.get_logger().error(
                    f"q_end needs {N_JOINTS} values, got {q_end.size}")
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
        q_start = self.q_meas.copy()            # start from where the arm IS

        t, q, dq, ddq = plan_joint_space_trajectory(q_start, q_end, T, DT)

        self.mode = 'joint'
        self.traj = {'t': t, 'q': q, 'dq': dq, 'ddq': ddq, 'T': T, 'n': len(t)}
        self.begin_execution()

        self.get_logger().info(
            f"JOINT-SPACE: {np.round(q_start, 3)} -> {np.round(q_end, 3)} "
            f"over {T:.2f}s, {len(t)} samples")

    def start_task_space(self, x_end, T):
        # The line starts at the measured tool position, so there is no step
        # between where the arm is and where the reference begins.
        x_start = forward_kinematics(self.q_meas)

        t, x, xdot, xddot = task_space_trajectory(x_start, x_end, T, DT)

        # Check the whole line is reachable and clear of singularities BEFORE
        # committing. A path that is straight in the plan and curved in
        # execution is worse than one known to be curved.
        worst_err, worst_sigma = 0.0, np.inf
        q_probe = self.q_meas.copy()
        for i in range(0, len(t), max(1, len(t) // 25)):
            q_probe, err = solve_ik_numpy(x[i], q_probe)
            worst_err = max(worst_err, err)
            sigma = float(np.linalg.svd(jacobian(q_probe), compute_uv=False)[-1])
            worst_sigma = min(worst_sigma, sigma)

        if worst_err > 0.005:
            self.get_logger().warn(
                f"Task-space path not fully reachable: worst IK residual "
                f"{worst_err*1000:.1f} mm. The tool will leave the line.")
        if worst_sigma < 0.02:
            self.get_logger().warn(
                f"Task-space path passes near a singularity: sigma_min "
                f"{worst_sigma:.4f}. Damping will engage and tracking will "
                f"degrade there.")

        self.mode = 'task'
        self.traj = {'t': t, 'x': x, 'xdot': xdot, 'xddot': xddot,
                     'T': T, 'n': len(t)}
        self.begin_execution()

        self.get_logger().info(
            f"TASK-SPACE: {np.round(x_start, 4)} -> {np.round(x_end, 4)} m "
            f"over {T:.2f}s, {len(t)} samples, "
            f"worst IK residual {worst_err*1000:.2f} mm, "
            f"sigma_min {worst_sigma:.4f}")

    def begin_execution(self):
        self.pid.reset()
        self.cmd_saturated = False
        self.correction = np.zeros(N_JOINTS)
        self._fb_seq_used = -1
        self._fb_time = None
        self.q_cmd = self.q_meas.copy()         # anchor the integrator
        self.log_rows = []
        self.t0 = time.time()

    def abort(self, reason):
        if self.mode is not None:
            self.get_logger().info(f"Trajectory {reason}.")
        self.finish()

    def finish(self):
        if self.log_rows:
            self.write_log()
        self.mode = None
        self.traj = None
        self.t0 = None

    # ------------------------------------------------------------------
    # Control loop
    # ------------------------------------------------------------------
    def control_tick(self):
        if self.mode is None or self.traj is None or not self.have_state:
            return

        elapsed = time.time() - self.t0
        idx = int(round(elapsed / DT))
        done = idx >= self.traj['n']
        idx = min(idx, self.traj['n'] - 1)

        if self.mode == 'joint':
            row = self.tick_joint_space(idx, elapsed)
        else:
            row = self.tick_task_space(idx, elapsed)

        self.log_rows.append(row)
        self.publish_command()

        if done:
            err = np.abs(self.traj['q'][-1] - self.q_meas).max() \
                if self.mode == 'joint' \
                else np.abs(self.traj['x'][-1] - forward_kinematics(self.q_meas)).max()
            unit = 'rad' if self.mode == 'joint' else 'm'
            self.get_logger().info(
                f"Trajectory complete. Final |error| = {err:.5f} {unit}")
            self.finish()

    # ------------------------------------------------------------------
    def tick_joint_space(self, idx, elapsed):
        """
        Joint-space loop. The error signal is formed in joint space, so the
        controller never sees Cartesian coordinates.

            e     = q_ref - q_meas
            q_cmd = q_ref + PID(e, edot)
        """
        q_ref = self.traj['q'][idx]
        qdot_ref = self.traj['dq'][idx]

        # Recompute the feedback term only when the measurement is new, and
        # hold it otherwise. q_ref still advances every tick, so the command
        # stays smooth between feedback updates instead of sawtoothing.
        if self.fb_seq != self._fb_seq_used:
            now = time.time()
            dt_fb = (DT if self._fb_time is None
                     else float(np.clip(now - self._fb_time, 1e-4, 0.2)))
            self._fb_time = now
            self._fb_seq_used = self.fb_seq

            e = q_ref - self.q_meas
            edot = qdot_ref - self.qdot_meas

            # Anti-windup: freeze the integrator on a joint limit, and when the
            # previous step's correction was already saturated. This arm's
            # frictionloss (0.2 N.m) exceeds the gravity torque at most
            # postures, so a stalled joint is a real event.
            at_limit = ((self.q_cmd <= Q_MIN + LIMIT_EPS) |
                        (self.q_cmd >= Q_MAX - LIMIT_EPS))
            freeze = bool(at_limit.any() or self.cmd_saturated)

            # Bound the CORRECTION, not the command's distance from q_meas: the
            # output stays q_ref + (bounded correction), so it always tracks the
            # reference all the way to the target.
            correction = self.pid(e, edot, dt_fb, freeze=freeze)
            corr = np.clip(correction, -CORR_MAX, CORR_MAX)
            self.cmd_saturated = bool(
                not np.allclose(corr, correction, atol=1e-12))
            self.correction = corr
        else:
            e = q_ref - self.q_meas
            edot = qdot_ref - self.qdot_meas
            freeze = self.cmd_saturated

        self.q_cmd = np.clip(q_ref + self.correction, Q_MIN, Q_MAX)

        if self.cmd_saturated:
            self.get_logger().warn(
                f"PID correction saturated at {CORR_MAX} rad; the arm is not "
                f"following its reference. Integrator held.",
                throttle_duration_sec=1.0)

        # FK of the REFERENCE: the Cartesian path the joint-space plan implies,
        # for plotting against the achieved path. FK of the MEASURED state is
        # what actually happened.
        x_desired = forward_kinematics(q_ref)
        x_measured = forward_kinematics(self.q_meas)

        self.publish_references(q_ref=q_ref, x_ref=None,
                                x_des=x_desired, x_meas=x_measured)

        return dict(t=elapsed, mode='joint',
                    q_ref=q_ref.copy(), q_meas=self.q_meas.copy(),
                    q_cmd=self.q_cmd.copy(), qdot_meas=self.qdot_meas.copy(),
                    x_ref=x_desired.copy(), x_des=x_desired.copy(),
                    x_meas=x_measured.copy(), frozen=freeze)

    # ------------------------------------------------------------------
    def tick_task_space(self, idx, elapsed):
        """
        Task-space loop. Error is formed in Cartesian space from FK of the
        MEASURED joints, three fuzzy controllers turn it into a velocity
        command, and inverse velocity kinematics maps that to joint rates.

            e        = x_ref - FK(q_meas)
            xdot_cmd = xdot_ref + [FLC_x(e_x, edot_x), FLC_y(...), FLC_z(...)]
            qdot_cmd = J#(q_meas) xdot_cmd + null-space
            q_cmd    = q_cmd + qdot_cmd dt          (frozen at joint limits)
        """
        x_ref = self.traj['x'][idx]
        xdot_ref = self.traj['xdot'][idx]

        x_meas = forward_kinematics(self.q_meas)
        J = jacobian(self.q_meas)
        xdot_meas = J @ self.qdot_meas

        e = x_ref - x_meas
        edot = xdot_ref - xdot_meas

        # Three decoupled SISO fuzzy controllers, one per axis
        fb = np.array([self.flc[k](e[k], edot[k]) for k in range(3)])
        xdot_cmd = xdot_ref + fb                    # feedforward + feedback

        # Inverse velocity kinematics with adaptive damping
        J_pinv, sigma_min, lam = damped_pinv(J)
        qdot_cmd = J_pinv @ xdot_cmd

        # Spend the redundant 2 DOF on staying away from joint limits
        if self.k_null != 0.0:
            N = np.eye(N_JOINTS) - J_pinv @ J
            qdot_cmd = qdot_cmd + N @ (-self.k_null *
                                       joint_limit_gradient(self.q_meas))

        # Scale uniformly, not per joint: clamping joints one at a time bends
        # the Cartesian path, which is the one thing task-space planning buys.
        over = np.abs(qdot_cmd) / QDOT_MAX
        scale = 1.0
        if over.max() > 1.0:
            scale = 1.0 / over.max()
            qdot_cmd = qdot_cmd * scale

        # Integrate, freezing the whole step if ANY joint would saturate.
        # Freezing all of them holds the Cartesian direction; freezing one
        # would steer the tool off the line.
        q_new = self.q_cmd + qdot_cmd * DT
        frozen = bool(((q_new <= Q_MIN + LIMIT_EPS) |
                       (q_new >= Q_MAX - LIMIT_EPS)).any())
        if not frozen:
            self.q_cmd = q_new
        else:
            qdot_cmd = np.zeros(N_JOINTS)

        # Bound how far the command may lead reality, so a blocked joint
        # cannot store up a lurch. Block only the components that would push
        # the lag further past the bound - hard-clamping q_cmd to q_meas on
        # every tick instead makes the command chase the 50 Hz feedback
        # staircase, which the arm shows as flicker.
        lag = self.q_cmd - self.q_meas
        blocked = (np.abs(lag) > MAX_LAG) & (np.sign(qdot_cmd) == np.sign(lag))
        if blocked.any():
            self.q_cmd = np.where(blocked, self.q_cmd - qdot_cmd * DT, self.q_cmd)
        self.q_cmd = np.clip(self.q_cmd, Q_MIN, Q_MAX)

        # FK of the command, for the same desired-vs-achieved comparison the
        # joint-space branch produces.
        x_desired = forward_kinematics(self.q_cmd)

        self.publish_references(q_ref=self.q_cmd, x_ref=x_ref,
                                x_des=x_desired, x_meas=x_meas)

        if frozen:
            self.get_logger().warn("Integration frozen: joint at limit",
                                   throttle_duration_sec=1.0)
        if lam > 0.0:
            self.get_logger().warn(
                f"Damping {lam:.4f} active (sigma_min {sigma_min:.4f}); "
                f"the tool is not tracking the line exactly",
                throttle_duration_sec=1.0)
        if scale < 1.0:
            self.get_logger().warn(
                f"Joint rate limit hit: command scaled to {scale*100:.0f}%",
                throttle_duration_sec=1.0)

        return dict(t=elapsed, mode='task',
                    q_ref=self.q_cmd.copy(), q_meas=self.q_meas.copy(),
                    q_cmd=self.q_cmd.copy(), qdot_meas=self.qdot_meas.copy(),
                    x_ref=x_ref.copy(), x_des=x_desired.copy(),
                    x_meas=x_meas.copy(), frozen=frozen)

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
        msg.vector.x, msg.vector.y, msg.vector.z = (float(v[0]), float(v[1]),
                                                    float(v[2]))
        return msg

    def publish_status(self):
        running = self.mode is not None
        progress = 0.0
        if running and self.t0 is not None:
            progress = min(1.0, (time.time() - self.t0) / self.traj['T'])

        msg = String()
        msg.data = json.dumps({
            'running': running,
            'mode': self.mode or 'idle',
            'progress': round(progress, 3),
            'gripper': round(float(self.grip_cmd), 4),
        })
        self.status_pub.publish(msg)

    # ------------------------------------------------------------------
    def write_log(self):
        """CSV for the report plots: reference, command and measurement."""
        path = os.path.join(
            self.log_dir,
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
                            + [f"{v:.6f}" for v in r['q_ref']]
                            + [f"{v:.6f}" for v in r['q_cmd']]
                            + [f"{v:.6f}" for v in r['q_meas']]
                            + [f"{v:.6f}" for v in r['qdot_meas']]
                            + [f"{v:.6f}" for v in r['x_ref']]
                            + [f"{v:.6f}" for v in r['x_des']]
                            + [f"{v:.6f}" for v in r['x_meas']])
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
    controller for the properties a rule base must have.
    Run with:  python3 trajectory_node.py --selftest
    """
    ok = True

    # --- joint-space planner ---
    q0 = np.array([0.1, -0.3, 0.5, -0.2, 0.4])
    q1 = np.array([-0.4, 0.2, -0.6, 0.8, -0.1])
    t, q, dq, ddq = plan_joint_space_trajectory(q0, q1, 3.0, 0.005)

    e_start = np.abs(q[0] - q0).max()
    e_end = np.abs(q[-1] - q1).max()
    e_v = max(np.abs(dq[0]).max(), np.abs(dq[-1]).max())
    e_a = max(np.abs(ddq[0]).max(), np.abs(ddq[-1]).max())
    print(f"joint samples / last t : {len(t)} / {t[-1]:.6f} s  (T = 3.0)")
    print(f"joint endpoints        : {max(e_start, e_end):.3e} rad")
    print(f"joint boundary vel     : {e_v:.3e} rad/s")
    print(f"joint boundary accel   : {e_a:.3e} rad/s^2")
    # Tolerances allow for a final sample a hair past T: the planners do not
    # clip tau, and extrapolation there is O(eps^3).
    ok &= max(e_start, e_end) < 1e-9 and e_v < 1e-6 and e_a < 1e-3

    # derivatives must agree with finite differences of the position
    dq_fd = np.gradient(q, t, axis=0)
    e_dq = np.abs(dq - dq_fd)[2:-2].max()
    print(f"qdot vs d(q)/dt        : {e_dq:.3e} rad/s")
    ok &= e_dq < 1e-4

    # --- task-space planner: every sample on the line ---
    x0 = np.array([-0.18, 0.00, 0.07])
    x1 = np.array([-0.10, -0.13, 0.03])
    t, x, xdot, xddot = task_space_trajectory(x0, x1, 2.0, 0.005)
    d = x1 - x0
    d_hat = d / np.linalg.norm(d)
    # distance of each sample from the infinite line through x0 along d_hat
    rel = x - x0
    perp = rel - np.outer(rel @ d_hat, d_hat)
    e_line = np.abs(perp).max()
    print(f"task samples / last t  : {len(t)} / {t[-1]:.6f} s  (T = 2.0)")
    print(f"task max tau           : {(t/2.0).max():.9f}")
    print(f"task off-line distance : {e_line:.3e} m")
    print(f"task endpoints         : "
          f"{max(np.abs(x[0]-x0).max(), np.abs(x[-1]-x1).max()):.3e} m")
    ok &= e_line < 1e-12
    ok &= max(np.abs(x[0]-x0).max(), np.abs(x[-1]-x1).max()) < 1e-9

    # peak speed of a quintic is 1.875 * (distance / T)
    peak = np.linalg.norm(xdot, axis=1).max()
    expect = 1.875 * np.linalg.norm(d) / 2.0
    print(f"task peak speed        : {peak:.4f} vs {expect:.4f} m/s")
    ok &= abs(peak - expect) < 1e-3

    # --- fuzzy controller ---
    flc = FuzzyAxisController(0.02, 0.10, 0.08)
    z = flc(0.0, 0.0)
    print(f"FLC at origin          : {z:+.3e} m/s")
    ok &= abs(z) < 1e-12

    # odd symmetry: the rule base is antisymmetric, so f(-e,-ed) = -f(e,ed)
    worst_sym = 0.0
    for e in np.linspace(-0.05, 0.05, 21):
        for ed in np.linspace(-0.2, 0.2, 21):
            worst_sym = max(worst_sym, abs(flc(e, ed) + flc(-e, -ed)))
    print(f"FLC odd symmetry       : {worst_sym:.3e}")
    ok &= worst_sym < 1e-12

    # monotone in e at fixed edot, and bounded by Ku
    vals = [flc(e, 0.0) for e in np.linspace(-0.05, 0.05, 101)]
    mono = all(b >= a - 1e-12 for a, b in zip(vals, vals[1:]))
    print(f"FLC monotone in e      : {mono}")
    print(f"FLC output bound       : {max(abs(v) for v in vals):.4f} <= 0.08")
    ok &= mono and max(abs(v) for v in vals) <= 0.08 + 1e-12

    print("SELF-TEST PASSED" if ok else "SELF-TEST FAILED")
    return ok


def main(args=None):
    if args is None and '--selftest' in sys.argv:
        sys.exit(0 if selftest() else 1)

    rclpy.init(args=args)
    node = TrajectoryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.finish()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()