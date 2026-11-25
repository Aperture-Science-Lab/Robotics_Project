#!/usr/bin/env python3
"""
MuJoCo bridge for the Low Cost Robot Arm.

Sits between the GUI and the kinematics nodes and gives the system an actual
robot:

    /joint_commands  (Float64MultiArray, 6 values)  ->  MuJoCo
    MuJoCo                                          ->  /joint_state (JointState)

Two modes:
    kinematic (default)  commanded angles are written straight into qpos and
                         mj_forward() updates the frames. No dynamics: the arm
                         is exactly where you told it to be. Right for checking
                         FK/IK.
    dynamic (--dynamics) commands go to the actuators and mj_step() integrates
                         the arm's equations of motion. The arm lags, sags
                         under gravity and respects limits.

The plant in --dynamics mode
    Equations of motion (Lagrangian, L = T - U summed over the links):

        tau_i = d/dt (dL/dq_dot_i) - dL/dq_i
          =>    M(q) q_ddot + C(q, q_dot) q_dot + G(q) = tau - tau_friction

        M  mass / inertia matrix (+ 0.5 kg m^2 rotor armature on every joint)
        C  centripetal and Coriolis terms
        G  gravity torques
        tau_friction  dry friction, 0.2 N m per joint

    Actuators: every joint has a position servo (low_cost_robot_arm.xml,
    <position kp="100" kv="10">) that turns the commanded angle into torque

        tau = kp (q_cmd - q) - kv q_dot,     clipped to the joint's forcerange

    so /joint_commands is the servo SET-POINT, not the joint angle: under
    gravity the joint settles below it until the trajectory controller's
    integral action raises q_cmd. MuJoCo builds M, C and G from the link
    masses and inertias in the model and integrates at 2 ms steps.

Run with mjpython (macOS requires it for launch_passive):

    mjpython scripts/mujoco_bridge.py
    mjpython scripts/mujoco_bridge.py --dynamics
    python   scripts/mujoco_bridge.py --compare      # no viewer, checks the DH model
"""

import argparse
import json
import os
import sys
import time
from pathlib import Path

import numpy as np
import mujoco
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, String
from sensor_msgs.msg import JointState

# scripts/ -> project root -> model/
PROJECT_ROOT = Path(__file__).resolve().parent.parent
DEFAULT_XML = PROJECT_ROOT / "model" / "scene.xml"

# Order the GUI and fk_node assume: q[0..4] are the arm joints, q[5] the gripper.
EXPECTED_JOINTS = [
    "base_rotation",
    "pitch",
    "elbow",
    "wrist_pitch",
    "wrist_roll",
    "gripper",
]

# Matched to trajectory_node's control rate. At 50 Hz against a 200 Hz
# controller the error signal is frozen for four ticks at a time, which makes
# the command sawtooth and the arm flicker.
PUBLISH_HZ = 200.0


def resolve_joints(model):
    """
    Map the expected joint names onto this model.

    Returns (names, qpos_addresses, dof_addresses). Falls back to the model's
    own hinge/slide joints in declaration order if the names don't match, and
    reports what it found either way so a mismatch is visible rather than
    silently producing a scrambled arm.
    """
    names, qadr, vadr = [], [], []

    for name in EXPECTED_JOINTS:
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        if jid >= 0:
            names.append(name)
            qadr.append(model.jnt_qposadr[jid])
            vadr.append(model.jnt_dofadr[jid])

    if len(names) == len(EXPECTED_JOINTS):
        return names, qadr, vadr

    # Name lookup incomplete: fall back to declaration order.
    names, qadr, vadr = [], [], []
    for jid in range(model.njnt):
        if model.jnt_type[jid] in (mujoco.mjtJoint.mjJNT_HINGE,
                                   mujoco.mjtJoint.mjJNT_SLIDE):
            nm = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, jid)
            names.append(nm if nm else f"joint_{jid}")
            qadr.append(model.jnt_qposadr[jid])
            vadr.append(model.jnt_dofadr[jid])
    return names, qadr, vadr


class MujocoBridge(Node):
    def __init__(self, model, data, names, qadr, vadr, dynamics):
        super().__init__("mujoco_bridge")
        self.model, self.data = model, data
        self.names, self.qadr, self.vadr = names, qadr, vadr
        self.dynamics = dynamics
        self.n = len(names)

        self._pending = None          # latest command only; applied per frame

        # Kinematic mode teleports qpos, so MuJoCo's qvel stays 0 and the arm
        # would always report standing still. Instead report the joint rate the
        # commands actually produce: displacement between applied commands over
        # the time between them, back to 0 once commands stop arriving.
        self._kin_qvel = np.zeros(self.n)
        self._last_q = None
        self._last_apply = None

        self.create_subscription(
            Float64MultiArray, "/joint_commands", self.on_command, 1)
        self.pub = self.create_publisher(JointState, "/joint_state", 10)
        self.create_timer(1.0 / PUBLISH_HZ, self.publish_state)

        mode = "dynamic (ctrl + mj_step)" if dynamics else "kinematic (qpos + mj_forward)"
        self.get_logger().info(f"MuJoCo bridge up in {mode} mode.")
        self.get_logger().info(f"Driving joints: {', '.join(self.names)}")
        if dynamics and model.nu == 0:
            self.get_logger().warning(
                "Model declares no actuators; --dynamics will not move the arm.")

    def on_command(self, msg):
        """Record the newest command. Superseded ones are dropped, not queued."""
        q = np.asarray(msg.data, dtype=float)
        if q.size != self.n:
            # Silently truncating a wrong-sized command drives the arm to a
            # pose nobody asked for. Two projects sharing one ROS domain is
            # how that happens, and it looks exactly like flicker.
            self.get_logger().warning(
                f"Ignoring /joint_commands with {q.size} values; this model has "
                f"{self.n} joints. Another publisher is on the topic.",
                throttle_duration_sec=2.0)
            return
        self._pending = q

    def apply_pending(self):
        """Apply the latest command, once per rendered frame."""
        if self._pending is None:
            return
        q, self._pending = self._pending, None
        k = min(q.size, self.n)
        if self.dynamics:
            nu = min(k, self.model.nu)
            self.data.ctrl[:nu] = q[:nu]
        else:
            for i in range(k):
                self.data.qpos[self.qadr[i]] = q[i]
                self.data.qvel[self.vadr[i]] = 0.0
            now = time.time()
            if self._last_apply is not None and 0.0 < now - self._last_apply < 0.2:
                self._kin_qvel = (q[:self.n] - self._last_q) / (now - self._last_apply)
            else:
                self._kin_qvel = np.zeros(self.n)    # first command after idle: a jump, not motion
            self._last_q, self._last_apply = q[:self.n].copy(), now

    def publish_state(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(self.names)
        msg.position = [float(self.data.qpos[a]) for a in self.qadr]
        if self.dynamics:
            msg.velocity = [float(self.data.qvel[a]) for a in self.vadr]
        else:
            if self._last_apply is None or time.time() - self._last_apply > 0.1:
                self._kin_qvel = np.zeros(self.n)    # commands stopped: arm is at rest
            msg.velocity = [float(v) for v in self._kin_qvel]
        self.pub.publish(msg)


# ==============================================================================
# PCB SORTING CELL: conveyor indexing + grasping
# ==============================================================================
NEST_XY = np.array([-0.18, 0.0])   # inspection nest, from scene.xml
BOARD_Z = 0.031                    # board centre on the belt / nest
BOARD_HALF_T = 0.003               # board half thickness
BELT_PITCH = 0.05                  # m between boards on the belt
INDEX_PERIOD = 20.0                # s between conveyor indexes
INDEX_TIME = 2.0                   # s one index takes
GRIP_ATTACH = 0.026                # m pad-centre distance: closing past this grasps
GRIP_RELEASE = 0.030               # m pad-centre distance: opening past this releases
GRIP_CONTACT_Q = -0.31             # rad gripper angle with the pads on a 20 mm board
BINS = {"pass": (-0.10, -0.13), "rework": (-0.10, 0.13), "scrap": (-0.245, -0.09)}
BIN_HALF = 0.022
STATE_HZ = 10.0                    # /pcb_cell/state publish rate


class PcbCell:
    """
    Drives the bodies named pcb_* in both bridge modes.

    Conveyor: boards waiting on the belt are carried by it (their pose is set
    every step). Every INDEX_PERIOD seconds the belt moves the queue one
    BELT_PITCH toward the nest over INDEX_TIME, so the next board lands on the
    same nest spot. An index is postponed while the nest is still occupied or
    the gripper is low over it, and happens as soon as both are clear.

    Grasp: when the gripper closes past GRIP_ATTACH with a board between its
    pads, the board is locked to the hand (its pose is set from the hand pose
    every step) until the gripper opens past GRIP_RELEASE. On release it falls
    under physics in --dynamics mode; in kinematic mode, which has no gravity,
    it is dropped straight down onto whatever is below it.

    State: /pcb_cell/state (std_msgs/String, JSON) at STATE_HZ, so a sequencer
    (pick_place_node.py) knows when a board is waiting on the nest and can
    check where each released board landed:
        {"nest": "pass" | "rework" | "scrap" | null,   board waiting on the nest
         "held": kind | null,                           board in the gripper
         "belt": n,                                     boards still on the belt
         "indexing": bool,                              conveyor moving now
         "released": [{"board", "kind", "bin", "correct", "x", "y"}, ...]}
    """

    def __init__(self, model, data, node, dynamics, grip_qadr):
        self.m, self.d, self.node, self.dynamics = model, data, node, dynamics
        self.grip_qadr = grip_qadr
        name = lambda kind, i: mujoco.mj_id2name(model, kind, i) or ""
        boards = [b for b in range(model.nbody)
                  if name(mujoco.mjtObj.mjOBJ_BODY, b).startswith("pcb_")]
        self.qadr = {b: model.jnt_qposadr[model.body_jntadr[b]] for b in boards}
        self.dadr = {b: model.jnt_dofadr[model.body_jntadr[b]] for b in boards}
        self.kind = {}
        for b in boards:
            g = model.body_geomadr[b]
            mat = name(mujoco.mjtObj.mjOBJ_MATERIAL, model.geom_matid[g])
            self.kind[b] = mat.replace("pcb_", "") or "?"
        self.label = {b: f"{name(mujoco.mjtObj.mjOBJ_BODY, b)} ({self.kind[b].upper()})"
                      for b in boards}

        # Boards off the nest are on the belt, in slots of one pitch
        self.slot = {}
        for b in boards:
            y = float(model.body_pos[b][1])
            if y > 0.5 * BELT_PITCH:
                self.slot[b] = int(round(y / BELT_PITCH))
        self.queue = sorted(self.slot, key=self.slot.get)

        gid = lambda n: mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, n)
        self.pads = (gid("gripper_static_finger_pad_1"), gid("gripper_moving_finger_pad_1"))
        self.hand = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "gripper_static_finger")
        self.held, self.off_p, self.off_q = None, None, None
        self.prev_pads = None
        self.index_start = None
        self.next_due = time.time() + INDEX_PERIOD
        self.ray_groups = np.array([1, 0, 0, 0, 0, 0], dtype=np.uint8)   # scene geoms only
        self.released = []
        self.state_pub = node.create_publisher(String, "/pcb_cell/state", 10)
        self.next_state = 0.0

        on_nest = [self.label[b] for b in boards if b not in self.slot]
        node.get_logger().info(
            f"PCB cell: {len(boards)} boards. On the nest: {', '.join(on_nest) or 'none'}. "
            f"Conveyor indexes every {INDEX_PERIOD:.0f}s. Bins: green=PASS, "
            f"yellow=REWORK, red=SCRAP.")

    # ------------------------------------------------------------------
    def update(self, now):
        """Call once per physics step (dynamics) or per frame (kinematic)."""
        mujoco.mj_kinematics(self.m, self.d)
        self._conveyor(now)
        self._grasp()
        self._hold()
        if now >= self.next_state:
            self.next_state = now + 1.0 / STATE_HZ
            self._publish_state()

    def _publish_state(self):
        nest = None
        for b in self.qadr:
            if b in self.slot or b == self.held:
                continue
            p = self.d.xpos[b]
            if np.linalg.norm(p[:2] - NEST_XY) < 0.01 and p[2] < 0.04:
                nest = self.kind[b]
                break
        msg = String()
        msg.data = json.dumps({
            "nest": nest if self.index_start is None else None,
            "held": self.kind[self.held] if self.held is not None else None,
            "belt": len(self.queue),
            "indexing": self.index_start is not None,
            "released": self.released,
        })
        self.state_pub.publish(msg)

    # ------------------------------------------------------------------
    def _set_pose(self, b, pos, quat=(1.0, 0.0, 0.0, 0.0)):
        a = self.qadr[b]
        self.d.qpos[a:a + 3] = pos
        self.d.qpos[a + 3:a + 7] = quat
        self.d.qvel[self.dadr[b]:self.dadr[b] + 6] = 0.0

    def _nest_clear(self):
        for b in self.qadr:
            if b in self.slot or b == self.held:
                continue
            p = self.d.xpos[b]
            if np.linalg.norm(p[:2] - NEST_XY) < 0.02 and p[2] < 0.06:
                return False
        # Fingers down at board height over the nest. (The Ready pose, pads at
        # ~57 mm, clears an incoming board by 2 cm and must not block the belt,
        # or waiting there for the next board would wait forever.)
        for g in self.pads:
            p = self.d.geom_xpos[g]
            if np.linalg.norm(p[:2] - NEST_XY) < 0.04 and p[2] < 0.045:
                return False
        return True

    def _conveyor(self, now):
        s = 0.0
        if self.index_start is None:
            if self.queue and now >= self.next_due and self._nest_clear():
                self.index_start = now
        if self.index_start is not None:
            u = min(1.0, (now - self.index_start) / INDEX_TIME)
            s = u * u * (3.0 - 2.0 * u)                      # smooth start and stop
            if u >= 1.0:
                for b in self.queue:
                    self.slot[b] -= 1
                arrived = [b for b in self.queue if self.slot[b] <= 0]
                for b in arrived:
                    del self.slot[b]
                    self._set_pose(b, [NEST_XY[0], NEST_XY[1], BOARD_Z])
                self.queue = [b for b in self.queue if b in self.slot]
                self.index_start, s = None, 0.0
                self.next_due = now + INDEX_PERIOD
                for b in arrived:
                    self.node.get_logger().info(
                        f"Conveyor: {self.label[b]} is on the nest -> "
                        f"{self.kind[b].upper()} bin. {len(self.queue)} left on the belt.")
                if not self.queue:
                    self.node.get_logger().info(
                        "Conveyor: belt is empty (restart the bridge to reload it).")
        for b in self.queue:                                  # the belt carries them
            y = NEST_XY[1] + (self.slot[b] - s) * BELT_PITCH
            self._set_pose(b, [NEST_XY[0], y, BOARD_Z])

    def _grasp(self):
        sp, mp = (self.d.geom_xpos[g].copy() for g in self.pads)
        gap = float(np.linalg.norm(mp - sp))
        if self.held is None and self.prev_pads is not None:
            a, b_, prev_gap = self.prev_pads
            if prev_gap > GRIP_ATTACH >= gap:
                # Closing past the attach width: is a board between the pads
                # as they were just before (still open)?
                ab = b_ - a
                for b in self.qadr:
                    if b in self.slot:
                        continue
                    c = self.d.xpos[b]
                    t = float((c - a) @ ab / (ab @ ab))
                    perp = float(np.linalg.norm(c - (a + t * ab)))
                    if 0.15 < t < 0.85 and perp < 0.008:
                        self._attach(b)
                        break
        elif self.held is not None and gap > GRIP_RELEASE:
            self._release()
        self.prev_pads = (sp, mp, gap)

    def _attach(self, b):
        hp, hq = self.d.xpos[self.hand], self.d.xquat[self.hand]
        neg = np.zeros(4)
        mujoco.mju_negQuat(neg, hq)
        self.off_p = np.zeros(3)
        mujoco.mju_rotVecQuat(self.off_p, self.d.xpos[b] - hp, neg)
        self.off_q = np.zeros(4)
        mujoco.mju_mulQuat(self.off_q, neg, self.d.xquat[b])
        self.held = b
        self.node.get_logger().info(
            f"Gripped {self.label[b]} -> take it to the {self.kind[b].upper()} bin.")

    def _hold(self):
        if self.held is None:
            return
        if not self.dynamics and self.d.qpos[self.grip_qadr] > GRIP_CONTACT_Q:
            # No contacts in kinematic mode: stop the fingers on the board
            # instead of letting them close through it.
            self.d.qpos[self.grip_qadr] = GRIP_CONTACT_Q
            mujoco.mj_kinematics(self.m, self.d)
        hp, hq = self.d.xpos[self.hand], self.d.xquat[self.hand]
        p = np.zeros(3)
        mujoco.mju_rotVecQuat(p, self.off_p, hq)
        q = np.zeros(4)
        mujoco.mju_mulQuat(q, hq, self.off_q)
        self._set_pose(self.held, hp + p, q)

    def _release(self):
        b, self.held = self.held, None
        a = self.qadr[b]
        pos = self.d.qpos[a:a + 3].copy()
        if not self.dynamics:
            # Drop straight down onto the first surface below, lying flat with
            # its current heading.
            geomid = np.array([-1], dtype=np.int32)
            dist = mujoco.mj_ray(self.m, self.d, pos, np.array([0.0, 0.0, -1.0]),
                                 self.ray_groups, 1, b, geomid)
            floor_z = pos[2] - dist if dist >= 0 else 0.0
            R = np.zeros(9)
            mujoco.mju_quat2Mat(R, self.d.qpos[a + 3:a + 7])
            yaw = float(np.arctan2(R[3], R[0]))
            pos[2] = floor_z + BOARD_HALF_T
            self._set_pose(b, pos, [np.cos(yaw / 2), 0.0, 0.0, np.sin(yaw / 2)])
        else:
            self.d.qvel[self.dadr[b]:self.dadr[b] + 6] = 0.0
        where = next((k for k, (x, y) in BINS.items()
                      if abs(pos[0] - x) < BIN_HALF and abs(pos[1] - y) < BIN_HALF), None)
        if where is None:
            verdict = "not over a bin"
        elif where == self.kind[b]:
            verdict = f"into the {where.upper()} bin - correct"
        else:
            verdict = f"into the {where.upper()} bin - WRONG, it belongs in {self.kind[b].upper()}"
        self.node.get_logger().info(f"Released {self.label[b]} {verdict}.")
        self.released.append({
            "board": mujoco.mj_id2name(self.m, mujoco.mjtObj.mjOBJ_BODY, b),
            "kind": self.kind[b], "bin": where, "correct": where == self.kind[b],
            "x": round(float(pos[0]), 4), "y": round(float(pos[1]), 4)})


def run_viewer(xml_path, dynamics):
    import mujoco.viewer

    model = mujoco.MjModel.from_xml_path(str(xml_path))
    data = mujoco.MjData(model)
    names, qadr, vadr = resolve_joints(model)

    rclpy.init()
    node = MujocoBridge(model, data, names, qadr, vadr, dynamics)
    cell = PcbCell(model, data, node, dynamics, qadr[names.index("gripper")]) \
        if "gripper" in names else None

    frame_dt = 1.0 / 60.0          # match the display; syncing faster can tear
    next_frame = time.time()

    # Wall time that data.time == 0 corresponds to. The sim is stepped to catch
    # up with this, so sim time and wall time stay locked together.
    sim_epoch = time.time()
    # Catch-up budget per iteration. 50 steps (0.1 s of sim) was too tight: a
    # transient pause leaves a deficit of a few hundred ms, the budget clears
    # only 100 ms of it, and the remainder is forgiven - so sim time is lost
    # that the machine was perfectly capable of making up. 250 steps is 0.5 s
    # of sim per iteration, enough to absorb the pauses seen in practice.
    MAX_CATCHUP = 250
    # A deficit larger than this is not a hiccup - a window drag, a display
    # sleep, a debugger pause. Chasing it would block the render for seconds,
    # so it is forgiven in one go instead.
    MAX_DEFICIT = 1.0              # s
    # A residual smaller than this after the budget is noise; the next
    # iteration clears it, and it is not worth a warning or a resync.
    WARN_DEFICIT = 0.05            # s

    # Real-time factor report. If the sim runs slower than wall time, the
    # trajectory node's reference (indexed by wall clock) advances faster than
    # the arm physically can, and the tracking error it reports is inflated by
    # exactly that ratio. Knowing the number decides whether that matters.
    rtf_wall0 = time.time()
    rtf_sim0 = float(data.time)
    rtf_next = rtf_wall0 + 5.0
    resyncs = 0

    try:
        with mujoco.viewer.launch_passive(model, data) as viewer:
            # Opening the window costs a few hundred ms. sim_epoch was set
            # before that, so without this the loop starts already half a
            # second behind and burns its first resync on nothing.
            sim_epoch = time.time()
            rtf_wall0, rtf_sim0 = time.time(), float(data.time)
            rtf_next = rtf_wall0 + 5.0

            while viewer.is_running():
                # Drain every pending callback, then apply only the newest
                # command. Processing one per frame renders a backlog of stale
                # poses, which looks like flicker.
                for _ in range(64):
                    before = node._pending
                    rclpy.spin_once(node, timeout_sec=0.0)
                    if node._pending is before:
                        break
                node.apply_pending()

                if dynamics:
                    # Step until SIM time catches up with WALL time. A single
                    # mj_step per iteration advances the sim by
                    # model.opt.timestep (0.002 s for this model) while the
                    # iteration itself costs that plus the ROS drain and the
                    # render, so the sim would run at a fraction of real time.
                    # trajectory_node indexes its reference by wall clock, so a
                    # slow sim appears as tracking error the controller never
                    # caused.
                    target = time.time() - sim_epoch
                    deficit = target - float(data.time)

                    if deficit > MAX_DEFICIT:
                        # Not a hiccup. Forgive it rather than block the render.
                        sim_epoch = time.time() - data.time
                        resyncs += 1
                        node.get_logger().warning(
                            f"Sim clock stalled {deficit:.1f}s behind real time "
                            f"(window drag, display sleep or similar); forgiving "
                            f"it. Any trajectory running across this moment has "
                            f"its timing corrupted - re-run it.",
                            throttle_duration_sec=5.0)
                    else:
                        steps = 0
                        while data.time < target and steps < MAX_CATCHUP:
                            mujoco.mj_step(model, data)
                            if cell is not None:
                                cell.update(time.time())
                            steps += 1

                        # Exhausting the budget is only a problem if a
                        # meaningful deficit SURVIVES it. A few ms left over is
                        # cleared by the next iteration, with no lost sim time.
                        behind = target - float(data.time)
                        if steps >= MAX_CATCHUP and behind > WARN_DEFICIT:
                            sim_epoch = time.time() - data.time
                            resyncs += 1
                            node.get_logger().warning(
                                f"Physics is {behind*1000:.0f} ms behind real "
                                f"time after {steps} catch-up steps; resyncing. "
                                f"Tracking figures from this run understate the "
                                f"controller.",
                                throttle_duration_sec=5.0)
                else:
                    if cell is not None:
                        cell.update(time.time())
                    mujoco.mj_forward(model, data)

                now = time.time()
                if now >= next_frame:
                    viewer.sync()
                    next_frame = max(now, next_frame + frame_dt)

                if dynamics and now >= rtf_next:
                    wall = now - rtf_wall0
                    sim = float(data.time) - rtf_sim0
                    rtf = sim / wall if wall > 0 else 0.0
                    node.get_logger().info(
                        f"real-time factor {rtf:.2f}x over the last {wall:.0f}s "
                        f"({sim:.1f}s simulated), {resyncs} resync(s). "
                        f"1.00x means wall clock and sim clock agree.")
                    rtf_wall0, rtf_sim0, rtf_next = now, float(data.time), now + 5.0
                    resyncs = 0
                # Short sleep in dynamics mode: the catch-up loop needs to be
                # polled often enough that it steps one or two at a time rather
                # than in bursts.
                time.sleep(0.001 if dynamics else 0.002)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def run_compare(xml_path, n_samples=200, seed=0):
    """
    Check the DH forward kinematics (kinematics.py) against MuJoCo's own.

    This is the test kinematics.py's selftest cannot do: the selftest only
    proves the derivation is internally consistent, not that it matches the
    model described in the XML.
    """
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    from kinematics import ee_position

    model = mujoco.MjModel.from_xml_path(str(xml_path))
    data = mujoco.MjData(model)
    names, qadr, _ = resolve_joints(model)

    sid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "frame_ee")
    if sid < 0:
        print("No site named 'frame_ee' in the model; cannot compare.")
        print("Sites found:", [
            mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_SITE, i)
            for i in range(model.nsite)])
        return 1

    rng = np.random.default_rng(seed)
    worst, worst_q = 0.0, None

    for _ in range(n_samples):
        q = rng.uniform(-1.0, 1.0, 5)
        for i in range(5):
            data.qpos[qadr[i]] = q[i]
        mujoco.mj_forward(model, data)

        mine = ee_position(q)
        theirs = data.site_xpos[sid].copy()
        err = float(np.abs(mine - theirs).max())
        if err > worst:
            worst, worst_q = err, q.copy()

    print(f"joints compared : {', '.join(names[:5])}")
    print(f"samples         : {n_samples}")
    print(f"max |error|     : {worst:.3e} m")
    if worst < 1e-9:
        print("MATCH - the DH model agrees with the XML.")
        return 0
    print("MISMATCH - the derivation and the XML disagree.")
    print("worst-case q    :", np.array2string(worst_q, precision=4))
    return 1


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--xml", default=str(DEFAULT_XML))
    ap.add_argument("--dynamics", action="store_true",
                    help="drive actuators and integrate physics")
    ap.add_argument("--compare", action="store_true",
                    help="check the DH forward kinematics against MuJoCo, no viewer")
    args = ap.parse_args()

    xml = Path(args.xml)
    if not xml.exists():
        print(f"Model not found: {xml}")
        return 1

    if args.compare:
        return run_compare(xml)

    run_viewer(xml, args.dynamics)
    return 0


if __name__ == "__main__":
    sys.exit(main())