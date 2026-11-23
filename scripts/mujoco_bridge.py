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
    dynamic (--dynamics) commands go to the actuators and mj_step() integrates.
                         The arm lags, sags under gravity and respects limits.

Run with mjpython (macOS requires it for launch_passive):

    mjpython scripts/mujoco_bridge.py
    mjpython scripts/mujoco_bridge.py --dynamics
    python   scripts/mujoco_bridge.py --compare      # no viewer, checks fk_node
"""

import argparse
import os
import sys
import time
from pathlib import Path

import numpy as np
import mujoco
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
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


def run_viewer(xml_path, dynamics):
    import mujoco.viewer

    model = mujoco.MjModel.from_xml_path(str(xml_path))
    data = mujoco.MjData(model)
    names, qadr, vadr = resolve_joints(model)

    rclpy.init()
    node = MujocoBridge(model, data, names, qadr, vadr, dynamics)

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
    finally:
        node.destroy_node()
        rclpy.shutdown()


def run_compare(xml_path, n_samples=200, seed=0):
    """
    Check fk_node's hand-derived chain against MuJoCo's own kinematics.

    This is the test the existing selftest() cannot do: selftest only proves
    the derivation is internally consistent, not that it matches the XML.
    """
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    from fk_node import forward_kinematics

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

        mine = np.array(forward_kinematics(q)[:3])
        theirs = data.site_xpos[sid].copy()
        err = float(np.abs(mine - theirs).max())
        if err > worst:
            worst, worst_q = err, q.copy()

    print(f"joints compared : {', '.join(names[:5])}")
    print(f"samples         : {n_samples}")
    print(f"max |error|     : {worst:.3e} m")
    if worst < 1e-9:
        print("MATCH - the derived chain agrees with the XML.")
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
                    help="check fk_node against MuJoCo, no viewer")
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