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

PUBLISH_HZ = 50.0


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

        self.create_subscription(
            Float64MultiArray, "/joint_commands", self.on_command, 1)
        self.pub = self.create_publisher(JointState, "/joint_state", 10)
        self.create_timer(1.0 / PUBLISH_HZ, self.publish_state)

        mode = "dynamic (ctrl + mj_step)" if dynamics else "kinematic (qpos + mj_forward)"
        self.get_logger().info(f"MuJoCo bridge up in {mode} mode.")
        self.get_logger().info(f"Driving joints: {', '.join(self.names)}")
        if dynamics and model.nu == 0:
            self.get_logger().warn(
                "Model declares no actuators; --dynamics will not move the arm.")

    def on_command(self, msg):
        """Record the newest command. Superseded ones are dropped, not queued."""
        self._pending = np.asarray(msg.data, dtype=float)

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

    def publish_state(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(self.names)
        msg.position = [float(self.data.qpos[a]) for a in self.qadr]
        msg.velocity = [float(self.data.qvel[a]) for a in self.vadr]
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

    try:
        with mujoco.viewer.launch_passive(model, data) as viewer:
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
                    mujoco.mj_step(model, data)
                else:
                    mujoco.mj_forward(model, data)

                now = time.time()
                if now >= next_frame:
                    viewer.sync()
                    next_frame = max(now, next_frame + frame_dt)
                time.sleep(0.002)
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