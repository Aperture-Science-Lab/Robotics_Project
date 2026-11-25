#!/usr/bin/env python3
"""
Pick-and-place sequencer for the PCB sorting cell.

Sorts every board the conveyor delivers into the bin of its colour
(green = PASS, yellow = REWORK, red = SCRAP) by sending trajectory
commands to trajectory_node, one leg at a time:

    wait for a board on the nest
    -> down to the board  -> close gripper -> lift
    -> over its bin       -> open gripper  -> back over the nest

--mode joint   every leg is a JOINT-SPACE quintic between the joint
               waypoints gui_node.PICK_PRESETS (one fuzzy loop per joint).
--mode task    every leg is a TASK-SPACE straight line between the
               Cartesian waypoints gui_node.TASK_PRESETS (MIMO fuzzy
               controller on the Cartesian error + J#).
               Only the first move from home to the Ready pose is in
               joint space: the task space here is the tool-tip POSITION
               only, so it cannot by itself bring the gripper upright.

The sequencer only talks to trajectory_node (/trajectory_command,
/trajectory/status) and watches the cell (/pcb_cell/state from
mujoco_bridge.py). It never writes /joint_commands itself, so the motion
seen is exactly the planners + controllers under test.

Run with the bridge in --dynamics mode:
    ros2 launch launch/bridge.launch.py dynamics:=true demo:=joint
    ros2 launch launch/bridge.launch.py dynamics:=true demo:=task
or by hand:
    python3 scripts/pick_place_node.py --mode task
"""

import argparse
import json
import sys
import threading
import time
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

sys.path.insert(0, str(Path(__file__).resolve().parent))
from gui_node import PICK_PRESETS, TASK_PRESETS              # noqa: E402
from trajectory_node import GRIP_OPEN, GRIP_CLOSED           # noqa: E402

# ------------------------------------------------------------------------------
# Waypoints (defined once, in gui_node.py, where the reasoning behind each
# value is documented)
# ------------------------------------------------------------------------------
_J = {name: list(q) for name, q in PICK_PRESETS}
JOINT_WP = {
    "ready":  _J["Ready (above nest)"],
    "grasp":  _J["Grasp (at nest)"],
    "pass":   _J["Pass bin (green)"],
    "rework": _J["Rework bin (yellow)"],
    "scrap":  _J["Scrap bin (red)"],
}
_X = {name: list(x) for name, x in TASK_PRESETS}
TASK_WP = {
    "ready":  _X["Above nest"],
    "grasp":  _X["At PCB (grasp)"],
    "pass":   _X["Above pass bin"],
    "rework": _X["Above rework bin"],
    "scrap":  _X["Above scrap bin"],
}

# Leg durations, s
T_FIRST = 3.0       # home -> ready
T_DOWN = 1.5        # ready -> grasp
T_LIFT = 1.5        # grasp -> ready
T_TO_BIN = 2.5      # ready -> bin
T_BACK = 2.5        # bin -> ready
T_GRIP = 1.0        # gripper open/close settle


class PickPlaceNode(Node):
    def __init__(self, mode, n_boards, start_delay=0.0):
        super().__init__("pick_place_node")
        self.mode = mode
        self.n_boards = n_boards
        self.start_delay = start_delay

        self.cmd_pub = self.create_publisher(String, "/trajectory_command", 10)
        self.create_subscription(String, "/trajectory/status", self.on_status, 10)
        self.create_subscription(String, "/pcb_cell/state", self.on_cell, 10)

        self.status = None
        self.status_t = 0.0
        self.cell = None
        self.lock = threading.Lock()

    # ---------------- feedback ----------------
    def on_status(self, msg):
        try:
            st = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        with self.lock:
            self.status, self.status_t = st, time.time()

    def on_cell(self, msg):
        try:
            st = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        with self.lock:
            self.cell = st

    def running(self):
        with self.lock:
            return bool(self.status and self.status.get("running"))

    def cell_state(self):
        with self.lock:
            return dict(self.cell) if self.cell else None

    # ---------------- commands ----------------
    def send(self, payload):
        msg = String()
        msg.data = json.dumps(payload)
        self.cmd_pub.publish(msg)

    def wait_until(self, cond, timeout, what):
        t_end = time.time() + timeout
        while time.time() < t_end:
            if not rclpy.ok():
                raise RuntimeError("ROS shut down")
            if cond():
                return True
            time.sleep(0.02)
        raise TimeoutError(f"timed out waiting for {what}")

    def move(self, target, T, label, space=None):
        space = space or self.mode
        if space == "joint":
            payload = {"mode": "joint", "q_end": list(map(float, target)), "T": T}
        else:
            payload = {"mode": "task", "x_end": list(map(float, target)), "T": T}
        self.get_logger().info(f"  {space.upper()}-SPACE  {label}  ({T:.1f} s)")
        self.send(payload)
        # status is published at 5 Hz: wait for the run to show, then end
        self.wait_until(self.running, 2.0, f"'{label}' to start")
        self.wait_until(lambda: not self.running(), T + 5.0, f"'{label}' to finish")

    def gripper(self, value, label):
        self.get_logger().info(f"  gripper {label}")
        self.send({"mode": "gripper", "value": float(value)})
        time.sleep(T_GRIP)

    def wp(self, name):
        return (JOINT_WP if self.mode == "joint" else TASK_WP)[name]

    # ---------------- sequence ----------------
    def run(self):
        L = self.get_logger()
        L.info(f"Pick-and-place in {self.mode.upper()}-SPACE mode. "
               f"Waiting for trajectory_node and the PCB cell...")
        self.wait_until(lambda: self.status is not None and self.cell is not None,
                        60.0, "/trajectory/status and /pcb_cell/state")
        time.sleep(1.0)
        if self.start_delay > 0.0:
            # Time to arrange the viewer / GUI windows before anything moves
            # (record_demo.py starts the screen recording just before this ends).
            L.info(f"Starting in {self.start_delay:.0f} s...")
            time.sleep(self.start_delay)

        self.gripper(GRIP_OPEN, "open")
        # The first move is always joint space: it brings the gripper upright
        # over the nest, which a position-only task-space move cannot promise.
        self.move(JOINT_WP["ready"], T_FIRST, "home -> ready (above nest)", space="joint")
        if self.mode == "task":
            self.move(TASK_WP["ready"], 1.0, "align tool tip over the board")

        done, failures = 0, 0
        t_start = time.time()
        while done < self.n_boards:
            cell = self.cell_state()
            if cell["nest"] is None:
                if cell["belt"] == 0 and not cell["indexing"]:
                    L.info("Belt is empty.")
                    break
                L.info("Waiting for the conveyor to bring the next board...")
                self.wait_until(lambda: (self.cell_state() or {}).get("nest") is not None,
                                60.0, "a board on the nest")
                cell = self.cell_state()
            kind = cell["nest"]
            L.info(f"[board {done + 1}] {kind.upper()} board on the nest -> {kind.upper()} bin")

            self.move(self.wp("grasp"), T_DOWN, "down to the board")
            self.gripper(GRIP_CLOSED, "close (grasp)")
            if self.cell_state()["held"] != kind:
                failures += 1
                L.warning(f"  grasp failed: board not in the gripper (attempt {failures})")
                self.gripper(GRIP_OPEN, "open")
                self.move(self.wp("ready"), T_LIFT, "back up")
                if failures >= 3:
                    L.error("Giving up after 3 failed grasps.")
                    break
                continue
            self.move(self.wp("ready"), T_LIFT, "lift")
            self.move(self.wp(kind), T_TO_BIN, f"over the {kind.upper()} bin")
            self.gripper(GRIP_OPEN, "open (release)")
            self.move(self.wp("ready"), T_BACK, "back over the nest")
            done += 1

        released = self.cell_state()["released"]
        n_ok = sum(1 for r in released if r["correct"])
        L.info(f"Sorting finished in {time.time() - t_start:.0f} s: "
               f"{n_ok}/{len(released)} boards in the correct bin.")
        for r in released:
            L.info(f"  {r['board']}: {r['kind'].upper():6s} -> "
                   f"{(r['bin'] or 'no bin').upper():6s} "
                   f"{'OK' if r['correct'] else 'WRONG'}  at ({r['x']:+.3f}, {r['y']:+.3f})")
        self.move([0.0] * 5, T_FIRST, "ready -> home", space="joint")
        return n_ok == len(released) and len(released) > 0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--mode", choices=["joint", "task"], default="joint")
    ap.add_argument("--boards", type=int, default=5,
                    help="number of boards to sort (the belt holds 5)")
    ap.add_argument("--start-delay", type=float, default=0.0,
                    help="seconds to wait once the system is up, before the first move")
    ap.add_argument("--exit-delay", type=float, default=3.0,
                    help="seconds to stay up after the last move")
    args, _ = ap.parse_known_args()

    rclpy.init()
    node = PickPlaceNode(args.mode, args.boards, args.start_delay)
    spinner = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spinner.start()
    ok = False
    try:
        ok = node.run()
        time.sleep(args.exit_delay)
    except (TimeoutError, RuntimeError) as exc:
        node.get_logger().error(str(exc))
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
