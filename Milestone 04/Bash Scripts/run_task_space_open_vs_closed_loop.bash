#!/usr/bin/env bash
# TASK-SPACE PCB sorting run, twice: CLOSED LOOP then OPEN LOOP (dynamics on),
# followed by one figure overlaying both runs - macOS + pixi.
#
#   Run 1  closed loop   fuzzy logic controller ON  (feedback from /joint_state)
#   Run 2  open loop     fuzzy logic controller OFF (planned X_dot_des resolved
#                        through J#(q_cmd) and integrated, no feedback)
#
# Each run opens the MuJoCo viewer + GUI, waits START_DELAY seconds (arrange
# the windows / start OBS), sorts all 5 boards and shuts down. Logs go to
#   "Milestone 04/Data/task_space_closed_vs_open/closed/"
#   "Milestone 04/Data/task_space_closed_vs_open/open/"
# and the figures to
#   "Milestone 04/Data/task_space_closed_vs_open/closed_vs_open_xyz.png"
#   "Milestone 04/Data/task_space_closed_vs_open/closed_vs_open_tool_path.png"
#
#   bash "Milestone 04/Bash Scripts/run task space closed vs open.bash"
#   START_DELAY=45 bash "..."          # longer pause before each run
#   PAUSE_BETWEEN=10 bash "..."        # seconds between the two runs (default 5)
#   RUN_TIMEOUT=900 bash "..."         # stop a run that has not finished (default 600 s)
#   PLOT_ONLY=1 bash "..."             # re-draw the figure from existing logs
#
# Needs trajectory_node.py with the open/closed-loop option: the loop mode of a
# whole run is chosen with the TRAJ_USE_CONTROLLER environment variable, which
# ros2 launch passes on to every node it starts.
#
# Works with macOS's stock bash 3.2 and BSD grep/sed.
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
M4="$(dirname "$SCRIPT_DIR")"                  # Milestone 04/
PKG="$(dirname "$M4")"                          # package root
WS="$(cd "$PKG/../.." && pwd)"                  # ros2_ws/
START_DELAY="${START_DELAY:-30}"
PAUSE_BETWEEN="${PAUSE_BETWEEN:-5}"
RUN_TIMEOUT="${RUN_TIMEOUT:-600}"

# ------------------------------------------------------------------------------
# 1. Activate the pixi environment (ROS 2 + MuJoCo)
# ------------------------------------------------------------------------------
find_manifest() {
    local d="$1"
    while [ "$d" != "/" ]; do
        if [ -f "$d/pixi.toml" ]; then echo "$d/pixi.toml"; return 0; fi
        if [ -f "$d/pyproject.toml" ] && grep -q '^\[tool\.pixi' "$d/pyproject.toml"; then
            echo "$d/pyproject.toml"; return 0
        fi
        d="$(dirname "$d")"
    done
    return 1
}

if [ -n "${PIXI_ENVIRONMENT_NAME:-}" ]; then
    echo "=== Already inside pixi environment '${PIXI_ENVIRONMENT_NAME}' ==="
else
    if ! command -v pixi >/dev/null 2>&1; then
        echo "ERROR: pixi not found on PATH. Install it (https://pixi.sh) or run from 'pixi shell'." >&2
        exit 1
    fi
    MANIFEST="${PIXI_MANIFEST:-$(find_manifest "$SCRIPT_DIR" || true)}"
    if [ -z "$MANIFEST" ] || [ ! -f "$MANIFEST" ]; then
        echo "ERROR: no pixi.toml found above $SCRIPT_DIR. Set PIXI_MANIFEST=/path/to/pixi.toml." >&2
        exit 1
    fi
    PIXI_ROOT="$(dirname "$MANIFEST")"

    # macOS ships bash 3.2, and pixi's activation loads bash-completion files
    # that need bash >= 4. Re-run with a newer bash if there is one, else take
    # the environment from pixi as JSON and export it directly.
    if [ "${BASH_VERSINFO[0]}" -lt 4 ] && [ -z "${_RERUN_WITH_NEW_BASH:-}" ]; then
        for b in /opt/homebrew/bin/bash /usr/local/bin/bash "$PIXI_ROOT/.pixi/envs/default/bin/bash"; do
            if [ -x "$b" ] && "$b" -c '[ "${BASH_VERSINFO[0]}" -ge 4 ]' 2>/dev/null; then
                echo "=== bash ${BASH_VERSION} is too old for pixi activation; re-running with $b ==="
                export _RERUN_WITH_NEW_BASH=1
                exec "$b" "${BASH_SOURCE[0]}" "$@"
            fi
        done
    fi

    echo "=== Activating pixi environment from $MANIFEST ==="
    if [ "${BASH_VERSINFO[0]}" -ge 4 ]; then
        eval "$(pixi shell-hook --manifest-path "$MANIFEST")"
    else
        PY=""
        for p in "$PIXI_ROOT/.pixi/envs/default/bin/python3" "$PIXI_ROOT/.pixi/envs/default/bin/python"; do
            [ -x "$p" ] && PY="$p" && break
        done
        if [ -z "$PY" ]; then
            echo "ERROR: bash ${BASH_VERSION} cannot source pixi's activation, and no python was" >&2
            echo "       found in $PIXI_ROOT/.pixi/envs/default. Run 'pixi install' first, or" >&2
            echo "       'brew install bash', then re-run this script." >&2
            exit 1
        fi
        ENV_JSON="$(pixi shell-hook --json --manifest-path "$MANIFEST")"
        eval "$("$PY" -c '
import json, shlex, sys
d = json.load(sys.stdin)
d = d.get("environment_variables", d)
for k, v in d.items():
    if k.isidentifier() and isinstance(v, str):
        print("export %s=%s" % (k, shlex.quote(v)))
' <<< "$ENV_JSON")"
    fi
fi

if ! command -v ros2 >/dev/null 2>&1; then
    echo "ERROR: 'ros2' is not available in the pixi environment." >&2
    exit 1
fi

# Built workspace overlay, if the package was built with colcon inside pixi
[ -f "$WS/install/setup.bash" ] && source "$WS/install/setup.bash"

# ------------------------------------------------------------------------------
# 2. macOS specifics
# ------------------------------------------------------------------------------
if [ "$(uname)" = "Darwin" ]; then
    if command -v mjpython >/dev/null 2>&1; then
        export MJPYTHON="$(command -v mjpython)"
        echo "=== Viewer interpreter: $MJPYTHON ==="
    else
        echo "WARNING: mjpython not found in the pixi environment. On macOS the MuJoCo" >&2
        echo "         viewer will fail to open under plain python." >&2
    fi
fi

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
export ROS_AUTOMATIC_DISCOVERY_RANGE="${ROS_AUTOMATIC_DISCOVERY_RANGE:-LOCALHOST}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-1}"        # pre-Jazzy distros
export PYTHONUNBUFFERED=1

ROOT="$M4/Data/task_space_closed_vs_open"
CLOSED_DIR="$ROOT/closed"
OPEN_DIR="$ROOT/open"
ESC="$(printf '\033')"

# ------------------------------------------------------------------------------
# 3. One run: launch, log, stop it if it hangs
# ------------------------------------------------------------------------------
run_once() {   # run_once <label> <true|false> <dir>
    local label="$1" controller="$2" dir="$3"
    mkdir -p "$dir"
    rm -f "$dir"/traj_*.csv "$dir"/launch.log
    cd "$dir"

    echo
    echo "======================================================================"
    echo "=== TASK-SPACE ${label}: windows open now, the arm starts moving in ${START_DELAY} s"
    echo "=== Arrange the windows and press Start Recording in OBS."
    echo "======================================================================"

    # The loop mode for the whole run; ros2 launch hands it to trajectory_node
    export TRAJ_USE_CONTROLLER="$controller"

    ros2 launch "$PKG/launch/bridge.launch.py" dynamics:=true demo:=task \
        start_delay:="$START_DELAY" > >(tee launch.log) 2>&1 &
    local pid=$! t=0

    # Watchdog: an open-loop run may miss a grasp and never finish
    while kill -0 "$pid" 2>/dev/null; do
        sleep 1
        t=$((t + 1))
        if [ "$t" -ge "$RUN_TIMEOUT" ]; then
            echo "=== ${label}: no finish after ${RUN_TIMEOUT} s - stopping the launch ==="
            kill -INT "$pid" 2>/dev/null || true
            sleep 10
            kill -TERM "$pid" 2>/dev/null || true
            break
        fi
    done
    wait "$pid" 2>/dev/null || true
    sleep 1                                   # let tee flush launch.log

    echo "=== ${label} done. Logs: $dir ==="
    grep -a -E "Sorting finished|  pcb_" launch.log \
        | sed -e "s/${ESC}\[[0-9;]*m//g" -e 's/.*\]: //' || true
    local n
    n=$(ls "$dir"/traj_task_*.csv 2>/dev/null | wc -l | tr -d ' ')
    echo "=== ${label}: ${n} trajectory log(s) written ==="
    cd "$ROOT"
}

if [ -z "${PLOT_ONLY:-}" ]; then
    mkdir -p "$ROOT"
    run_once "CLOSED LOOP (fuzzy controller ON)" true  "$CLOSED_DIR"
    echo
    echo "=== Next: OPEN LOOP in ${PAUSE_BETWEEN} s. Keep or restart the OBS recording. ==="
    sleep "$PAUSE_BETWEEN"
    run_once "OPEN LOOP (no controller)"         false "$OPEN_DIR"
fi

# ------------------------------------------------------------------------------
# 4. Overlay both runs in one figure
# ------------------------------------------------------------------------------
PLOT_PY="$(command -v python3 || command -v python)"
if ! "$PLOT_PY" -c "import matplotlib, numpy" 2>/dev/null; then
    echo "WARNING: matplotlib is not in the pixi environment, so no figure was drawn." >&2
    echo "         Add it once with:  pixi add matplotlib" >&2
    echo "         then redraw with:  PLOT_ONLY=1 bash \"${BASH_SOURCE[0]}\"" >&2
    exit 0
fi

echo
echo "=== Drawing the closed-loop vs open-loop figures ==="
"$PLOT_PY" - "$CLOSED_DIR" "$OPEN_DIR" "$ROOT" <<'PYEOF'
"""
Overlay the closed-loop and open-loop task-space runs.

Each run is a sequence of straight-line legs (one traj_task_*.csv per leg, in
time order). Leg i of one run is the same move as leg i of the other, so the
legs are laid end to end on one time axis, each leg starting where the longer
of the two runs' previous leg ended.

Task-space logs: x_ref = X_des(t), the planned straight line;
                 x_meas = f(q_act), forward kinematics of the MEASURED joints.
"""
import glob
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

closed_dir, open_dir, out_dir = sys.argv[1:4]
AXES = ["X", "Y", "Z"]


def load(d):
    legs = []
    for f in sorted(glob.glob(os.path.join(d, "traj_task_*.csv"))):
        a = np.genfromtxt(f, delimiter=",", names=True)
        if a.size < 2:
            continue
        legs.append({
            "t": a["t"],
            "x_ref": np.column_stack([a[f"x_ref_{c}"] for c in "xyz"]),
            "x_meas": np.column_stack([a[f"x_meas_{c}"] for c in "xyz"]),
            "frozen": a["frozen"],
        })
    return legs


closed, opened = load(closed_dir), load(open_dir)
if not closed or not opened:
    sys.exit(f"Need logs from both runs: closed={len(closed)} legs, open={len(opened)} legs.")
n_legs = min(len(closed), len(opened))
if len(closed) != len(opened):
    print(f"Note: closed run has {len(closed)} legs, open run {len(opened)}; "
          f"comparing the first {n_legs}.")

offsets, t0 = [], 0.0
for i in range(n_legs):
    offsets.append(t0)
    t0 += max(closed[i]["t"][-1], opened[i]["t"][-1]) + 0.3


def series(legs, key):
    ts, ys = [], []
    for i in range(n_legs):
        ts.append(legs[i]["t"] + offsets[i]); ys.append(legs[i][key])
        ts.append([np.nan]); ys.append(np.full((1, legs[i][key].shape[1]), np.nan))
    return np.concatenate(ts), np.vstack(ys)


t_c, xr_c = series(closed, "x_ref")
_, xm_c = series(closed, "x_meas")
t_o, xr_o = series(opened, "x_ref")
_, xm_o = series(opened, "x_meas")

C_REF, C_CL, C_OL = "#222222", "#1f77b4", "#d62728"
MM = 1000.0

# ---- Figure 1: X, Y, Z against time, plus the Cartesian tracking error ----
fig, axes = plt.subplots(4, 1, figsize=(12, 10.5), sharex=True)
for k in range(3):
    ax = axes[k]
    ax.plot(t_c, xr_c[:, k] * MM, "--", color=C_REF, lw=1.2,
            label="Planned X_des, straight line (closed-loop run)")
    ax.plot(t_c, xm_c[:, k] * MM, color=C_CL, lw=1.4, label="Closed loop (fuzzy controller)")
    ax.plot(t_o, xm_o[:, k] * MM, color=C_OL, lw=1.4, label="Open loop (no controller)")
    ax.set_ylabel(f"{AXES[k]} (mm)")
    ax.grid(alpha=0.3)
    for o in offsets[1:]:
        ax.axvline(o - 0.15, color="#999999", lw=0.6, ls=":")

e_c = np.linalg.norm(xr_c - xm_c, axis=1) * MM
e_o = np.linalg.norm(xr_o - xm_o, axis=1) * MM
ax = axes[3]
ax.plot(t_c, e_c, color=C_CL, lw=1.4, label="Closed loop")
ax.plot(t_o, e_o, color=C_OL, lw=1.4, label="Open loop")
ax.set_ylabel("|X_des - X_meas|\n(mm)")
ax.set_xlabel("Time (s), trajectory legs laid end to end (dotted lines = leg boundaries)")
ax.grid(alpha=0.3)
for o in offsets[1:]:
    ax.axvline(o - 0.15, color="#999999", lw=0.6, ls=":")

handles, labels = axes[0].get_legend_handles_labels()
fig.legend(handles, labels, loc="upper center", bbox_to_anchor=(0.5, 0.94), ncol=3,
           frameon=False, fontsize=10.5)


def stats(legs):
    err = [np.linalg.norm(L["x_ref"] - L["x_meas"], axis=1) * MM for L in legs[:n_legs]]
    allerr = np.concatenate(err)
    return np.sqrt(np.mean(allerr ** 2)), np.max(allerr), np.mean([e[-1] for e in err])


rc, mc, fc = stats(closed)
ro, mo, fo = stats(opened)
fig.suptitle("Task-space trajectory tracking: closed loop vs open loop "
             f"({n_legs} legs, MuJoCo --dynamics)\n"
             f"RMS error  closed {rc:.2f} mm, open {ro:.2f} mm   |   "
             f"final error (mean over legs)  closed {fc:.2f} mm, open {fo:.2f} mm",
             y=0.995, fontsize=12)
fig.tight_layout(rect=(0, 0, 1, 0.905))
p1 = os.path.join(out_dir, "closed_vs_open_xyz.png")
fig.savefig(p1, dpi=150)

# ---- Figure 2: tool-tip path in 3D ----
xr = np.vstack([L["x_ref"] for L in closed[:n_legs]]) * MM
xc = np.vstack([L["x_meas"] for L in closed[:n_legs]]) * MM
xo = np.vstack([L["x_meas"] for L in opened[:n_legs]]) * MM
fig2 = plt.figure(figsize=(9, 7.5))
ax3 = fig2.add_subplot(111, projection="3d")
ax3.plot(*xr.T, "--", color=C_REF, lw=1.2, label="Planned straight-line path")
ax3.plot(*xc.T, color=C_CL, lw=1.4, label="Closed loop")
ax3.plot(*xo.T, color=C_OL, lw=1.4, label="Open loop")
ax3.set_xlabel("X (mm)"); ax3.set_ylabel("Y (mm)"); ax3.set_zlabel("Z (mm)")
ax3.set_title("Tool-tip path, task-space trajectories: closed vs open loop")
ax3.legend(loc="upper left")
fig2.tight_layout()
p2 = os.path.join(out_dir, "closed_vs_open_tool_path.png")
fig2.savefig(p2, dpi=150)

print(f"RMS Cartesian error   closed {rc:.2f} mm   open {ro:.2f} mm")
print(f"Max Cartesian error   closed {mc:.2f} mm   open {mo:.2f} mm")
print(f"Final error, mean     closed {fc:.2f} mm   open {fo:.2f} mm")
print(f"Figures: {p1}\n         {p2}")
PYEOF

echo "=== All done. Stop the OBS recording. ==="