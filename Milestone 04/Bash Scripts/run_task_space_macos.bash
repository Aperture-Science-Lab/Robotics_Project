#!/usr/bin/env bash
# PCB sorting validation run in TASK-SPACE mode (dynamics on) - macOS + pixi.
#
# Opens the MuJoCo viewer + GUI, waits START_DELAY seconds so you can arrange
# the windows and press "Start Recording" in OBS, then sorts all 5 boards and
# shuts everything down. Trajectory CSV logs and launch.log are written to
# "Milestone 04/Data/task_space/".
#
#   bash "Milestone 04/Bash Scripts/run task space.bash"        # 30 s delay
#   START_DELAY=45 bash "Milestone 04/Bash Scripts/run task space.bash"
#
# ROS 2 and MuJoCo come from a pixi environment (RoboStack), not /opt/ros or a
# venv. The script finds the nearest pixi.toml above this folder; set
# PIXI_MANIFEST=/path/to/pixi.toml to point it elsewhere. Running it from
# inside `pixi shell` also works (activation is then skipped).
#
# Works with macOS's stock bash 3.2 and BSD grep/sed.
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
M4="$(dirname "$SCRIPT_DIR")"                  # Milestone 04/
PKG="$(dirname "$M4")"                          # package root
WS="$(cd "$PKG/../.." && pwd)"                  # ros2_ws/
START_DELAY="${START_DELAY:-30}"

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

    # macOS ships bash 3.2. The activation scripts pixi emits also load the
    # environment's bash-completion files, which need bash >= 4 and abort a
    # bash 3.2 shell ("syntax error in conditional expression"). So:
    #   1. if a newer bash exists (Homebrew, or inside the pixi env), re-run
    #      this script with it;
    #   2. otherwise take the activated environment as JSON from pixi and
    #      export the variables directly, sourcing no shell scripts at all.
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
# MuJoCo's passive viewer (launch_passive) must run under mjpython on macOS.
# Export its path so bridge.launch.py can use it for mujoco_bridge.py.
if [ "$(uname)" = "Darwin" ]; then
    if command -v mjpython >/dev/null 2>&1; then
        export MJPYTHON="$(command -v mjpython)"
        echo "=== Viewer interpreter: $MJPYTHON ==="
    else
        echo "WARNING: mjpython not found in the pixi environment. On macOS the MuJoCo" >&2
        echo "         viewer will fail to open under plain python." >&2
    fi
fi

# Keep DDS traffic on this machine: no macOS firewall prompts, and no other
# project on the network can publish into our /joint_commands.
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
export ROS_AUTOMATIC_DISCOVERY_RANGE="${ROS_AUTOMATIC_DISCOVERY_RANGE:-LOCALHOST}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-1}"        # pre-Jazzy distros
# Print node output as it happens (tee would otherwise buffer it)
export PYTHONUNBUFFERED=1

# ------------------------------------------------------------------------------
# 3. Run
# ------------------------------------------------------------------------------
DATA="$M4/Data/task_space"
mkdir -p "$DATA"
rm -f "$DATA"/traj_*.csv "$DATA"/launch.log
cd "$DATA"

echo "=== TASK-SPACE run: windows open now, the arm starts moving in ${START_DELAY} s ==="
echo "=== Arrange the windows and press Start Recording in OBS. ==="
ros2 launch "$PKG/launch/bridge.launch.py" dynamics:=true demo:=task start_delay:="$START_DELAY" 2>&1 | tee launch.log

echo
echo "=== Done. Stop the OBS recording. Logs: $DATA ==="
# BSD sed has no \x1b escape: build the ESC character with printf instead
ESC="$(printf '\033')"
grep -a -E "Sorting finished|  pcb_" launch.log \
    | sed -e "s/${ESC}\[[0-9;]*m//g" -e 's/.*\]: //' || true