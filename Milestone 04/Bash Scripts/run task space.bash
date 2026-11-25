#!/usr/bin/env bash
# PCB sorting validation run in TASK-SPACE mode (dynamics on).
#
# Opens the MuJoCo viewer + GUI, waits START_DELAY seconds so you can arrange
# the windows and press "Start Recording" in OBS, then sorts all 5 boards and
# shuts everything down. Trajectory CSV logs and launch.log are written to
# "Milestone 04/Data/task_space/".
#
#   bash "Milestone 04/Bash Scripts/run task space.bash"        # 30 s delay
#   START_DELAY=45 bash "Milestone 04/Bash Scripts/run task space.bash"
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
M4="$(dirname "$SCRIPT_DIR")"                  # Milestone 04/
PKG="$(dirname "$M4")"                          # package root
WS="$(cd "$PKG/../.." && pwd)"                  # ros2_ws/
START_DELAY="${START_DELAY:-30}"

# ROS 2 (Lyrical here; falls back to whatever distro is installed)
if [ -f /opt/ros/lyrical/setup.bash ]; then
    source /opt/ros/lyrical/setup.bash
else
    source /opt/ros/*/setup.bash
fi
# venv with DeepMind's mujoco package
[ -f "$WS/.venv/bin/activate" ] && source "$WS/.venv/bin/activate"

DATA="$M4/Data/task_space"
mkdir -p "$DATA"
rm -f "$DATA"/traj_*.csv "$DATA"/launch.log
cd "$DATA"

echo "=== TASK-SPACE run: windows open now, the arm starts moving in ${START_DELAY} s ==="
echo "=== Arrange the windows and press Start Recording in OBS. ==="
ros2 launch "$PKG/launch/bridge.launch.py" dynamics:=true demo:=task start_delay:="$START_DELAY" 2>&1 | tee launch.log

echo
echo "=== Done. Stop the OBS recording. Logs: $DATA ==="
grep -a -E "Sorting finished|  pcb_" launch.log | sed 's/\x1b\[[0-9;]*m//g; s/.*\]: //' || true
