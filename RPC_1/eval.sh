#!/usr/bin/env bash
WS="$HOME/project_name"
KIN=$(find "$WS/src" -name kinematics.py | head -n 1)
TOOLS="$WS/src/Robotics_Project/RPC_1/rpc_tools.py"

export LD_LIBRARY_PATH="$HOME/.mujoco/mujoco-3.2.0/lib:$LD_LIBRARY_PATH"
source /opt/ros/jazzy/setup.bash >/dev/null 2>&1
source "$WS/install/setup.bash" >/dev/null 2>&1

rebuild_node() {
    echo -e "\n--> Rebuilding panda_kinematics..."
    (cd "$WS" && bash "$WS/src/Robotics_Project/RPC_1/build.sh" >/dev/null 2>&1)
    source "$WS/install/setup.bash" >/dev/null 2>&1
    echo "--> Done."
}

while true; do
    clear
    echo "=========================================================="
    echo "        MCTR911 RPC 1 - EVALUATION CONTROL PANEL          "
    echo "=========================================================="
    echo "  1) Run Complete Math & Kinematics Audit (Tests A-D)"
    echo "  2) Check Live Arm Accuracy vs Target (reads /joint_state)"
    echo "----------------------------------------------------------"
    echo "  3) Switch to Target 1 [Normal Pose]"
    echo "  4) Switch to Target 2 [Alternative Reachable Pose]"
    echo "  5) Switch to Singularity Target [q5 = 0.0 rad]"
    echo "  6) Switch to Unreachable Target [Position: 2, 0, 1]"
    echo "  7) Revert Unreachable Target"
    echo "----------------------------------------------------------"
    echo "  8) Launch Full System (Sim + Node in 1 window)"
    echo "  q) Quit"
    echo "=========================================================="
    read -rp "Select an option: " opt

    case $opt in
        1)
            python3 "$TOOLS" math
            read -rp "Press Enter to return to menu..."
            ;;
        2)
            python3 "$TOOLS" check
            read -rp "Press Enter to return to menu..."
            ;;
        3)
            python3 -c "
path = '$KIN'
with open(path, 'r') as f: content = f.read()
import re
content = re.sub(r'Q_TARGET = np\.array\(.*?\)', 'Q_TARGET = np.array([0.3, -0.4, 0.2, -1.8, 0.1, 1.6, 0.5])', content)
content = re.sub(r'TARGET_POSE\[:3, 3\] = \[.*\]', '', content)
with open(path, 'w') as f: f.write(content)
"
            rebuild_node
            read -rp "Target 1 active. Press Enter to return to menu..."
            ;;
        4)
            python3 -c "
path = '$KIN'
with open(path, 'r') as f: content = f.read()
import re
content = re.sub(r'Q_TARGET = np\.array\(.*?\)', 'Q_TARGET = np.array([-0.8, 0.5, -0.6, -2.2, 0.4, 2.0, -0.3])', content)
content = re.sub(r'TARGET_POSE\[:3, 3\] = \[.*\]', '', content)
with open(path, 'w') as f: f.write(content)
"
            rebuild_node
            read -rp "Target 2 active. Press Enter to return to menu..."
            ;;
        5)
            python3 -c "
path = '$KIN'
with open(path, 'r') as f: content = f.read()
import re
content = re.sub(r'Q_TARGET = np\.array\(.*?\)', 'Q_TARGET = np.array([0.3, -0.4, 0.2, -1.8, 0.0, 1.6, 0.5])', content)
content = re.sub(r'TARGET_POSE\[:3, 3\] = \[.*\]', '', content)
with open(path, 'w') as f: f.write(content)
"
            rebuild_node
            read -rp "Singularity Target (q5=0) active. Press Enter to return to menu..."
            ;;
        6)
            python3 -c "
path = '$KIN'
with open(path, 'r') as f: content = f.read()
if 'TARGET_POSE[:3, 3]' not in content:
    content = content.replace('TARGET_POSE = forward_kinematics(Q_TARGET)', 'TARGET_POSE = forward_kinematics(Q_TARGET)\nTARGET_POSE[:3, 3] = [2, 0, 1]')
with open(path, 'w') as f: f.write(content)
"
            rebuild_node
            read -rp "Unreachable target active. Press Enter to return to menu..."
            ;;
        7)
            python3 -c "
path = '$KIN'
with open(path, 'r') as f: content = f.read()
content = content.replace('\nTARGET_POSE[:3, 3] = [2, 0, 1]', '')
with open(path, 'w') as f: f.write(content)
"
            rebuild_node
            read -rp "Unreachable target reverted. Press Enter to return to menu..."
            ;;
        8)
            export LD_LIBRARY_PATH="$HOME/.mujoco/mujoco-3.2.0/lib:$LD_LIBRARY_PATH"
            ros2 launch panda_kinematics demo.launch.py
            ;;
        q)
            exit 0
            ;;
        *)
            echo "Invalid option."
            sleep 1
            ;;
    esac
done
