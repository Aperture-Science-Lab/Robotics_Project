"""
Launch the DeepMind MuJoCo bridge together with the FK, IK and GUI nodes.

This is the Python-only alternative to simulation.launch.py: instead of the
C++ mujoco_node, the simulator is scripts/mujoco_bridge.py, which runs
DeepMind's official `mujoco` Python package (engine + viewer) and bridges
/joint_commands -> MuJoCo -> /joint_state.

The scripts are run straight from the source tree, so no colcon build is needed:

    ros2 launch launch/bridge.launch.py                  # from the package root
    ros2 launch launch/bridge.launch.py dynamics:=true   # physics instead of kinematic

The `mujoco` Python package must be importable by the chosen interpreter
(e.g. activate your venv or pixi shell first). On macOS the bridge needs
`mjpython`, which is picked automatically; override with python:=<path>.

Closing the MuJoCo viewer or the GUI window shuts everything down.
"""

import sys
from pathlib import Path

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess, LogInfo,
                            RegisterEventHandler)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, PythonExpression


def find_scripts_dir() -> Path:
    # resolve() follows symlinks, so this also works from a --symlink-install.
    scripts = Path(__file__).resolve().parent.parent / "scripts"
    if not (scripts / "mujoco_bridge.py").exists():
        raise FileNotFoundError(
            f"Could not find mujoco_bridge.py in {scripts}. Run this launch file "
            "from the source tree (ros2 launch launch/bridge.launch.py) or build "
            "with colcon build --symlink-install.")
    return scripts


def generate_launch_description():
    scripts = find_scripts_dir()

    default_python = "mjpython" if sys.platform == "darwin" else "python3"

    python = LaunchConfiguration("python")
    dynamics = LaunchConfiguration("dynamics")

    args = [
        DeclareLaunchArgument(
            "python", default_value=default_python,
            description="Interpreter for the bridge (needs the `mujoco` package; mjpython on macOS)."),
        DeclareLaunchArgument(
            "dynamics", default_value="false",
            description="true: drive actuators and integrate physics; false: kinematic (qpos) mode."),
    ]

    # Unbuffered so print()/log output shows up live in the launch terminal.
    env = {"PYTHONUNBUFFERED": "1"}

    def script(name, cmd_args=None, interpreter="python3", condition=None):
        return ExecuteProcess(
            cmd=[interpreter, str(scripts / name)] + (cmd_args or []),
            name=name.removesuffix(".py"),
            output="screen",
            emulate_tty=True,
            additional_env=env,
            condition=condition,
        )

    # 1. MuJoCo simulator + ROS bridge (DeepMind viewer window).
    bridge_kinematic = script(
        "mujoco_bridge.py", interpreter=python,
        condition=IfCondition(PythonExpression(["'", dynamics, "'.lower() != 'true'"])))
    bridge_dynamic = script(
        "mujoco_bridge.py", ["--dynamics"], interpreter=python,
        condition=IfCondition(PythonExpression(["'", dynamics, "'.lower() == 'true'"])))

    # 2-4. Kinematics nodes and GUI (plain ROS 2 Python, no mujoco needed).
    fk = script("fk_node.py")
    ik = script("ik_node.py")
    gui = script("gui_node.py")

    # Closing the viewer or the GUI window stops the whole system.
    shutdown_on_exit = [
        RegisterEventHandler(OnProcessExit(
            target_action=action,
            on_exit=[LogInfo(msg=f"{label} closed, shutting down."), EmitEvent(event=Shutdown())]))
        for action, label in [(bridge_kinematic, "MuJoCo viewer"),
                              (bridge_dynamic, "MuJoCo viewer"),
                              (gui, "GUI")]
    ]

    return LaunchDescription(args + [bridge_kinematic, bridge_dynamic, fk, ik, gui] + shutdown_on_exit)
