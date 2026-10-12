import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    directory = get_package_share_directory('Robotics_Project')                                          # Gets relative path of Robotics_Project package    
    
    xmlScenePath = os.path.join(directory, 'model', 'scene.xml')
    
    if not os.path.exists(xmlScenePath):
        raise FileNotFoundError(f"Scene file does not exist: {xmlScenePath}.")

    # 1. MuJoCo Simulation Node
    mujoco = Node(
        package    = "Robotics_Project",
        executable = "mujoco_node",
        output     = "screen",
        arguments  = [xmlScenePath],
        parameters = [   
                        {"joint_state_topic_name" : "joint_state"},
                        {"joint_command_topic_name" : "joint_commands"},
                        {"control_mode" : "POSITION"},                      # Set to POSITION for direct angle control
                        {"simulation_frequency" : 1000},
                        {"visualisation_frequency" : 20},
                        {"camera_focal_point": [0.0, 0.0, 0.25]},
                        {"camera_distance": 2.5},
                        {"camera_azimuth": 135.0},
                        {"camera_elevation": -20.0},
                        {"camera_orthographic": False}
                     ]
    )

    # 2. GUI Control Node (Sliders / Constants / Signal Builder)
    gui = Node(
        package    = "Robotics_Project",
        executable = "gui_node.py",
        output     = "screen"
    )

    return LaunchDescription([mujoco, gui])
