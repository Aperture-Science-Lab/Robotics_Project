import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    mujoco_pkg = get_package_share_directory('mujoco_ros2')
    
    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(mujoco_pkg, 'launch', 'example.py')
        )
    )
    
    kinematics_node = Node(
        package='panda_kinematics',
        executable='kinematics',
        name='panda_kinematics',
        output='screen'
    )
    
    return LaunchDescription([
        sim_launch,
        kinematics_node
    ])
