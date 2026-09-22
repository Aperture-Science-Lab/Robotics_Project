/**
 * @file   mujoco_node.cpp
 * @author Jon Woolfrey
 * @email  jonathan.woolfrey@gmail.com
 * @date   April 2025
 * @version 1.1
 * @brief  Starts ROS2 and runs the MuJoCoNode.
 * 
 * @details This contains the main() function for the C++ executable.
 *          Its purpose is to start ROS2 and an instance of the MuJoCoNode class.
 * 
 * @copyright Copyright (c) 2025 Jon Woolfrey
 * 
 * @license GNU General Public License V3
 * 
 * @see https://mujoco.org/ for more information about MuJoCo
 * @see https://docs.ros.org/en/humble/index.html for ROS 2 documentation
 */
#include <mujoco_ros2/mujoco_ros.hpp>
#include <iostream>

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);                                                                       // Starts up ROS2
    
    if(argc < 2)
    {
        throw std::invalid_argument("[ERROR] Invalid number of arguments. Usage: mujoco_node path/to/scene.xml");                                  
    }
   
    std::string xmlPath = argv[1];
    
    try
    {
        auto mujocoNode = std::make_shared<MuJoCoROS>(xmlPath);
    
        rclcpp::spin(mujocoNode);                                                                   // Run indefinitely
        
        rclcpp::shutdown();
        
        return 0; 
    }
    catch(const std::exception &exception)
    {
        RCLCPP_ERROR(rclcpp::get_logger("main"), exception.what());
        
        rclcpp::shutdown();                                                                         // Stop ROS2
        
        return 1;                                                                                   // Flag error
    }
}
