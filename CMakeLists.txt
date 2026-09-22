cmake_minimum_required(VERSION 3.20)
project(Robotics_Project)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


################################################################### DEPENDENCIES ##############################################################
find_package(ament_cmake REQUIRED)                                                                  # So ROS2 can compile?
find_package(glfw3 REQUIRED)                                                                        # Graphics Library FrameWork
find_package(rclcpp REQUIRED)                                                                       # ROS2 C++ libraries
find_package(sensor_msgs REQUIRED)                                                                  # sensor_msgs::msg::JointState
find_package(std_msgs REQUIRED)

# MuJoCo
set(MUJOCO_PATH "/opt/mujoco/mujoco-3.13.0") # UPDATE THIS TO YOUR MUJOCO PATH
include_directories(include)                                                                        # Local header files
include_directories(${MUJOCO_PATH}/include/mujoco)                                                  # MuJoCo header files
include_directories(${MUJOCO_PATH}/include)                                                         # Additional MuJoCo header files
link_directories(${MUJOCO_PATH}/lib)                                                                # Location of MuJoCo libraries
###############################################################################################################################################


################################################################## EXECUTABLES ################################################################
add_executable(mujoco_node 		src/nodes/mujoco_node.cpp
                           		src/mujoco_ros.cpp)                                          # Define new executable, list source files
    
target_link_libraries(mujoco_node	mujoco
				  	glfw)                                                        # Link GLFW libraries to the executable
                                     
target_link_libraries(mujoco_node	rclcpp::rclcpp
  					${sensor_msgs_TARGETS}
  					${std_msgs_TARGETS}
)                                                                                                    # List ROS2 dependencies for this executable

###############################################################################################################################################


################################################################### INSTALLATION ##############################################################
install(TARGETS mujoco_node
        DESTINATION lib/${PROJECT_NAME})                                                            # Install so ROS2 can find it

install(DIRECTORY launch
                  model
        DESTINATION share/${PROJECT_NAME})                                                          # Install so ROS2 can find them
                                              
###############################################################################################################################################

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
