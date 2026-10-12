/**
 * @file   mujoco_ros.cpp
 * @author Jon Woolfrey (Modified with mouse/camera navigation)
 * @email  jonathan.woolfrey@gmail.com
 * @date   April 2025
 * @version 1.2
 * @brief  A class for connecting a MuJoCo simulation with ROS2 communication.
 */
 
#include <mujoco_ros2/mujoco_ros.hpp>

// Static pointers and state for mouse navigation callbacks
static mjModel*   g_model  = nullptr;
static mjvCamera* g_camera = nullptr;

static bool   g_button_left   = false;
static bool   g_button_middle = false;
static bool   g_button_right  = false;
static double g_lastx         = 0;
static double g_lasty         = 0;

// Mouse button callback (tracks which button is pressed)
static void mouse_button_callback(GLFWwindow* window, int button, int action, int mods)
{
    (void)button;
    (void)action;
    (void)mods;
    g_button_left   = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT)   == GLFW_PRESS);
    g_button_middle = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS);
    g_button_right  = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT)  == GLFW_PRESS);
    glfwGetCursorPos(window, &g_lastx, &g_lasty);
}

// Mouse motion callback (rotate, pan, zoom)
static void mouse_move_callback(GLFWwindow* window, double xpos, double ypos)
{
    if (!g_button_left && !g_button_middle && !g_button_right) return;
    if (!g_model || !g_camera) return;

    double dx = xpos - g_lastx;
    double dy = ypos - g_lasty;
    g_lastx = xpos;
    g_lasty = ypos;

    int width, height;
    glfwGetWindowSize(window, &width, &height);

    bool mod_shift = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT)  == GLFW_PRESS ||
                      glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS);

    mjtMouse action;
    if (g_button_right)
    {
        action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
    }
    else if (g_button_left)
    {
        action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
    }
    else
    {
        action = mjMOUSE_ZOOM;
    }

    mjv_moveCamera(g_model, action, dx / height, dy / height, g_camera);
}

// Mouse scroll callback (zoom in / out)
static void scroll_callback(GLFWwindow* window, double xoffset, double yoffset)
{
    (void)window;
    (void)xoffset;
    if (!g_model || !g_camera) return;

    mjv_moveCamera(g_model, mjMOUSE_ZOOM, 0.0, -0.05 * yoffset, g_camera);
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                         Constructor                                            //
////////////////////////////////////////////////////////////////////////////////////////////////////
MuJoCoROS::MuJoCoROS(const std::string &xmlLocation) : Node("mujoco_node")
{
    // Declare & get parameters for this node
    std::string jointStateTopicName   = this->declare_parameter<std::string>("joint_state_topic_name", "joint_state");
    std::string jointCommandTopicName = this->declare_parameter<std::string>("joint_command_topic_name", "joint_commands");
    std::string controlMode           = this->declare_parameter<std::string>("control_mode", "TORQUE");
    _simFrequency                     = this->declare_parameter<int>("simulation_frequency", 1000);
    int visualisationFrequency        = this->declare_parameter<int>("visualisation_frequency", 20);

    // Load the robot model     
    char errorMessage[1000] = "Could not load model.";
    _model = mj_loadXML(xmlLocation.c_str(), nullptr, errorMessage, 1000);
    if (not _model) throw std::runtime_error("[ERROR] [MuJoCo NODE] Problem loading model: " + std::string(errorMessage)); 
    _model->opt.timestep = 1/((double)_simFrequency);
    
    // Resize arrays based on the number of joints in the model
    _jointState = mj_makeData(_model);
    _jointStateMessage.name.resize(_model->nq);
    _jointStateMessage.position.resize(_model->nq);
    _jointStateMessage.velocity.resize(_model->nq);
    _jointStateMessage.effort.resize(_model->nq);    
    _torqueInput.resize(_model->nq, 0.0);

    // Record joint names
    for (int i = 0; i < _model->nq; ++i) _jointStateMessage.name[i] = mj_id2name(_model, mjOBJ_JOINT, i);
       
    // Set the control mode  
         if (controlMode == "POSITION") _controlMode = POSITION;
    else if (controlMode == "VELOCITY") _controlMode = VELOCITY;
    else if (controlMode == "TORQUE"  ) _controlMode = TORQUE;
    else
    {   
        throw std::invalid_argument("[ERROR] [MuJoCo NODE] Unknown control mode. "
                                    "Argument was '" + controlMode + "', but expected 'POSITION', 'VELOCITY', or 'TORQUE'.");
    }

    // Create timers
    _simTimer = this->create_wall_timer(std::chrono::milliseconds(static_cast<int>(1000/_simFrequency)),
                                        std::bind(&MuJoCoROS::update_simulation, this));
                                        
    _visTimer = this->create_wall_timer(std::chrono::milliseconds(static_cast<int>(1000/visualisationFrequency)),
                                        std::bind(&MuJoCoROS::update_visualization, this));

    // Create joint state publisher and joint command subscriber
    _jointCommandSubscriber = this->create_subscription<std_msgs::msg::Float64MultiArray>(jointCommandTopicName, 1,  std::bind(&MuJoCoROS::joint_command_callback, this, std::placeholders::_1));
    _jointStatePublisher = this->create_publisher<sensor_msgs::msg::JointState>(jointStateTopicName, 1);
                      
    // Initialize Graphics Library FrameWork (GLFW)
    if (not glfwInit()) throw std::runtime_error("Failed to initialise Graphics Library Framework (GLFW).");

    _window = glfwCreateWindow(1200, 900, "MuJoCo Visualization", nullptr, nullptr);
    if (not _window) throw std::runtime_error("Failed to create Graphics Library Framework (GLFW) window.");
    
    // Make the OpenGL context current
    glfwMakeContextCurrent(_window);
    glfwSwapInterval(1);

    // Initialize MuJoCo rendering context
    mjv_defaultCamera(&_camera);
    mjv_defaultOption(&_renderingOptions);
    _renderingOptions.frame = mjFRAME_SITE;
    mjv_defaultPerturb(&_perturbation);
    mjr_defaultContext(&_context);
    mjv_makeScene(_model, &_scene, 1000);
    
    // Declare & get parameters for camera, visualisation
    _camera.azimuth      = this->declare_parameter<double>("camera_azimuth", 135);    
    _camera.distance     = this->declare_parameter<double>("camera_distance", 2.5);
    _camera.elevation    = this->declare_parameter<double>("camera_elevation", -35);
    _camera.orthographic = this->declare_parameter<bool>("camera_orthographic", false);
    
    auto focalPoint = this->declare_parameter<std::vector<double>>("camera_focal_point", {0.0, 0.0, 0.5});
    for(int i = 0; i < 3; ++i) _camera.lookat[i] = focalPoint[i]; 

    // Connect mouse callbacks to window
    g_model  = _model;
    g_camera = &_camera;

    glfwSetMouseButtonCallback(_window, mouse_button_callback);
    glfwSetCursorPosCallback(_window, mouse_move_callback);
    glfwSetScrollCallback(_window, scroll_callback);

    // Create MuJoCo rendering context
    glfwMakeContextCurrent(_window);
    mjr_makeContext(_model, &_context, mjFONTSCALE_100);
    
    RCLCPP_INFO(this->get_logger(), "MuJoCo simulation initiated. "
                                    "Publishing the joint state to '%s' topic. "
                                    "Subscribing to joint %s commands via '%s' topic.",
                                    jointStateTopicName.c_str(), controlMode.c_str(), jointCommandTopicName.c_str());
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                    Update the simulation                                       //
////////////////////////////////////////////////////////////////////////////////////////////////////
void MuJoCoROS::update_simulation()
{
    if (not _model and not _jointState)
    {
        RCLCPP_ERROR(this->get_logger(), "MuJoCo model or data is not initialized. How did that happen?");
        return;
    }

    if (_controlMode == TORQUE)
    {
        for (int i = 0; i < _model->nq; ++i)
        {
            _jointState->ctrl[i] = _torqueInput[i]
                                 + _jointState->qfrc_bias[i]
                                 - 0.01*_jointState->qvel[i];
                                 
            _torqueInput[i] = 0.0;
        }
    }

    mj_step(_model, _jointState);

    // Add joint state data to ROS2 message
    for (int i = 0; i < _model->nq; ++i)
    {
        _jointStateMessage.position[i] = _jointState->qpos[i];
        _jointStateMessage.velocity[i] = _jointState->qvel[i];
        _jointStateMessage.effort[i]   = _jointState->actuator_force[i];
    }

    _jointStateMessage.header.stamp = this->get_clock()->now();
    _jointStatePublisher->publish(_jointStateMessage);
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                    Handle joint commands                                       //
////////////////////////////////////////////////////////////////////////////////////////////////////
void MuJoCoROS::joint_command_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
    if (static_cast<int>(msg->data.size()) != _model->nq)
    {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                             "Received joint command with incorrect size.");
        return;
    }
    else
    {
        switch(_controlMode)
        {
            case POSITION:
            {
                for(int i = 0; i < _model->nq; ++i)
                {
                    _jointState->ctrl[i] = msg->data[i];
                }
                break;
            }
            case VELOCITY:
            {
                for(int i = 0; i < _model->nq; ++i)
                {
                    _jointState->ctrl[i] += msg->data[i] / (double)_simFrequency;
                }
                break;
            }
            case TORQUE:
            {
                for(int i = 0; i < _model->nq; ++i)
                {
                    _torqueInput[i] = msg->data[i];
                }
                break;
            }
            default:
            {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                     "Unknown control mode. Unable to set joint commands.");
                break;
            }
        }
    }
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                    Update the 3D simulation                                    //
////////////////////////////////////////////////////////////////////////////////////////////////////
void MuJoCoROS::update_visualization()
{
    glfwMakeContextCurrent(_window);
    
    mjv_updateScene(_model, _jointState, &_renderingOptions, NULL, &_camera, mjCAT_ALL, &_scene);

    int width, height;
    glfwGetFramebufferSize(_window, &width, &height);
    mjrRect viewport = {0, 0, width, height};

    mjr_render(viewport, &_scene, &_context);
    
    glfwSwapBuffers(_window);
    glfwPollEvents();
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                           Destructor                                           //
////////////////////////////////////////////////////////////////////////////////////////////////////
MuJoCoROS::~MuJoCoROS()
{
    mj_deleteData(_jointState);
    mj_deleteModel(_model);
    mjv_freeScene(&_scene);
    mjr_freeContext(&_context);
    glfwDestroyWindow(_window);
    glfwTerminate();
}
