#include <ar_hardware_interface/ar_hardware_interface.hpp>
#include <sstream>


namespace ar_hardware_interface
{

  hardware_interface::CallbackReturn ARHardwareInterface::on_init(const hardware_interface::HardwareInfo &info)
  {
    RCLCPP_INFO(logger_, "Initializing hardware interface...");

    if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
    {
      return hardware_interface::CallbackReturn::ERROR;
    }

    info_ = info;
    init_variables();

    // НАСТРОЙКА ДРАЙВЕРА!!!!
    // init motor driver
    std::string serial_port = info_.hardware_parameters.at("serial_port");
    RCLCPP_INFO(logger_, "serial port %s", serial_port.c_str());
    m_modbusClient = new ModbusClient("/dev/ttyUSB0");

    std::this_thread::sleep_for(std::chrono::microseconds(3000));

    m_stepper1 = new Stepper(0, m_modbusClient);
    m_stepper2 = new Stepper(1, m_modbusClient);
    m_stepper3 = new Stepper(2, m_modbusClient);
    m_stepper4 = new Stepper(3, m_modbusClient);
    m_stepper5 = new Stepper(4, m_modbusClient);   
    m_stepper6 = new Stepper(5, m_modbusClient);
    m_servo = new Servo(m_modbusClient);
    Stepper m_steppers[] = {*m_stepper1, *m_stepper2, *m_stepper3, *m_stepper4, *m_stepper5, *m_stepper6};
    m_group = new SteppersGroup(m_modbusClient, m_steppers);
    m_group->setMaxSpeedAll(2 * M_PI);
    m_group->setAccelerationAll(20*M_PI);

    m_servo = new Servo(m_modbusClient);
    m_servo->setMaxSpeed(1);
    m_servo->setAcceleration(1);

    // Пауза для инициализации контроллера Arduino
    std::this_thread::sleep_for(std::chrono::seconds(4));

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  // НАСТРОЙКА ПЕРЕМЕННЫХ
  void ARHardwareInterface::init_variables()
  {
    // resize vectors
    int num_joints = info_.joints.size();
    std::cout << "NUM " << num_joints << std::endl;
    actuator_commands_.resize(num_joints);
    actuator_positions_.resize(num_joints);
    joint_positions_.resize(num_joints);
    joint_velocities_.resize(num_joints);
    joint_efforts_.resize(num_joints);
    joint_position_commands_.resize(num_joints);
    joint_velocity_commands_.resize(num_joints);
    joint_effort_commands_.resize(num_joints);
    joint_offsets_.resize(num_joints);
    for (int i = 0; i < num_joints; ++i)
    {
      joint_offsets_[i] =
          std::stod(info_.joints[i].parameters["position_offset"]);
    }
  }

  hardware_interface::CallbackReturn ARHardwareInterface::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
  {
    RCLCPP_INFO(logger_, "Activating hardware interface...");

    // calibrate joints if needed
    bool calibrate = info_.hardware_parameters.at("calibrate") == "True";
    if (calibrate)
    {
      // run calibration
      RCLCPP_INFO(logger_, "Running joint calibration...");
      // driver_.calibrateJoints();
    }

    // init position commands at current positions
    // driver_.getJointPositions(actuator_positions_);
    for (size_t i = 0; i < info_.joints.size(); ++i)
    {
      // apply offsets, convert from deg to rad for moveit
      joint_positions_[i] = degToRad(actuator_positions_[i] + joint_offsets_[i]);
      joint_position_commands_[i] = joint_positions_[i];
    }

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn ARHardwareInterface::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
  {
    RCLCPP_INFO(logger_, "Deactivating hardware interface...");
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  std::vector<hardware_interface::StateInterface> ARHardwareInterface::export_state_interfaces()
  {
    std::vector<hardware_interface::StateInterface> state_interfaces;

    for (size_t i = 0; i < info_.joints.size(); ++i)
    {
      state_interfaces.emplace_back(info_.joints[i].name, "position", &joint_positions_[i]);
    }
    state_interfaces.emplace_back(info_.joints[6].name, "velocity", &joint_velocities_[6]);
    return state_interfaces;
  }

  std::vector<hardware_interface::CommandInterface> ARHardwareInterface::export_command_interfaces()
  {
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    for (size_t i = 0; i < info_.joints.size(); ++i)
    {
      command_interfaces.emplace_back(info_.joints[i].name, "position",
                                      &joint_position_commands_[i]);
    }
    //command_interfaces.emplace_back(info_.joints[6].name, "velocity",
    //                                  &joint_position_commands_[i]);
    return command_interfaces;
  }

  hardware_interface::return_type ARHardwareInterface::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
  {
    // Чтение положения узлов
    float *position = m_group->getCurrentPositionAll();
    //for (int i = 0; i < STEPPERS_COUNT; i++)
    //{
        //std::cout << position[i] << std::endl;
        joint_positions_[0] = degToRad(position[0])/k1;
        joint_positions_[1] = degToRad(position[1])/k2;
        joint_positions_[2] = degToRad(position[2])/k3;
        joint_positions_[3] = degToRad(position[3])/k4;
        joint_positions_[4] = degToRad(position[4])/k5;
        joint_positions_[5] = degToRad(position[5])/k6;
        float servo_pos = m_servo->getCurrentPosition();
        joint_positions_[6] = angular_to_linear_pos(servo_pos);

        std::cout << "RX " << servo_pos << " " << joint_positions_[6] << std::endl;
    //}
    
    
    /*
    std::cout << "RX " << joint_positions_[0] << " " << 
                        joint_positions_[1] << " " << 
                        joint_positions_[2] << " " <<  
                        joint_positions_[3] << " " << 
                        joint_positions_[4] << " " << 
                        joint_positions_[5] << " " <<
                        joint_positions_[6] << std::endl;
                        */
    //std::cout << info_.joints[6].name << std::endl;
                        
                        

    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type ARHardwareInterface::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
  {

    for (size_t i = 0; i < info_.joints.size(); ++i)
    {
      // convert from rad to deg, apply offsets
      actuator_commands_[i] = joint_position_commands_[i] - joint_offsets_[i];
    }
    std::string logInfo = "Joint Cmd: ";
    for (size_t i = 0; i < info_.joints.size(); i++)
    {
      std::stringstream jointPositionStm;
      jointPositionStm << std::fixed << std::setprecision(2)
                       << radToDeg(joint_position_commands_[i]);
      logInfo += info_.joints[i].name + ": " + jointPositionStm.str() + " | ";
    }
    RCLCPP_DEBUG_THROTTLE(logger_, clock_, 500, logInfo.c_str());

    //actuator_commands_[4] = 0;
    //actuator_commands_[5] = 0; 

    
    
    /*std::cout << "TX " << actuator_commands_[0] << " " << 
                          actuator_commands_[1] << " " << 
                          actuator_commands_[2] << " " <<  
                          actuator_commands_[3] << " " << 
                          actuator_commands_[4] << " " << 
                          actuator_commands_[5] << " " <<
                          actuator_commands_[6] << std::endl;*/
    
                          
                        

    m_group->rotateAll(
                        k1*actuator_commands_[0], 
                        k2*actuator_commands_[1], 
                        k3*actuator_commands_[2], 
                        k4*actuator_commands_[3], 
                        k5*actuator_commands_[4], 
                        k6*actuator_commands_[5]
                      );

    //std::cout << "TX " << actuator_commands_[6] << " " << linear_to_angular_pos(actuator_commands_[6]) << std::endl;
    m_servo->rotate(linear_to_angular_pos(actuator_commands_[6]));

    return hardware_interface::return_type::OK;
  }

} // namespace ar_hardware_interface

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(ar_hardware_interface::ARHardwareInterface, hardware_interface::SystemInterface)
