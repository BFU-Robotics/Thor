#include <thor_hardware_interface/thor_hardware_interface.hpp>
#include <sstream>


namespace thor_hardware_interface
{

  hardware_interface::CallbackReturn THORHardwareInterface::on_init(const hardware_interface::HardwareInfo &info)
  {
    RCLCPP_INFO(logger_, "Initializing hardware interface...");

    if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
    {
      return hardware_interface::CallbackReturn::ERROR;
    }

    info_ = info;
    init_variables();

    // Настройка последовательного порта (см. launch-файл)
    std::string serial_port = info_.hardware_parameters.at("serial_port");
    RCLCPP_INFO(logger_, "Последовательный порт %s", serial_port.c_str());
    m_modbusClient = new ModbusClient(serial_port.c_str());

    // Пауза для инициализации миконтроллера
    std::this_thread::sleep_for(std::chrono::seconds(3));

    // Настройка шаговых двигателей
    m_stepper1 = new Stepper(0, m_modbusClient);
    m_stepper2 = new Stepper(1, m_modbusClient);
    m_stepper3 = new Stepper(2, m_modbusClient);
    m_stepper4 = new Stepper(3, m_modbusClient);
    m_stepper5 = new Stepper(4, m_modbusClient);   
    m_stepper6 = new Stepper(5, m_modbusClient);
    m_servo = new Servo(m_modbusClient);
    Stepper m_steppers[] = {*m_stepper1, *m_stepper2, *m_stepper3, *m_stepper4, *m_stepper5, *m_stepper6};
    m_group = new SteppersGroup(m_modbusClient, m_steppers);
    m_group->setMaxSpeedAll(10 * M_PI);
    m_group->setAccelerationAll(20 * M_PI);

    // Настройка серво
    m_servo = new Servo(m_modbusClient);
    m_servo->setMaxSpeed(1);
    m_servo->setAcceleration(1);

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  void THORHardwareInterface::init_variables()
  {
    // Resize векторов по кол-ву шарниров
    int num_joints = info_.joints.size();
    actuator_commands_.resize(num_joints);
    actuator_positions_.resize(num_joints);
    joint_positions_.resize(num_joints);
    joint_position_commands_.resize(num_joints);
    
    // Чтение оффсетов из файла config/joint_offsets.yaml
    joint_offsets_.resize(num_joints);
    for (int i = 0; i < num_joints; ++i)
    {
      joint_offsets_[i] = std::stod(info_.joints[i].parameters["position_offset"]);
    }
  }

  // Включение интерфейса
  hardware_interface::CallbackReturn THORHardwareInterface::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
  {
    RCLCPP_INFO(logger_, "Activating hardware interface...");

    // TODO Добавить калибровку
    bool calibrate = info_.hardware_parameters.at("calibrate") == "True";
    if (calibrate)
    {
      RCLCPP_INFO(logger_, "Running joint calibration...");
    }

    // Команды привести в соответствие с текущими положениями узлов
    for (size_t i = 0; i < info_.joints.size(); ++i)
    {
      joint_positions_[i] = degToRad(actuator_positions_[i] + joint_offsets_[i]);
      joint_position_commands_[i] = joint_positions_[i];
    }

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  // Отключение интерфейса
  hardware_interface::CallbackReturn THORHardwareInterface::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
  {
    RCLCPP_INFO(logger_, "Deactivating hardware interface...");
    
    // Pass
    
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  // Экспорт интерфейса состояния шарниров (их ищет JointStateController)
  std::vector<hardware_interface::StateInterface> THORHardwareInterface::export_state_interfaces()
  {
    std::vector<hardware_interface::StateInterface> state_interfaces;

    for (size_t i = 0; i < info_.joints.size(); ++i)
    {
      state_interfaces.emplace_back(info_.joints[i].name, "position", &joint_positions_[i]);
    }
    state_interfaces.emplace_back(info_.joints[6].name, "velocity", &joint_positions_[6]);

    return state_interfaces;
  }

  // Экспорт интерфейса команд шарниров (их ищет JointTrajectoryController и GripperController)
  std::vector<hardware_interface::CommandInterface> THORHardwareInterface::export_command_interfaces()
  {
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    for (size_t i = 0; i < info_.joints.size(); ++i)
    {
      command_interfaces.emplace_back(info_.joints[i].name, "position", &joint_position_commands_[i]);
    }
    return command_interfaces;
  }

  // Чтение состояния шарниров из микроконтроллера
  hardware_interface::return_type THORHardwareInterface::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
  {
    // Чтение положения узлов
    float *position = m_group->getCurrentPositionAll();
    // Данные приходят в радианах, переводятся в градусы и делятся на передаточный коэффциент
    joint_positions_[0] = degToRad(position[0])/k1;     
    joint_positions_[1] = degToRad(position[1])/k2;
    joint_positions_[2] = degToRad(position[2])/k3;
    joint_positions_[3] = degToRad(position[3])/k4;
    joint_positions_[4] = degToRad(position[4])/k5;
    joint_positions_[5] = degToRad(position[5])/k6;
    // Данные приходят в градусах, переводятся в метры
    joint_positions_[6] = angular_to_linear_pos(m_servo->getCurrentPosition());
   
    /* 
    // Отладка
    std::cout << "RX " << joint_positions_[0] << " " << 
                        joint_positions_[1] << " " << 
                        joint_positions_[2] << " " <<  
                        joint_positions_[3] << " " << 
                        joint_positions_[4] << " " << 
                        joint_positions_[5] << " " <<
                        joint_positions_[6] << std::endl;
    */
    
    return hardware_interface::return_type::OK;
  }

  // Запись команд в микроконтроллер
  hardware_interface::return_type THORHardwareInterface::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
  {

    for (size_t i = 0; i < info_.joints.size(); ++i)
    {
      actuator_commands_[i] = joint_position_commands_[i] - joint_offsets_[i];
    }
  
    /*
    // Отладка
    std::cout << "TX " << actuator_commands_[0] << " " << 
                          actuator_commands_[1] << " " << 
                          actuator_commands_[2] << " " <<  
                          actuator_commands_[3] << " " << 
                          actuator_commands_[4] << " " << 
                          actuator_commands_[5] << " " <<
                          actuator_commands_[6] << std::endl;
    */
    
    // TODO Сделать отправку данных одной командой
    // Положение шарниров (в радианах + передаточный коэффициент)
    m_group->rotateAll(
                        k1*actuator_commands_[0], 
                        k2*actuator_commands_[1], 
                        k3*actuator_commands_[2], 
                        k4*actuator_commands_[3], 
                        k5*actuator_commands_[4], 
                        k6*actuator_commands_[5]
                      );
    // Положение серво (перевод в радианы)
    m_servo->rotate(linear_to_angular_pos(actuator_commands_[6]));

    return hardware_interface::return_type::OK;
  }

}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(thor_hardware_interface::THORHardwareInterface, hardware_interface::SystemInterface)
