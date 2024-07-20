#pragma once

// #include <boost/scoped_ptr.hpp>
#include <chrono>
#include <hardware_interface/system_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <thread>
#include "entities.h"

using namespace hardware_interface;

namespace thor_hardware_interface
{
    class THORHardwareInterface : public hardware_interface::SystemInterface
    {
    public:
        RCLCPP_SHARED_PTR_DEFINITIONS(THORHardwareInterface);

        hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo &info) override;
        std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
        std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
        hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;
        hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;
        hardware_interface::return_type read(const rclcpp::Time &time, const rclcpp::Duration &period) override;
        hardware_interface::return_type write(const rclcpp::Time &time, const rclcpp::Duration &period) override;

    private:
        rclcpp::Logger logger_ = rclcpp::get_logger("thor_hardware_interface");
        rclcpp::Clock clock_ = rclcpp::Clock(RCL_ROS_TIME);

        std::vector<double> actuator_commands_;
        std::vector<double> actuator_positions_;

        std::vector<double> joint_offsets_;
        std::vector<double> joint_positions_;
        std::vector<double> joint_position_commands_;

        ModbusClient*   m_modbusClient;
        Stepper*        m_stepper1;
        Stepper*        m_stepper2;
        Stepper*        m_stepper3;
        Stepper*        m_stepper4;
        Stepper*        m_stepper5;   
        Stepper*        m_stepper6;
        Servo*          m_servo;
        SteppersGroup*  m_group;

        const float k1 = 5.0f;
        const float k2 = 6.0f;
        const float k3 = 30.0f;
        const float k4 = 2.0f;
        const float k5 = 2.0f;
        const float k6 = 4.0f;

        void init_variables();
        double degToRad(double deg) { return deg / 180.0 * M_PI; };
        double radToDeg(double rad) { return rad / M_PI * 180.0; };

        float m_servo_arm_length = 0.06;
        float m_zero_deg_offset = 0;
        
        // 0.06 м - крайнее положение серво, используется, т.к. модель и реальный робот
        // работают в противоход
        double angular_to_linear_pos(int angular_pos) {
            return 0.06 - m_servo_arm_length *
                sin(angular_pos * M_PI / 180);
        };

        float linear_to_angular_pos(double linear_pos) {
            return asin((0.06 - linear_pos) / m_servo_arm_length);
        };
    };
}
