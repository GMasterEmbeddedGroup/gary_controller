#pragma once

#include "rclcpp/rclcpp.hpp"
#include "controller_interface/controller_interface.hpp"
#include "realtime_tools/realtime_publisher.h"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include <chrono>


using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
using namespace std::chrono;


namespace gary_controller {

class HardwareMonitor : public controller_interface::ControllerInterface {

public:

    HardwareMonitor();


    CallbackReturn on_init() override{ return CallbackReturn::SUCCESS; };

    controller_interface::return_type init(const std::string &controller_name,
                                           const std::string &namespace_,
                                           const rclcpp::NodeOptions &node_options) override;


    controller_interface::InterfaceConfiguration command_interface_configuration() const override;


    controller_interface::InterfaceConfiguration state_interface_configuration() const override;


    CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;


    CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;


    CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;


    controller_interface::return_type update(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
    //params
    std::string offline_interface_name;
    std::string temperature_interface_name;
    double overheat_threshold;
    std::string diagnose_topic;
    double pub_rate;
    //publisher
    std::unique_ptr<realtime_tools::RealtimePublisher<diagnostic_msgs::msg::DiagnosticArray>> publisher;

    rclcpp::Time last_time;
    bool flag_publish;
};
}