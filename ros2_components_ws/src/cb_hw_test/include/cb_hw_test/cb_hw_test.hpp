#ifndef CB_HW_TEST__CB_HW_TEST_HPP_
#define CB_HW_TEST__CB_HW_TEST_HPP_

#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include <rclcpp/rclcpp.hpp>

//Custom ros2 interfaces
#include <yarp_control_msgs/srv/get_control_modes.hpp>
#include <yarp_control_msgs/srv/get_position.hpp>
#include <yarp_control_msgs/srv/get_velocity.hpp>
#include <yarp_control_msgs/srv/set_control_modes.hpp>
#include <yarp_control_msgs/srv/get_available_control_modes.hpp>
#include <yarp_control_msgs/srv/get_joints_names.hpp>
#include <yarp_control_msgs/msg/position.hpp>
#include <yarp_control_msgs/msg/velocity.hpp>
#include <yarp_control_msgs/msg/position_direct.hpp>

#include <mutex>

#include "cb_hw_test/visibility_control.h"

namespace cb_hw_test {

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class CbHwTest : public hardware_interface::SystemInterface
{
private:
    std::string  m_nodeName;             // name of the rosNode
    std::string  m_jointStateTopicName;  // name of the rosTopic
    std::string  m_msgs_name;
    std::string  m_posTopicName;
    std::string  m_velTopicName;
    std::string  m_getModesClientName;
    std::string  m_getPositionClientName;
    std::string  m_getVelocityClientName;
    std::string  m_setModesClientName;
    std::string  m_getJointsNamesClientName;
    std::string  m_getAvailableModesClientName;
    mutable std::mutex       m_cmdMutex;
    std::vector<std::string> m_jointNames; // name of the joints

    // Ros2 related attributes
    Ros2Spinner*            m_spinner{nullptr};
    rclcpp::Node::SharedPtr m_node;
    rclcpp::Pubisher<yarp_control_msgs::msg::Position>::SharedPtr               m_posPublisher;
    rclcpp::Publisher<yarp_control_msgs::msg::Velocity>::SharedPtr              m_velPublisher;
    rclcpp::Client<yarp_control_msgs::srv::GetJointsNames>::SharedPtr           m_getJointsNamesClient;
    rclcpp::Client<yarp_control_msgs::srv::GetControlModes>::SharedPtr          m_getControlModesClient;
    rclcpp::Client<yarp_control_msgs::srv::GetPosition>::SharedPtr              m_getPositionClient;
    rclcpp::Client<yarp_control_msgs::srv::GetVelocity>::SharedPtr              m_getVelocityClient;
    rclcpp::Client<yarp_control_msgs::srv::SetControlModes>::SharedPtr          m_setControlModesClient;
    rclcpp::Client<yarp_control_msgs::srv::GetAvailableControlModes>::SharedPtr m_getAvailableModesClient;



public:
    RCLCPP_SHARED_PTR_DEFINITIONS(CbHwTest)
    CbHwTest();
    ~CbHwTest();

    // SystemInterface virtual functions
    CB_HW_TEST_PUBLIC
    CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

    CB_HW_TEST_PUBLIC
    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

    CB_HW_TEST_PUBLIC
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    CB_HW_TEST_PUBLIC
    hardware_interface::return_type prepare_command_mode_switch(const std::vector<std::string> & start_interfaces, const std::vector<std::string> & stop_interfaces) override;

    CB_HW_TEST_PUBLIC
    return_type perform_command_mode_switch(const std::vector<std::string> & /*start_interfaces*/, const std::vector<std::string> & /*stop_interfaces*/) override;

    CB_HW_TEST_PUBLIC
    hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

    CB_HW_TEST_PUBLIC
    hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

    CB_HW_TEST_PUBLIC
    return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;

    CB_HW_TEST_PUBLIC
    return_type write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override;

};

}  // namespace cb_hw_test

#endif  // CB_HW_TEST__CB_HW_INTERFACE_HPP_
