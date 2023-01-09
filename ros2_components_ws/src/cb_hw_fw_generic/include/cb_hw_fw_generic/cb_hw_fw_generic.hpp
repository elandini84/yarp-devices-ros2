#ifndef CB_HW_FW_GENERIC__CB_HW_FW_GENERIC_HPP_
#define CB_HW_FW_GENERIC__CB_HW_FW_GENERIC_HPP_

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

#include "cb_hw_fw_generic/visibility_control.h"

namespace hardware_interface
{
/// Constant defining position direct interface
constexpr char HW_IF_POSDIRECT[] = "position_direct";
/// Constant defining torque interface
constexpr char HW_IF_TORQUE[] = "torque";
/// Constant defining acceleration interface
constexpr char HW_IF_CURRENT[] = "current";
/// Constant defining PWM interface
constexpr char HW_IF_PWM[] = "PWM";
}  // namespace hardware_interface

namespace cb_hw_fw_generic {

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
const std::vector<std::string> HW_IFs{std::string(hardware_interface::HW_IF_POSITION),
                                std::string(hardware_interface::HW_IF_VELOCITY),
                                std::string(hardware_interface::HW_IF_POSDIRECT),
                                std::string(hardware_interface::HW_IF_TORQUE),
                                std::string(hardware_interface::HW_IF_CURRENT),
                                std::string(hardware_interface::HW_IF_PWM),
                                std::string(hardware_interface::HW_IF_EFFORT)};

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class CbHwFwGeneric : public hardware_interface::SystemInterface
{
private:
    bool         m_active{false};
    bool         m_continuousPosWrite{true};
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

    // State and command interfaces
    // Store the commands for the simulated robot
    std::vector<double> m_hwCommandsPositions;
    std::vector<double> m_hwCommandsVelocity;
    std::vector<double> m_hwCommandsDirPositions; // For control mode "position direct"
    std::vector<double> m_hwCommandsTorque; // This could be the same thing as effort so
                                            // only this or "m_hwCommandsEffort"
    std::vector<double> m_hwCommandsEffort;
    std::vector<double> m_hwCommandsCurrent;
    std::vector<double> m_hwCommandsPWM;
    std::vector<double> m_oldPositions; // This array has to be used in order to avoid sending
                                        // the same position commans over and over

    std::vector<std::string> m_modes; // Current joints control modes.
                                      // We have to assume that the joints do not change
                                      // control mode without this hw interface class knowing.
                                      // In other words, the only way to switch control mode
                                      // should be via "perform_command_mode_switch" method.

    // Ros2 related attributes
    rclcpp::Node::SharedPtr m_node;
    rclcpp::Publisher<yarp_control_msgs::msg::Position>::SharedPtr              m_posPublisher;
    rclcpp::Publisher<yarp_control_msgs::msg::Velocity>::SharedPtr              m_velPublisher;
    rclcpp::Client<yarp_control_msgs::srv::GetJointsNames>::SharedPtr           m_getJointsNamesClient;
    rclcpp::Client<yarp_control_msgs::srv::GetControlModes>::SharedPtr          m_getControlModesClient;
    rclcpp::Client<yarp_control_msgs::srv::GetPosition>::SharedPtr              m_getPositionClient;
    rclcpp::Client<yarp_control_msgs::srv::GetVelocity>::SharedPtr              m_getVelocityClient;
    rclcpp::Client<yarp_control_msgs::srv::SetControlModes>::SharedPtr          m_setControlModesClient;
    rclcpp::Client<yarp_control_msgs::srv::GetAvailableControlModes>::SharedPtr m_getAvailableModesClient;

    // Internal functions
    bool _checkJoints(const std::vector<hardware_interface::ComponentInfo>& joints);
    CallbackReturn _initExportableInterfaces(const std::vector<hardware_interface::ComponentInfo>& joints,
                                             bool overwriteMode);
    CallbackReturn _getHWCurrentValues();

public:
    RCLCPP_SHARED_PTR_DEFINITIONS(CbHwFwGeneric)
    CbHwFwGeneric();
    ~CbHwFwGeneric();

    // SystemInterface virtual functions
    CB_HW_FW_GENERIC_PUBLIC
    CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

    CB_HW_FW_GENERIC_PUBLIC
    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

    CB_HW_FW_GENERIC_PUBLIC
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    CB_HW_FW_GENERIC_PUBLIC
    hardware_interface::return_type prepare_command_mode_switch(const std::vector<std::string> & start_interfaces, const std::vector<std::string> & stop_interfaces) override;

    CB_HW_FW_GENERIC_PUBLIC
    hardware_interface::return_type perform_command_mode_switch(const std::vector<std::string> & /*start_interfaces*/, const std::vector<std::string> & /*stop_interfaces*/) override;

    CB_HW_FW_GENERIC_PUBLIC
    hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

    CB_HW_FW_GENERIC_PUBLIC
    hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

    CB_HW_FW_GENERIC_PUBLIC
    hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;

    CB_HW_FW_GENERIC_PUBLIC
    hardware_interface::return_type write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override;

};

}  // namespace cb_hw_fw_generic

#endif  // CB_HW_FW_GENERIC__CB_HW_INTERFACE_HPP_
