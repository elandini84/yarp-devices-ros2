#ifndef CB_HW_IFACE__CB_HW_IFACE_HPP_
#define CB_HW_IFACE__CB_HW_IFACE_HPP_

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
#include <yarp_control_msgs/srv/set_control_modes.hpp>
#include <yarp_control_msgs/srv/get_available_control_modes.hpp>
#include <yarp_control_msgs/srv/get_joints_names.hpp>
#include <yarp_control_msgs/msg/position.hpp>
#include <yarp_control_msgs/msg/velocity.hpp>
#include <yarp_control_msgs/msg/position_direct.hpp>

#include <mutex>

#include "cb_hw_interface/visibility_control.h"

namespace cb_hw_interface {

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class CbHwInterface : public hardware_interface::SystemInterface
{
public:
    RCLCPP_SHARED_PTR_DEFINITIONS(CbHwInterface)
    CbHwInterface();
    ~CbHwInterface();

    // SystemInterface virtual functions
    CB_HW_IFACE_PUBLIC
    CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

    CB_HW_IFACE_PUBLIC
    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

    CB_HW_IFACE_PUBLIC
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    CB_HW_IFACE_PUBLIC
    return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;

    CB_HW_IFACE_PUBLIC
    return_type write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override;

};

} // namespace cb_hw_interface end

#endif  // CB_HW_IFACE__CB_HW_IFACE_HPP_