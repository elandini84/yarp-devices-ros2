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

#include "cb_hw_test/visibility_control.h"

using hardware_interface::return_type;

namespace cb_hw_test
{

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class CbHwInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(CbHwInterface)
  CbHwInterface();
  ~CbHwInterface();

  CB_HW_TEST_PUBLIC
  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

  CB_HW_TEST_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  CB_HW_TEST_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  CB_HW_TEST_PUBLIC
  return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  CB_HW_TEST_PUBLIC
  return_type write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override
  {
    return return_type::OK;
  }

protected:
  /// Use standard interfaces for joints because they are relevant for dynamic behavior
  /**
   * By splitting the standard interfaces from other type, the users are able to inherit this
   * class and simply create small "simulation" with desired dynamic behavior.
   * The advantage over using Gazebo is that enables "quick & dirty" tests of robot's URDF and
   * controllers.
   */
  const std::vector<std::string> standard_interfaces_ = {
    hardware_interface::HW_IF_POSITION, hardware_interface::HW_IF_VELOCITY,
    hardware_interface::HW_IF_ACCELERATION, hardware_interface::HW_IF_EFFORT};

  const size_t POSITION_INTERFACE_INDEX = 0;

  struct MimicJoint
  {
    std::size_t joint_index;
    std::size_t mimicked_joint_index;
    double multiplier = 1.0;
  };
  std::vector<MimicJoint> mimic_joints_;

  /// The size of this vector is (standard_interfaces_.size() x nr_joints)
  std::vector<std::vector<double>> joint_commands_;
  std::vector<std::vector<double>> joint_states_;

  std::vector<std::string> other_interfaces_;
  /// The size of this vector is (other_interfaces_.size() x nr_joints)
  std::vector<std::vector<double>> other_commands_;
  std::vector<std::vector<double>> other_states_;

  std::vector<std::string> sensor_interfaces_;
  /// The size of this vector is (sensor_interfaces_.size() x nr_joints)
  std::vector<std::vector<double>> sensor_fake_commands_;
  std::vector<std::vector<double>> sensor_states_;

  std::vector<std::string> gpio_interfaces_;
  /// The size of this vector is (gpio_interfaces_.size() x nr_joints)
  std::vector<std::vector<double>> gpio_fake_commands_;
  std::vector<std::vector<double>> gpio_commands_;
  std::vector<std::vector<double>> gpio_states_;

private:
  template <typename HandleType>
  bool get_interface(
    const std::string & name, const std::vector<std::string> & interface_list,
    const std::string & interface_name, const size_t vector_index,
    std::vector<std::vector<double>> & values, std::vector<HandleType> & interfaces);

  void initialize_storage_vectors(
    std::vector<std::vector<double>> & commands, std::vector<std::vector<double>> & states,
    const std::vector<std::string> & interfaces);

  template <typename InterfaceType>
  bool populate_interfaces(
    const std::vector<hardware_interface::ComponentInfo> & components,
    std::vector<std::string> & interfaces, std::vector<std::vector<double>> & storage,
    std::vector<InterfaceType> & target_interfaces, bool using_state_interfaces);

  bool use_fake_gpio_command_interfaces_;
  bool use_fake_sensor_command_interfaces_;

  double position_state_following_offset_;
  std::string custom_interface_with_following_offset_;
  size_t index_custom_interface_with_following_offset_;
};

}  // namespace cb_hw_test

#endif  // CB_HW_TEST__CB_HW_INTERFACE_HPP_
