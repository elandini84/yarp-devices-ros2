#include <algorithm>
#include <cmath>
#include <iterator>
#include <limits>
#include <set>

#include "cb_hw_test/cb_hw_test.hpp"

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rcutils/logging_macros.h"

namespace cb_hw_test
{

CbHwTest::CbHwTest()
{
}

CbHwTest::~CbHwTest()
{
}

hardware_interface::CallbackReturn CbHwTest::on_init(const hardware_interface::HardwareInfo & info)
{
    if(info.hardware_parameters.count("node_name")<=0)
    {
        RCLCPP_FATAL(rclcpp::get_logger("CbHwTest"),"No node name specified");
        return hardware_interface::CallbackReturn::ERROR;
    }
    if(info.hardware_parameters.count("cb_nws_msgs_name")<=0)
    {
        RCLCPP_FATAL(rclcpp::get_logger("CbHwTest"),"No msgs name for the controlBoard_nws_ros2 specified");
        return hardware_interface::CallbackReturn::ERROR;
    }

    m_nodeName = info.hardware_parameters["node_name"];
    m_msgs_name = info.hardware_parameters["cb_nws_msgs_name"];

    m_node = NodeCreator::createNode(m_nodeName);

    // Initialize topics and services names ------------------------------------------------------------------- //
    m_posTopicName = m_msgs_name+"/position";
    m_velTopicName = m_msgs_name+"/velocity";
    m_getModesClientName = m_msgs_name+"/get_modes";
    m_setModesClientName = m_msgs_name+"/set_modes";
    m_getVelocityClientName = m_msgs_name+"/get_velocity";
    m_getPositionClientName = m_msgs_name+"/get_position";
    m_getAvailableModesClientName = m_msgs_name+"/get_available_modes";
    m_getJointsNamesClientName = m_msgs_name+"/get_joints_names";

    // Initialize publishers ---------------------------------------------------------------------------------- //
    m_posPublisher = m_node->create_publisher<yarp_control_msgs::msg::Position>(m_posTopicName, 10);
    if(!m_posPublisher){
        RCLCPP_ERROR(m_node->get_logger(),"Could not initialize the Position publisher");
        return hardware_interface::CallbackReturn::ERROR;
    }
    m_velPublisher = m_node->create_publisher<yarp_control_msgs::msg::Velocity>(m_velTopicName, 10);
    if(!m_velPublisher){
        RCLCPP_ERROR(m_node->get_logger(),"Could not initialize the Velocity publisher");
        return hardware_interface::CallbackReturn::ERROR;
    }

    // Initialize services clients ---------------------------------------------------------------------------- //
    m_getJointsNamesClient = m_node->create_client<yarp_control_msgs::srv::GetJointsNames>(m_getJointsNamesClientName);
    if(!m_getJointsNamesClient){
        RCLCPP_ERROR(m_node->get_logger(),"Could not initialize the GetJointsNames service client");
        return hardware_interface::CallbackReturn::ERROR;
    }
    m_getControlModesClient = m_node->create_client<yarp_control_msgs::srv::GetControlModes>(m_getModesClientName);
    if(!m_getControlModesClient){
        RCLCPP_ERROR(m_node->get_logger(),"Could not initialize the GetControlModes service client");
        return hardware_interface::CallbackReturn::ERROR;
    }
    m_getPositionClient = m_node->create_client<yarp_control_msgs::srv::GetPosition>(m_getPositionClientName);
    if(!m_getPositionClient){
        RCLCPP_ERROR(m_node->get_logger(),"Could not initialize the GetPosition service client");
        return hardware_interface::CallbackReturn::ERROR;
    }
    m_getVelocityClient = m_node->create_client<yarp_control_msgs::srv::GetVelocity>(m_getVelocityClientName);
    if(!m_getVelocityClient){
        RCLCPP_ERROR(m_node->get_logger(),"Could not initialize the GetVelocity service client");
        return hardware_interface::CallbackReturn::ERROR;
    }
    m_setControlModesClient = m_node->create_client<yarp_control_msgs::srv::SetControlModes>(m_setModesClientName);
    if(!m_setControlModesClient){
        RCLCPP_ERROR(m_node->get_logger(),"Could not initialize the SetControlModes service client");
        return hardware_interface::CallbackReturn::ERROR;
    }
    m_getAvailableModesClient = m_node->create_client<yarp_control_msgs::srv::GetAvailableControlModes>(m_getAvailableModesClientName);
    if(!m_getAvailableModesClient){
        RCLCPP_ERROR(m_node->get_logger(),"Could not initialize the GetAvailableControlModes service client");
        return hardware_interface::CallbackReturn::ERROR;
    }

    /*
     auto request = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
    auto result = client->async_send_request(request);
    // Wait for the result.
    if (rclcpp::spin_until_future_complete(node, result) ==
        rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sum: %ld", result.get()->sum);
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service add_two_ints");
    }
    */

    auto namesRequest = std::make_shared<yarp_control_msgs::srv::GetJointsNames::Request>();
    while (!m_getJointsNamesClient->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(m_node->get_logger(), "Interrupted while waiting for the service. Exiting.");
            return 0;
        }
        RCLCPP_INFO(m_node->get_logger(), "service not available, waiting again...");
    }
    auto namesResponse = m_getJointsNamesClient->async_send_request(namesRequest);
    if(rclcpp::spin_until_future_complete(m_node, namesResponse) == rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_INFO(m_node->get_logger(), "Got joints names");
        m_jointNames = namesResponse->names;
    }
    else {
        RCLCPP_ERROR(m_node->get_logger(),"Failed to get joints names");
    }
}

std::vector<hardware_interface::StateInterface> CbHwTest::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> ifacesToReturn;

    return ifacesToReturn;
}

std::vector<hardware_interface::CommandInterface> CbHwTest::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> ifacesToReturn;

    return ifacesToReturn;
}

hardware_interface::return_type CbHwTest::prepare_command_mode_switch(const std::vector<std::string> & start_interfaces, const std::vector<std::string> & stop_interfaces)
{
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type CbHwTest::perform_command_mode_switch(const std::vector<std::string> & start_interfaces, const std::vector<std::string> & stop_interfaces)
{
    return hardware_interface::return_type::OK;
}

hardware_interface::CallbackReturn CbHwTest::on_activate(const rclcpp_lifecycle::State & previous_state)
{
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn CbHwTest::on_deactivate(const rclcpp_lifecycle::State & previous_state)
{
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type CbHwTest::read(const rclcpp::Time & time, const rclcpp::Duration & period)
{
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type CbHwTest::write(const rclcpp::Time & time, const rclcpp::Duration & period)
{
    return hardware_interface::return_type::OK;
}

}  // namespace cb_hw_test

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(cb_hw_test::CbHwTest, hardware_interface::SystemInterface)
