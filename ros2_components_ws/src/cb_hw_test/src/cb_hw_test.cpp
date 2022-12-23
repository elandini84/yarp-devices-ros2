#include <algorithm>
#include <cmath>
#include <iterator>
#include <limits>
#include <set>
#include <chrono>
#include <cstdlib>
#include <memory>

#include "cb_hw_test/cb_hw_test.hpp"

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rcutils/logging_macros.h"

using namespace std::chrono_literals;

namespace cb_hw_test
{

CbHwTest::CbHwTest()
{
}

CbHwTest::~CbHwTest()
{
}

bool CbHwTest::_checkJoints(const std::vector<hardware_interface::ComponentInfo>& joints)
{
    for(const auto& joint : joints){
        if (std::find(m_jointNames.begin(), m_jointNames.end(), joint.name) == m_jointNames.end())
        {
            RCLCPP_FATAL(m_node->get_logger(),"The joint named %s was not found among the available ones",
                         joint.name.c_str());
            return false;
        }
    }
    return true;
}

CallbackReturn CbHwTest::_initExportableInterfaces(const std::vector<hardware_interface::ComponentInfo>& joints)
{
    if(!_checkJoints(joints))
    {
        RCLCPP_FATAL(m_node->get_logger(),"Unable to initialize the joints. Check the previous errors for more details");
        return CallbackReturn::ERROR;
    }
    m_hwCommandsPositions.resize(joints.size(), std::numeric_limits<double>::quiet_NaN());
    m_hwStatesPositions.resize(joints.size(), std::numeric_limits<double>::quiet_NaN());
    m_hwStatesVelocities.resize(joints.size(), std::numeric_limits<double>::quiet_NaN());
    size_t i=0;

    for (const auto& joint : joints)
    {
        // RRBotSystemPositionOnly has exactly one state and command interface on each joint
        if (joint.command_interfaces.size() != 1)
        {
            RCLCPP_FATAL(
                m_node->get_logger(),
                "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
                joint.command_interfaces.size());
            return CallbackReturn::ERROR;
        }

        if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
        {
            RCLCPP_FATAL(
                m_node->get_logger(),
                "This device supports only POSITION command interfaces. Check again your hardware configuration");
            return CallbackReturn::ERROR;
        }

        if (joint.state_interfaces.size() > 2)
        {
            RCLCPP_FATAL(
                m_node->get_logger(),
                "Joint '%s' has %zu state interface. 2 maximum expected.", joint.name.c_str(),
                joint.state_interfaces.size());
            return CallbackReturn::ERROR;
        }

        if ((joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION &&
            joint.state_interfaces[1].name != hardware_interface::HW_IF_POSITION) ||
            (joint.state_interfaces[0].name != hardware_interface::HW_IF_VELOCITY &&
            joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY) ||
            joint.state_interfaces[0].name == joint.state_interfaces[1].name)
        {
            RCLCPP_FATAL(
                m_node->get_logger(),
                "Joint '%s' has no %s state interface. Check your configuration", joint.name.c_str(),
                hardware_interface::HW_IF_POSITION);
            return CallbackReturn::ERROR;
        }
        if ((joint.state_interfaces[0].name != hardware_interface::HW_IF_VELOCITY &&
            joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY))
        {
            RCLCPP_FATAL(
                m_node->get_logger(),
                "Joint '%s' has no %s state interface. Check your configuration", joint.name.c_str(),
                hardware_interface::HW_IF_VELOCITY);
            return CallbackReturn::ERROR;
        }
        if (joint.state_interfaces[0].name == joint.state_interfaces[1].name)
        {
            RCLCPP_FATAL(
                m_node->get_logger(),
                "Joint '%s' has two equal state interfaces. This is not allowed. Check your configuration",
                joint.name.c_str());
            return CallbackReturn::ERROR;
        }

        m_hwCommandsPositions[i] = 0.0;
        m_hwStatesPositions[i] = 0.0;
        m_hwStatesVelocities[i++] = 0.0;
    }

    return CallbackReturn::SUCCESS;
}

CallbackReturn CbHwTest::on_init(const hardware_interface::HardwareInfo & info)
{
    if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
    {
        return hardware_interface::CallbackReturn::ERROR;
    }
    /* controllare i giunti in ingresso e verificare che siano tra quelli disponibili tramite il client
     * di getJointNames. A quel punto si espongono solo le interfacce di comando provenienti dall'esterno
     * tramite il parametro "info".
     * Le interfacce attive saranno quelle corrispondenti ai control mode dei vari giunti.
     * Come si controlla sta cosa? Boh...
     */
    if(info.hardware_parameters.count("node_name")<=0)
    {
        RCLCPP_FATAL(rclcpp::get_logger("CbHwTest"),"No node name specified");
        return CallbackReturn::ERROR;
    }
    if(info.hardware_parameters.count("cb_nws_msgs_name")<=0)
    {
        RCLCPP_FATAL(rclcpp::get_logger("CbHwTest"),"No msgs name for the controlBoard_nws_ros2 specified");
        return CallbackReturn::ERROR;
    }

    m_nodeName = std::string(info_.hardware_parameters["node_name"].c_str());
    m_msgs_name = info_.hardware_parameters["cb_nws_msgs_name"];

    m_node = rclcpp::Node::make_shared(m_nodeName);

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
        return CallbackReturn::ERROR;
    }
    m_velPublisher = m_node->create_publisher<yarp_control_msgs::msg::Velocity>(m_velTopicName, 10);
    if(!m_velPublisher){
        RCLCPP_ERROR(m_node->get_logger(),"Could not initialize the Velocity publisher");
        return CallbackReturn::ERROR;
    }

    // Initialize services clients ---------------------------------------------------------------------------- //
    m_getJointsNamesClient = m_node->create_client<yarp_control_msgs::srv::GetJointsNames>(m_getJointsNamesClientName);
    if(!m_getJointsNamesClient){
        RCLCPP_ERROR(m_node->get_logger(),"Could not initialize the GetJointsNames service client");
        return CallbackReturn::ERROR;
    }
    m_getControlModesClient = m_node->create_client<yarp_control_msgs::srv::GetControlModes>(m_getModesClientName);
    if(!m_getControlModesClient){
        RCLCPP_ERROR(m_node->get_logger(),"Could not initialize the GetControlModes service client");
        return CallbackReturn::ERROR;
    }
    m_getPositionClient = m_node->create_client<yarp_control_msgs::srv::GetPosition>(m_getPositionClientName);
    if(!m_getPositionClient){
        RCLCPP_ERROR(m_node->get_logger(),"Could not initialize the GetPosition service client");
        return CallbackReturn::ERROR;
    }
    m_getVelocityClient = m_node->create_client<yarp_control_msgs::srv::GetVelocity>(m_getVelocityClientName);
    if(!m_getVelocityClient){
        RCLCPP_ERROR(m_node->get_logger(),"Could not initialize the GetVelocity service client");
        return CallbackReturn::ERROR;
    }
    m_setControlModesClient = m_node->create_client<yarp_control_msgs::srv::SetControlModes>(m_setModesClientName);
    if(!m_setControlModesClient){
        RCLCPP_ERROR(m_node->get_logger(),"Could not initialize the SetControlModes service client");
        return CallbackReturn::ERROR;
    }
    m_getAvailableModesClient = m_node->create_client<yarp_control_msgs::srv::GetAvailableControlModes>(m_getAvailableModesClientName);
    if(!m_getAvailableModesClient){
        RCLCPP_ERROR(m_node->get_logger(),"Could not initialize the GetAvailableControlModes service client");
        return CallbackReturn::ERROR;
    }

    auto namesRequest = std::make_shared<yarp_control_msgs::srv::GetJointsNames::Request>();
    while (!m_getJointsNamesClient->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(m_node->get_logger(), "Interrupted while waiting for the service. Exiting.");
            return CallbackReturn::ERROR;
        }
        RCLCPP_INFO(m_node->get_logger(), "service not available, waiting again...");
    }
    auto namesResponse = m_getJointsNamesClient->async_send_request(namesRequest);
    if(rclcpp::spin_until_future_complete(m_node, namesResponse) == rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_INFO(m_node->get_logger(), "Got joints names");
        m_jointNames = namesResponse.get()->names;
    }
    else {
        RCLCPP_ERROR(m_node->get_logger(),"Failed to get joints names");
        return CallbackReturn::ERROR;
    }

    return _initExportableInterfaces(info_.joints);
}

std::vector<hardware_interface::StateInterface> CbHwTest::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> ifacesToReturn;
    for (size_t i=0; i<m_hwStatesPositions.size(); i++){
        ifacesToReturn.emplace_back(hardware_interface::StateInterface(
            info_.joints[i].name, hardware_interface::HW_IF_POSITION, &m_hwStatesPositions[i]));
        ifacesToReturn.emplace_back(hardware_interface::StateInterface(
            info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &m_hwStatesVelocities[i]));
    }

    return ifacesToReturn;
}

std::vector<hardware_interface::CommandInterface> CbHwTest::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> ifacesToReturn;
    for (size_t i=0; i<m_hwCommandsPositions.size(); i++)
    {
        ifacesToReturn.emplace_back(hardware_interface::CommandInterface(
            info_.joints[i].name, hardware_interface::HW_IF_POSITION, &m_hwCommandsPositions[i]));
    }

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

CallbackReturn CbHwTest::on_activate(const rclcpp_lifecycle::State & previous_state)
{
    return CallbackReturn::SUCCESS;
}

CallbackReturn CbHwTest::on_deactivate(const rclcpp_lifecycle::State & previous_state)
{
    return CallbackReturn::SUCCESS;
}

hardware_interface::return_type CbHwTest::read(const rclcpp::Time & time, const rclcpp::Duration & period)
{
    //std::lock_guard <std::mutex> lg(m_cmdMutex);

    // RCLCPP_INFO(m_node->get_logger(), "There we are %f and %f", time.seconds(), period.seconds());

    // // Acquiring the positions of the robot joint
    // auto positionRequest = std::make_shared<yarp_control_msgs::srv::GetPosition::Request>();
    // while (!m_getPositionClient->wait_for_service(1s))
    // {
    //     if (!rclcpp::ok())
    //     {
    //         RCLCPP_ERROR(m_node->get_logger(), "Interrupted while waiting for the service. Exiting.");
    //         return hardware_interface::return_type::ERROR;
    //     }
    //     RCLCPP_INFO(m_node->get_logger(), "service not available, waiting again...");
    // }
    // positionRequest->names = m_jointNames;
    // auto positionResponse = m_getPositionClient->async_send_request(positionRequest);
    // if(rclcpp::spin_until_future_complete(m_node, positionResponse) == rclcpp::FutureReturnCode::SUCCESS)
    // {
    //     auto positions = positionResponse.get()->positions;
    //     for (size_t i=0; i < positions.size(); i++)
    //     {
    //         m_hwStatesPositions[i] = positions[i];
    //     }
    //     //RCLCPP_INFO(m_node->get_logger(), "Got positions");
    // }
    // else {
    //     RCLCPP_ERROR(m_node->get_logger(),"Failed to get joints positions");
    //     return hardware_interface::return_type::ERROR;
    // }

    // // Acquiring the velocities of the robot joint
    // auto velocityRequest = std::make_shared<yarp_control_msgs::srv::GetVelocity::Request>();
    // while (!m_getVelocityClient->wait_for_service(1s))
    // {
    //     if (!rclcpp::ok())
    //     {
    //         RCLCPP_ERROR(m_node->get_logger(), "Interrupted while waiting for the service. Exiting.");
    //         return hardware_interface::return_type::ERROR;
    //     }
    //     RCLCPP_INFO(m_node->get_logger(), "service not available, waiting again...");
    // }
    // velocityRequest->names = m_jointNames;
    // auto velocityResponse = m_getVelocityClient->async_send_request(velocityRequest);
    // if(rclcpp::spin_until_future_complete(m_node, velocityResponse) == rclcpp::FutureReturnCode::SUCCESS)
    // {
    //     auto velocities = velocityResponse.get()->velocities;
    //     for (size_t i=0; i < velocities.size(); i++)
    //     {
    //         m_hwStatesVelocities[i] = velocities[i];
    //     }
    //     //RCLCPP_INFO(m_node->get_logger(), "Got velocities");
    // }
    // else {
    //     RCLCPP_ERROR(m_node->get_logger(),"Failed to get joints velocities");
    //     return hardware_interface::return_type::ERROR;
    // }

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type CbHwTest::write(const rclcpp::Time & time, const rclcpp::Duration & period)
{
    return hardware_interface::return_type::OK;
}

}  // namespace cb_hw_test

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(cb_hw_test::CbHwTest, hardware_interface::SystemInterface)
