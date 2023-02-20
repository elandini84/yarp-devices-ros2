#include <algorithm>
#include <cmath>
#include <iterator>
#include <limits>
#include <set>
#include <chrono>
#include <cstdlib>
#include <memory>

#include "cb_hw_pos_vel_test/cb_hw_pos_vel_test.hpp"

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rcutils/logging_macros.h"

using namespace std::chrono_literals;

namespace cb_hw_pos_vel_test
{

CbHwPosVelTest::CbHwPosVelTest()
{
}

CbHwPosVelTest::~CbHwPosVelTest()
{
}

void CbHwPosVelTest::_jointsStatesCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    std::lock_guard<std::mutex> dataGuard(m_dataMutex);
    m_currentJointsStates = *msg;
}

bool CbHwPosVelTest::_checkJoints(const std::vector<hardware_interface::ComponentInfo>& joints)
{
    std::vector<std::string> all_joints;
    auto namesRequest = std::make_shared<yarp_control_msgs::srv::GetJointsNames::Request>();
    while (!m_getJointsNamesClient->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(m_node->get_logger(), "Interrupted while waiting for the service. Exiting.");
            return false;
        }
        RCLCPP_INFO(m_node->get_logger(), "service not available, waiting again...");
    }
    auto namesResponse = m_getJointsNamesClient->async_send_request(namesRequest);
    if(rclcpp::spin_until_future_complete(m_node, namesResponse) == rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_INFO(m_node->get_logger(), "Got joints names");
        all_joints = namesResponse.get()->names;
    }
    else {
        RCLCPP_ERROR(m_node->get_logger(),"Failed to get joints names");
        return false;
    }

    size_t ind = 0;
    for(const auto& joint : joints){
        if (std::find(all_joints.begin(), all_joints.end(), joint.name) == all_joints.end())
        {
            RCLCPP_FATAL(m_node->get_logger(),"The joint named %s was not found among the available ones",
                         joint.name.c_str());
            return false;
        }
        m_jointNames.push_back(joint.name);
        m_jointsIndexes.push_back(ind++);
    }

    auto modeRequest = std::make_shared<yarp_control_msgs::srv::GetControlModes::Request>();
    while (!m_getControlModesClient->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(m_node->get_logger(), "Interrupted while waiting for the service. Exiting.");
            return false;
        }
        RCLCPP_INFO(m_node->get_logger(), "service not available, waiting again...");
    }
    modeRequest->names = m_jointNames;
    auto modeFuture = m_getControlModesClient->async_send_request(modeRequest);
    if(rclcpp::spin_until_future_complete(m_node, modeFuture) == rclcpp::FutureReturnCode::SUCCESS) {
        auto modeResponse = modeFuture.get();
        for(size_t i=0; i<m_jointNames.size(); i++)
        {
            if(modeResponse->modes[i] != "POSITION")
            {
                RCLCPP_ERROR(m_node->get_logger(), "Joint %s in not in control mode POSITION. Check your configuration and, if possible, change the joint control mode",m_jointNames[i].c_str());
                return false;
            }
        }
    }
    else {
        RCLCPP_ERROR(m_node->get_logger(),"Failed to get joints control modes");
        return false;
    }

    return true;
}

CallbackReturn CbHwPosVelTest::_initExportableInterfaces(const std::vector<hardware_interface::ComponentInfo>& joints)
{
    if(!_checkJoints(joints))
    {
        RCLCPP_FATAL(m_node->get_logger(),"Unable to initialize the joints. Check the previous errors for more details");
        return CallbackReturn::ERROR;
    }
    m_hwCommandsPositions.resize(joints.size(), std::numeric_limits<double>::quiet_NaN());
    m_hwStatesPositions.resize(joints.size(), std::numeric_limits<double>::quiet_NaN());
    m_hwStatesVelocities.resize(joints.size(), std::numeric_limits<double>::quiet_NaN());
    m_oldPositions.resize(joints.size(), std::numeric_limits<double>::quiet_NaN());
    size_t i=0;

    for (const auto& joint : joints)
    {
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

        if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION &&
            joint.state_interfaces[1].name != hardware_interface::HW_IF_POSITION)
        {
            RCLCPP_FATAL(
                m_node->get_logger(),
                "Joint '%s' has no %s state interface. Check your configuration", joint.name.c_str(),
                hardware_interface::HW_IF_POSITION);
            return CallbackReturn::ERROR;
        }
        if (joint.state_interfaces[0].name != hardware_interface::HW_IF_VELOCITY &&
            joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
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
        m_hwStatesVelocities[i] = 0.0;
        m_oldPositions[i++] = 0.0;
    }

    return _getHWCurrentValues();
}

CallbackReturn CbHwPosVelTest::_getHWCurrentValues()
{
    auto posRequest = std::make_shared<yarp_control_msgs::srv::GetPosition::Request>();
    posRequest->names = m_jointNames;
    while (!m_getPositionClient->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(m_node->get_logger(), "Interrupted while waiting for the service. Exiting.");
            return CallbackReturn::ERROR;
        }
        RCLCPP_INFO(m_node->get_logger(), "service not available, waiting again...");
    }
    auto posFuture = m_getPositionClient->async_send_request(posRequest);
    auto posResponse = std::make_shared<yarp_control_msgs::srv::GetPosition::Response>();
    if(rclcpp::spin_until_future_complete(m_node, posFuture) == rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_INFO(m_node->get_logger(), "Got joints positions");
        posResponse = posFuture.get();
    }
    else {
        RCLCPP_ERROR(m_node->get_logger(),"Failed to get joints positions");
        return CallbackReturn::ERROR;
    }

    for (size_t i=0; i<m_jointNames.size(); i++)
    {
        m_hwCommandsPositions[i] = posResponse->positions[i];
        m_oldPositions[i] = posResponse->positions[i];
    }

    m_active = true;

    return CallbackReturn::SUCCESS;
}

CallbackReturn CbHwPosVelTest::on_init(const hardware_interface::HardwareInfo & info)
{
    if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
    {
        return CallbackReturn::ERROR;
    }

    if(info.hardware_parameters.count("node_name")<=0)
    {
        RCLCPP_FATAL(rclcpp::get_logger("CbHwPosVelTest"),"No node name specified");
        return CallbackReturn::ERROR;
    }
    if(info.hardware_parameters.count("cb_nws_msgs_name")<=0)
    {
        RCLCPP_FATAL(rclcpp::get_logger("CbHwPosVelTest"),"No msgs name for the controlBoard_nws_ros2 specified");
        return CallbackReturn::ERROR;
    }
    if(info.hardware_parameters.count("continuous_pos_write")<=0)
    {
        RCLCPP_FATAL(rclcpp::get_logger("CbHwPosVelTest"),"No flag for the position continuous writing");
        return CallbackReturn::ERROR;
    }

    m_nodeName = info_.hardware_parameters["node_name"];
    m_msgs_name = info_.hardware_parameters["cb_nws_msgs_name"];
    m_continuousPosWrite = info_.hardware_parameters["continuous_pos_write"]==std::string("true") || info_.hardware_parameters["continuous_pos_write"]==std::string("True");

    m_node = rclcpp::Node::make_shared(m_nodeName);

    // Initialize topics and services names ------------------------------------------------------------------- //
    m_posTopicName = m_msgs_name+"/position";
    m_getModesClientName = m_msgs_name+"/get_modes";
    m_getPositionClientName = m_msgs_name+"/get_position";
    m_getJointsNamesClientName = m_msgs_name+"/get_joints_names";

    // Initialize publishers ---------------------------------------------------------------------------------- //
    m_posPublisher = m_node->create_publisher<yarp_control_msgs::msg::Position>(m_posTopicName, 10);
    if(!m_posPublisher){
        RCLCPP_ERROR(m_node->get_logger(),"Could not initialize the Position publisher");
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

    return _initExportableInterfaces(info_.joints);
}

std::vector<hardware_interface::StateInterface> CbHwPosVelTest::export_state_interfaces()
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

std::vector<hardware_interface::CommandInterface> CbHwPosVelTest::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> ifacesToReturn;
    for (size_t i=0; i<m_hwCommandsPositions.size(); i++)
    {
        ifacesToReturn.emplace_back(hardware_interface::CommandInterface(
            info_.joints[i].name, hardware_interface::HW_IF_POSITION, &m_hwCommandsPositions[i]));
    }

    return ifacesToReturn;
}

hardware_interface::return_type CbHwPosVelTest::prepare_command_mode_switch(const std::vector<std::string> & start_interfaces, const std::vector<std::string> & stop_interfaces)
{
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type CbHwPosVelTest::perform_command_mode_switch(const std::vector<std::string> & start_interfaces, const std::vector<std::string> & stop_interfaces)
{
    return hardware_interface::return_type::OK;
}

CallbackReturn CbHwPosVelTest::on_activate(const rclcpp_lifecycle::State & previous_state)
{
    if(_getHWCurrentValues() == CallbackReturn::ERROR)
    {
        RCLCPP_ERROR(m_node->get_logger(),"Could not successfully read the current joints positions. Check previous errors for more info");
        return CallbackReturn::ERROR;
    }
    m_active = true;
    return CallbackReturn::SUCCESS;
}

CallbackReturn CbHwPosVelTest::on_deactivate(const rclcpp_lifecycle::State & previous_state)
{
    m_active = false;
    return CallbackReturn::SUCCESS;
}

hardware_interface::return_type CbHwPosVelTest::read(const rclcpp::Time & time, const rclcpp::Duration & period)
{
    std::lock_guard<std::mutex> dataGuard(m_dataMutex);
    for(size_t i=0; i<m_jointsIndexes.size(); i++)
    {
        m_hwStatesPositions[i] = m_currentJointsStates.position[m_jointsIndexes[i]];
        m_hwStatesVelocities[i] = m_currentJointsStates.velocity[m_jointsIndexes[i]];
    }
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type CbHwPosVelTest::write(const rclcpp::Time & time, const rclcpp::Duration & period)
{
    if(!m_active)
    {
        return hardware_interface::return_type::OK;
    }
    else if(m_hwCommandsPositions == m_oldPositions && !m_continuousPosWrite)
    {
        //RCLCPP_INFO(m_node->get_logger(), "No changes in the stored command values. Skipping");
        return hardware_interface::return_type::OK;
    }
    yarp_control_msgs::msg::Position posToSend;
    posToSend.names = m_jointNames;
    posToSend.positions = m_hwCommandsPositions;

    m_posPublisher->publish(posToSend);
    m_oldPositions = m_hwCommandsPositions;

    return hardware_interface::return_type::OK;
}

}  // namespace cb_hw_pos_vel_test

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(cb_hw_pos_vel_test::CbHwPosVelTest, hardware_interface::SystemInterface)
