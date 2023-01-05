/*
 * Copyright (C) 2006-2023 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * BSD-3-Clause license. See the accompanying LICENSE file for details.
 */

#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif

#include "ControlBoard_nws_ros2.h"

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cmath>

#include <yarp/os/LogComponent.h>
#include <yarp/os/LogStream.h>
#include <Ros2Utils.h>
#include <rcutils/logging_macros.h>

using namespace std::chrono_literals;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;

namespace {
YARP_LOG_COMPONENT(CONTROLBOARD_ROS2, "yarp.ros2.controlBoard_nws_ros2", yarp::os::Log::TraceType);


/** convert degrees to radiants for ROS messages */
inline double convertDegreesToRadians(double degrees)
{
    return degrees / 180.0 * M_PI;
}

}


ControlBoard_nws_ros2::ControlBoard_nws_ros2() :
        yarp::os::PeriodicThread(m_default_period)
{
}

void ControlBoard_nws_ros2::closePorts()
{
    // FIXME
    yCWarning(CONTROLBOARD_ROS2, "FIXME: closePorts() is not implemented yet!");
    YARP_UNUSED(this);
}

bool ControlBoard_nws_ros2::close()
{
    if(m_spinner){
        if(m_spinner->isRunning()){
            m_spinner->stop();
        }
    }
    // Ensure that the device is not running
    if (isRunning()) {
        stop();
    }

    closeDevice();
    closePorts();

    return true;
}


bool ControlBoard_nws_ros2::open(Searchable& config)
{
    Property prop;
    prop.fromString(config.toString());

    // Check parameter, so if both are present we use the correct one
    if (prop.check("period")) {
        if (!prop.find("period").isFloat64()) {
            yCError(CONTROLBOARD_ROS2) << "'period' parameter is not a double value";
            return false;
        }
        m_period = prop.find("period").asFloat64();
        if (m_period <= 0) {
            yCError(CONTROLBOARD_ROS2) << "'period' parameter is not valid, read value is" << m_period;
            return false;
        }
    } else {
        yCDebug(CONTROLBOARD_ROS2) << "'period' parameter missing, using default thread period = 0.02s";
        m_period = m_default_period;
    }

    // check for node_name parameter
    if (!config.check("node_name")) {
        yCError(CONTROLBOARD_ROS2) << " cannot find node_name parameter";
        return false;
    }
    m_nodeName = config.find("node_name").asString();
    if(m_nodeName[0] == '/'){
        yCError(CONTROLBOARD_ROS2) << "node_name cannot have an initial /";
        return false;
    }
    // check for topic_name parameter
    if (!config.check("topic_name")) {
        yCError(CONTROLBOARD_ROS2) << " cannot find topic_name parameter";
        return false;
    }
    m_jointStateTopicName = config.find("topic_name").asString();
    if(m_jointStateTopicName[0] != '/'){
        yCError(CONTROLBOARD_ROS2) << "topic_name must begin with an initial /";
        return false;
    }
    yCInfo(CONTROLBOARD_ROS2) << "topic_name is " << m_jointStateTopicName;

    m_node = NodeCreator::createNode(m_nodeName);
    m_publisher = m_node->create_publisher<sensor_msgs::msg::JointState>(m_jointStateTopicName, 10);

    if (config.check("msgs_name")) {
        m_msgs_name = config.find("msgs_name").asString();
        if (m_msgs_name[0] != '/') {
            m_msgs_name = "/"+m_msgs_name;
        }
    }

    return true;
}


bool ControlBoard_nws_ros2::initRos2Control(const std::string& name){

    m_posTopicName = name+"/position";
    m_posDirTopicName = name+"/position_direct";
    m_velTopicName = name+"/velocity";
    m_getModesSrvName = name+"/get_modes";
    m_setModesSrvName = name+"/set_modes";
    m_getAvailableModesSrvName = name+"/get_available_modes";
    m_getJointsNamesSrvName = name+"/get_joints_names";

    // Creating topics ------------------------------------------------------------------------------------------------- //

    if(m_iPositionControl){
        m_posSubscription = m_node->create_subscription<yarp_control_msgs::msg::Position>(m_posTopicName, 10,
                                                                                        std::bind(&ControlBoard_nws_ros2::positionTopic_callback,
                                                                                        this, std::placeholders::_1));
        if(!m_posSubscription){
            yCError(CONTROLBOARD_ROS2) << "Could not initialize the Position msg subscription";
            RCLCPP_ERROR(m_node->get_logger(),"Could not initialize the Position msg subscription");

            return false;
        }
    }
    if(m_iPositionDirect){
        m_posDirectSubscription = m_node->create_subscription<yarp_control_msgs::msg::PositionDirect>(m_posDirTopicName, 10,
                                                                                                    std::bind(&ControlBoard_nws_ros2::positionDirectTopic_callback,
                                                                                                                this, std::placeholders::_1));
        if(!m_posDirectSubscription){
            yCError(CONTROLBOARD_ROS2) << "Could not initialize the Position direct msg subscription";
            RCLCPP_ERROR(m_node->get_logger(),"Could not initialize the Position direct msg subscription");

            return false;
        }
    }
    if(m_iVelocityControl){
        m_velSubscription = m_node->create_subscription<yarp_control_msgs::msg::Velocity>(m_velTopicName, 10,
                                                                                        std::bind(&ControlBoard_nws_ros2::velocityTopic_callback,
                                                                                        this, std::placeholders::_1));
        if(!m_velSubscription){
            yCError(CONTROLBOARD_ROS2) << "Could not initialize the Velocity msg subscription";
            RCLCPP_ERROR(m_node->get_logger(),"Could not initialize the Velocity msg subscription");

            return false;
        }
    }

    // Creating services ----------------------------------------------------------------------------------------------- //

    if(!m_iControlMode){
        return true;
    }

    rmw_qos_profile_t qos;
    qos.history = RMW_QOS_POLICY_HISTORY_SYSTEM_DEFAULT;
    qos.depth=10;
    rmw_time_t time;
    time.sec=10000;
    time.nsec = 0;
    qos.deadline= time;
    qos.lifespan=time;
    qos.liveliness_lease_duration=time;
    qos.reliability = RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT;
    qos.durability = RMW_QOS_POLICY_DURABILITY_SYSTEM_DEFAULT;
    qos.liveliness = RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT;
    qos.avoid_ros_namespace_conventions = true;
    m_getControlModesSrv = m_node->create_service<yarp_control_msgs::srv::GetControlModes>(m_getModesSrvName,
                                                                                           std::bind(&ControlBoard_nws_ros2::getControlModesCallback,
                                                                                                     this,std::placeholders::_1,std::placeholders::_2,
                                                                                                     std::placeholders::_3));
    if(!m_getControlModesSrv){
        yCError(CONTROLBOARD_ROS2) << "Could not initialize the GetControlModes service";
        RCLCPP_ERROR(m_node->get_logger(),"Could not initialize the GetControlModes service");

        return false;
    }
    m_setControlModesSrv = m_node->create_service<yarp_control_msgs::srv::SetControlModes>(m_setModesSrvName,
                                                                                           std::bind(&ControlBoard_nws_ros2::setControlModesCallback,
                                                                                                     this,std::placeholders::_1,std::placeholders::_2,
                                                                                                     std::placeholders::_3));
    if(!m_setControlModesSrv){
        yCError(CONTROLBOARD_ROS2) << "Could not initialize the SetControlModes service";
        RCLCPP_ERROR(m_node->get_logger(),"Could not initialize the SetControlModes service");

        return false;
    }
    m_getAvailableModesSrv = m_node->create_service<yarp_control_msgs::srv::GetAvailableControlModes>(m_getAvailableModesSrvName,
                                                                                                      std::bind(&ControlBoard_nws_ros2::getAvailableModesCallback,
                                                                                                                this,std::placeholders::_1,std::placeholders::_2,
                                                                                                                std::placeholders::_3));
    if(!m_getAvailableModesSrv){
        yCError(CONTROLBOARD_ROS2) << "Could not initialize the GetAvailableModes service";
        RCLCPP_ERROR(m_node->get_logger(),"Could not initialize the GetAvailableModes service");

        return false;
    }
    m_getJointsNamesSrv = m_node->create_service<yarp_control_msgs::srv::GetJointsNames>(m_getJointsNamesSrvName,
                                                                                         std::bind(&ControlBoard_nws_ros2::getJointsNamesCallback,
                                                                                                   this,std::placeholders::_1,std::placeholders::_2,
                                                                                                   std::placeholders::_3));
    if(!m_getJointsNamesSrv){
        yCError(CONTROLBOARD_ROS2) << "Could not initialize the GetJointsNames service";
        RCLCPP_ERROR(m_node->get_logger(),"Could not initialize the GetJointsNames service");

        return false;
    }

    m_spinner = new Ros2Spinner(m_node);
    if (!m_spinner){
        yCError(CONTROLBOARD_ROS2) << "Could not initialize the GetJointsNames service";
        RCLCPP_ERROR(m_node->get_logger(),"Could not initialize the GetJointsNames service");

        return false;
    }

    std::string tmpName;
    for(size_t i=0; i<m_subdevice_joints; i++){
        if(!m_iAxisInfo->getAxisName(i,tmpName)){
            yCError(CONTROLBOARD_ROS2) << "Error retrieving axis" << i << "name. For this device to work, every joint needs a name";

            return false;
        }
        m_quickJointRef[tmpName] = i;
    }

    return true;
}


bool ControlBoard_nws_ros2::setDevice(yarp::dev::DeviceDriver* driver)
{
    yCAssert(CONTROLBOARD_ROS2, driver);

    // Save the pointer and subDeviceOwned
    m_subdevice_ptr = driver;

    m_subdevice_ptr->view(m_iPositionControl);
    if (!m_iPositionControl) {
        yCError(CONTROLBOARD_ROS2, "<%s - %s>: IPositionControl interface was not found in attached device.",  m_nodeName.c_str(), m_jointStateTopicName.c_str());
    }

    m_subdevice_ptr->view(m_iPositionDirect);
    if (!m_iPositionDirect) {
        yCError(CONTROLBOARD_ROS2, "<%s - %s>: IPositionDirect interface was not found in attached device.",  m_nodeName.c_str(), m_posTopicName.c_str());
    }

    m_subdevice_ptr->view(m_iVelocityControl);
    if (!m_iVelocityControl) {
        yCError(CONTROLBOARD_ROS2, "<%s - %s>: IVelocityControl interface was not found in attached device.",  m_nodeName.c_str(), m_posTopicName.c_str());
    }

    m_subdevice_ptr->view(m_iControlMode);
    if (!m_iControlMode) {
        yCError(CONTROLBOARD_ROS2, "<%s - %s>: IControlMode interface was not found in attached device.",  m_nodeName.c_str(), m_posTopicName.c_str());
    }

    m_subdevice_ptr->view(m_iEncodersTimed);
    if (!m_iEncodersTimed) {
        yCError(CONTROLBOARD_ROS2, "<%s - %s>: IEncodersTimed interface was not found in attached device. Quitting",  m_nodeName.c_str(), m_jointStateTopicName.c_str());
        return false;
    }

    m_subdevice_ptr->view(m_iTorqueControl);
    if (!m_iTorqueControl) {
        yCWarning(CONTROLBOARD_ROS2, "<%s - %s>: ITorqueControl interface was not found in attached device.",  m_nodeName.c_str(), m_jointStateTopicName.c_str());
    }

    m_subdevice_ptr->view(m_iAxisInfo);
    if (!m_iAxisInfo) {
        yCError(CONTROLBOARD_ROS2, "<%s - %s>: IAxisInfo interface was not found in attached device. Quitting",  m_nodeName.c_str(), m_jointStateTopicName.c_str());
        return false;
    }

    // Get the number of controlled joints
    int tmp_axes;
    if (!m_iEncodersTimed->getAxes(&tmp_axes)) {
        yCError(CONTROLBOARD_ROS2, "<%s - %s>: Failed to get axes number for attached device ",  m_nodeName.c_str(), m_jointStateTopicName.c_str());
        return false;
    }
    if (tmp_axes <= 0) {
        yCError(CONTROLBOARD_ROS2, "<%s - %s>: attached device has an invalid number of joints (%d)",  m_nodeName.c_str(), m_jointStateTopicName.c_str(), tmp_axes);
        return false;
    }
    m_subdevice_joints = static_cast<size_t>(tmp_axes);
    m_times.resize(m_subdevice_joints);
    m_ros_struct.name.resize(m_subdevice_joints);
    m_ros_struct.position.resize(m_subdevice_joints);
    m_ros_struct.velocity.resize(m_subdevice_joints);
    m_ros_struct.effort.resize(m_subdevice_joints);

    if (!updateAxisName()) {
        return false;
    }

    return true;
}


void ControlBoard_nws_ros2::closeDevice()
{
    m_subdevice_ptr = nullptr;
    m_subdevice_joints = 0;

    m_times.clear();

    // Clear all interfaces
    m_iPositionControl = nullptr;
    m_iEncodersTimed = nullptr;
    m_iTorqueControl = nullptr;
    m_iAxisInfo = nullptr;
}

bool ControlBoard_nws_ros2::attach(yarp::dev::PolyDriver* poly)
{
    if (!setDevice(poly)) {
        return false;
    }

    setPeriod(m_period);
    if (!start()) {
        yCError(CONTROLBOARD_ROS2) << "Error starting thread";
        return false;
    }

    if(!initRos2Control(m_msgs_name)){
        yCError(CONTROLBOARD_ROS2) << "Error initializing the ROS2 control related topics and services";
        RCLCPP_ERROR(m_node->get_logger(),"Error initializing the ROS2 control related topics and services");
        return false;
    }

    if(m_spinner){
        if(!m_spinner->start()){
            yCError(CONTROLBOARD_ROS2) << "Error starting the spinner";
        }
    }

    return true;
}

bool ControlBoard_nws_ros2::detach()
{
    // Ensure that the device is not running
    if (isRunning()) {
        stop();
    }

    closeDevice();

    return true;
}

bool ControlBoard_nws_ros2::updateAxisName()
{
    // IMPORTANT!! This function has to be called BEFORE the thread starts,
    // the name has to be correct right from the first message!!

    yCAssert(CONTROLBOARD_ROS2, m_iAxisInfo);

    std::vector<std::string> tmpVect;
    for (size_t i = 0; i < m_subdevice_joints; i++) {
        std::string tmp;
        bool ret = m_iAxisInfo->getAxisName(i, tmp);
        if (!ret) {
            yCError(CONTROLBOARD_ROS2, "Joint name for axis %zu not found!", i);
            return false;
        }
        tmpVect.emplace_back(tmp);
    }

    yCAssert(CONTROLBOARD_ROS2, tmpVect.size() == m_subdevice_joints);

    m_jointNames = tmpVect;

    return true;
}

void ControlBoard_nws_ros2::run()
{
    yCAssert(CONTROLBOARD_ROS2, m_iEncodersTimed);
    yCAssert(CONTROLBOARD_ROS2, m_iAxisInfo);

    bool positionsOk = m_iEncodersTimed->getEncodersTimed(m_ros_struct.position.data(), m_times.data());
    YARP_UNUSED(positionsOk);

    bool speedsOk = m_iEncodersTimed->getEncoderSpeeds(m_ros_struct.velocity.data());
    YARP_UNUSED(speedsOk);

    if (m_iTorqueControl) {
        bool torqueOk = m_iTorqueControl->getTorques(m_ros_struct.effort.data());
        YARP_UNUSED(torqueOk);
    }

    // Update the port envelope time by averaging all timestamps
    m_time.update(std::accumulate(m_times.begin(), m_times.end(), 0.0) / m_subdevice_joints);
    yarp::os::Stamp averageTime = m_time;

    // Data from HW have been gathered few lines before
    JointTypeEnum jType;
    for (size_t i = 0; i < m_subdevice_joints; i++) {
        m_iAxisInfo->getJointType(i, jType);
        if (jType == VOCAB_JOINTTYPE_REVOLUTE) {
            m_ros_struct.position[i] = convertDegreesToRadians(m_ros_struct.position[i]);
            m_ros_struct.velocity[i] = convertDegreesToRadians(m_ros_struct.velocity[i]);
        }
    }

    m_ros_struct.name = m_jointNames;
    m_ros_struct.header.stamp.sec = int(averageTime.getTime()); // FIXME
    m_ros_struct.header.stamp.nanosec = static_cast<int>(1000000000 * (averageTime.getTime() - int(averageTime.getTime()))); // FIXME

//    m_ros_struct.header.stamp = m_node->get_clock()->now();    //@@@@@@@@@@@ FIXME: averageTime.getTime();
//     m_ros_struct.header.frame_id = m_frame_id; // FIXME


    // FIXME
    ++m_counter;
//     m_ros_struct.header.seq = m_counter++;

    m_publisher->publish(m_ros_struct);
}
