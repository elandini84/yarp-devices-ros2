/*
 * Copyright (C) 2006-2021 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * BSD-3-Clause license. See the accompanying LICENSE file for details.
 */
#include "Ros2Utils.h"


rclcpp::Node::SharedPtr NodeCreator::createNode(std::string name)
{
    if(!rclcpp::ok())
    {
        rclcpp::init(/*argc*/ 0, /*argv*/ nullptr);
    }
    return std::make_shared<rclcpp::Node>(name);

}
rclcpp::Node::SharedPtr NodeCreator::createNode(std::string name, rclcpp::NodeOptions& node_options)
{
    if(!rclcpp::ok())
    {
        rclcpp::init(/*argc*/ 0, /*argv*/ nullptr);
    }
    return std::make_shared<rclcpp::Node>(name, node_options);

}

builtin_interfaces::msg::Time ros2TimeFromYarp(double yarpTime)
{
    builtin_interfaces::msg::Time ros2Time;
    uint64_t sec_part;
    uint64_t nsec_part;
    sec_part = int(yarpTime); // (time / 1000000000UL);
    nsec_part = (yarpTime - sec_part)*1000000000UL;
    ros2Time.sec = sec_part;
    ros2Time.nanosec = (uint32_t)nsec_part;
    return ros2Time;
}
