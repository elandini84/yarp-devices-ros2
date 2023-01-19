/*
 * SPDX-FileCopyrightText: 2023 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef YARP_ROS2_ROS2TEST_H
#define YARP_ROS2_ROS2TEST_H

#include <yarp/dev/DeviceDriver.h>
#include <yarp/os/PeriodicThread.h>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <mutex>

class Ros2Init
{
public:
    Ros2Init();

    std::shared_ptr<rclcpp::Node> node;

    static Ros2Init& get();
};


/**
 *  @ingroup dev_impl_nws_ros2 dev_impl_fake
 *
 * \brief `ros2test`: A Network publisher test
 *
 *  Documentation to be added
 *
*/
class MinimalPublisher
{
public:
    MinimalPublisher(const std::string& topicname);
};


class Ros2Test :
        public yarp::dev::DeviceDriver,
        public yarp::os::PeriodicThread
{
public:
    Ros2Test();
    Ros2Test(const Ros2Test&) = delete;
    Ros2Test(Ros2Test&&) noexcept = delete;
    Ros2Test& operator=(const Ros2Test&) = delete;
    Ros2Test& operator=(Ros2Test&&) noexcept = delete;
    ~Ros2Test() override = default;

    // DeviceDriver
    bool open(yarp::os::Searchable& config) override;
    bool close() override;

    // PeriodicThread
    void run() override;

private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr m_publisher;
    std::string m_topic;
    size_t m_count {0};
};

#endif // YARP_ROS2_ROS2TEST_H
