/*
 * SPDX-FileCopyrightText: 2023 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef ROS2_RGBD_CONVERSION_UTILS_H
#define ROS2_RGBD_CONVERSION_UTILS_H

#include <iostream>
#include <cstring>

#include <yarp/sig/all.h>
#include <yarp/sig/Matrix.h>
#include <yarp/os/Stamp.h>
#include <yarp/os/Property.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <std_msgs/msg/header.hpp>

#include "ros2PixelCode.h"


constexpr double ONE_BILLION = 1000000000;
typedef yarp::sig::ImageOf<yarp::sig::PixelFloat> DepthImage;

namespace yarp {
    namespace dev {
        namespace Ros2RGBDConversionUtils {

    void convertTimeStampRos2ToYarp(const std_msgs::msg::Header& ros_stamp,
                                    yarp::os::Stamp& yarp_stamp);
    void convertTimeStampYarpToRos2(const yarp::os::Stamp& yarp_stamp,
                                    std_msgs::msg::Header& ros_stamp);

    void convertRGBImageRos2ToYarpFlexImage(sensor_msgs::msg::Image::SharedPtr ros_image_src,
                                            yarp::sig::FlexImage& dest);
    void convertDepthImageRos2ToYarpImageOf(sensor_msgs::msg::Image::SharedPtr ros_image_src,
                                            yarp::sig::ImageOf<yarp::sig::PixelFloat>& dest);

    void updateStamp(sensor_msgs::msg::CameraInfo::SharedPtr ros_camera_info_src,
                     std::string& frame_id_dest,
                     yarp::os::Stamp& yarp_stamp);

    void deepCopyFlexImage(const yarp::sig::FlexImage& src, yarp::sig::FlexImage& dest);

    void deepCopyImageOf(const DepthImage& src, DepthImage& dest);
} // namespace Ros2RGBDConversionUtils
} // namespace dev
} // namespace yarp

#endif //ROS2_RGBD_CONVERSION_UTILS_H
