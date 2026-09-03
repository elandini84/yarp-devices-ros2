/*
 * SPDX-FileCopyrightText: 2023 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "FrameGrabber_nws_ros2.h"

#include <yarp/os/LogComponent.h>
#include <yarp/os/LogStream.h>
#include <yarp/dev/PolyDriver.h>

#include <sensor_msgs/image_encodings.hpp>
#include <Ros2Utils.h>

namespace {
YARP_LOG_COMPONENT(FRAMEGRABBER_NWS_ROS2, "yarp.device.frameGrabber_nws_ros2")

// FIXME Copied from rgbdSensor_nws_ros2
std::string yarp2RosPixelCode(int code)
{
    switch (code)
    {
    case VOCAB_PIXEL_BGR:
        return sensor_msgs::image_encodings::BGR8;
    case VOCAB_PIXEL_BGRA:
        return sensor_msgs::image_encodings::BGRA8;
    case VOCAB_PIXEL_ENCODING_BAYER_BGGR16:
        return sensor_msgs::image_encodings::BAYER_BGGR16;
    case VOCAB_PIXEL_ENCODING_BAYER_BGGR8:
        return sensor_msgs::image_encodings::BAYER_BGGR8;
    case VOCAB_PIXEL_ENCODING_BAYER_GBRG16:
        return sensor_msgs::image_encodings::BAYER_GBRG16;
    case VOCAB_PIXEL_ENCODING_BAYER_GBRG8:
        return sensor_msgs::image_encodings::BAYER_GBRG8;
    case VOCAB_PIXEL_ENCODING_BAYER_GRBG16:
        return sensor_msgs::image_encodings::BAYER_GRBG16;
    case VOCAB_PIXEL_ENCODING_BAYER_GRBG8:
        return sensor_msgs::image_encodings::BAYER_GRBG8;
    case VOCAB_PIXEL_ENCODING_BAYER_RGGB16:
        return sensor_msgs::image_encodings::BAYER_RGGB16;
    case VOCAB_PIXEL_ENCODING_BAYER_RGGB8:
        return sensor_msgs::image_encodings::BAYER_RGGB8;
    case VOCAB_PIXEL_MONO:
        return sensor_msgs::image_encodings::MONO8;
    case VOCAB_PIXEL_MONO16:
        return sensor_msgs::image_encodings::MONO16;
    case VOCAB_PIXEL_RGB:
        return sensor_msgs::image_encodings::RGB8;
    case VOCAB_PIXEL_RGBA:
        return sensor_msgs::image_encodings::RGBA8;
    case VOCAB_PIXEL_MONO_FLOAT:
        return sensor_msgs::image_encodings::TYPE_32FC1;
    default:
        return sensor_msgs::image_encodings::RGB8;
    }
}
} // namespace

FrameGrabber_nws_ros2::FrameGrabber_nws_ros2() :
        PeriodicThread(s_default_period)
{
}


FrameGrabber_nws_ros2::~FrameGrabber_nws_ros2()
{
    close();
}


bool FrameGrabber_nws_ros2::close()
{
    if (!m_active) {
        return false;
    }
    m_active = false;

    detach();

    return true;
}


bool FrameGrabber_nws_ros2::open(yarp::os::Searchable& config)
{
    if (m_active) {
        yCError(FRAMEGRABBER_NWS_ROS2, "Device is already opened");
        return false;
    }
    parseParams(config);

    if(m_namespace.empty())
    {
        m_node = NodeCreator::createNode(m_node_name);
    } else {
        m_node = NodeCreator::createNode(m_node_name, m_namespace);
    }

    if(m_node == nullptr) {
        yCError(FRAMEGRABBER_NWS_ROS2) << " opening " << m_node_name << " Node, check your yarp-ROS2 network configuration\n";
        return false;
    }

    publisher_image = m_node->create_publisher<sensor_msgs::msg::Image>(m_topic_name, 10);


    // set "cameraInfoTopicName" and open publisher
    std::string cameraInfoTopicName = m_topic_name.substr(0,m_topic_name.rfind('/')) + "/camera_info";
    publisher_cameraInfo = m_node->create_publisher<sensor_msgs::msg::CameraInfo>(cameraInfoTopicName, 10);

    yCInfo(FRAMEGRABBER_NWS_ROS2) << "Running, waiting for attach...";

    m_active = true;

    return true;
}

bool FrameGrabber_nws_ros2::attach(yarp::dev::PolyDriver* poly)
{
    if (!poly->isValid()) {
        yCError(FRAMEGRABBER_NWS_ROS2) << "Device " << poly << " to attach to is not valid ... cannot proceed";
        return false;
    }

    PeriodicThread::setPeriod(m_period);

    poly->view(iRgbVisualParams);
    poly->view(iFrameGrabberImage);
    poly->view(iPreciselyTimed);

    if (iFrameGrabberImage == nullptr) {
        yCError(FRAMEGRABBER_NWS_ROS2) << "IFrameGrabberImage interface is not available on the device";
        return false;
    }

    if (iRgbVisualParams == nullptr) {
        yCWarning(FRAMEGRABBER_NWS_ROS2) << "IRgbVisualParams interface is not available on the device";
    }

    return PeriodicThread::start();
}


bool FrameGrabber_nws_ros2::detach()
{
    if (yarp::os::PeriodicThread::isRunning()) {
        yarp::os::PeriodicThread::stop();
    }

    iRgbVisualParams = nullptr;
    iFrameGrabberImage = nullptr;
    iPreciselyTimed = nullptr;

    return true;
}

bool FrameGrabber_nws_ros2::threadInit()
{
    yarpimg = new yarp::sig::ImageOf<yarp::sig::PixelRgb>;
    return true;
}

void FrameGrabber_nws_ros2::threadRelease()
{
    delete yarpimg;
    yarpimg = nullptr;
}


// Publish the images on the buffered port
void FrameGrabber_nws_ros2::run()
{
//     if (false /* FIXME Can we check if there are subscribers connected in ROS2? */) {
//         // If no subscribers are connected, do not call getImage on the interface.
//         return;
//     }

    if (iPreciselyTimed) {
        m_stamp = iPreciselyTimed->getLastInputStamp();
    } else {
        m_stamp.update(yarp::os::Time::now());
    }

    if (iFrameGrabberImage)
    {
        if(publisher_image->get_subscription_count()>0){
            if (iFrameGrabberImage->getImage(*yarpimg))
            {
                sensor_msgs::msg::Image rosimg;
                rosimg.data.resize(yarpimg->getRawImageSize());
                rosimg.width = yarpimg->width();
                rosimg.height = yarpimg->height();
                rosimg.encoding = yarp2RosPixelCode(yarpimg->getPixelCode());
                rosimg.step = yarpimg->getRowSize();
                rosimg.header.frame_id = m_frame_id;
        //         rosimg.header.stamp.sec = static_cast<int>(m_stamp.getTime()); // FIXME
        //         rosimg.header.stamp.nanosec = static_cast<int>(1000000000UL * (m_stamp.getTime() - int(m_stamp.getTime()))); // FIXME
                rosimg.is_bigendian = 0;
                    memcpy(rosimg.data.data(), yarpimg->getRawImage(), yarpimg->getRawImageSize());
                    publisher_image->publish(rosimg);
            }
            else
            {
                yCError(FRAMEGRABBER_NWS_ROS2) << "Image not captured (getImage failed). Check hardware configuration.";
            }
        }
    }
    else
    {
        yCError(FRAMEGRABBER_NWS_ROS2) << "Invalid call to interface iFrameGrabberImage";
    }

    if (iRgbVisualParams && publisher_cameraInfo->get_subscription_count()>0)
    {
        sensor_msgs::msg::CameraInfo cameraInfo;
        if (setCamInfo(cameraInfo)) {

            publisher_cameraInfo->publish(cameraInfo);
        }
    }
    else
    {
        yCError(FRAMEGRABBER_NWS_ROS2) << "Invalid call to interface iRgbVisualParams";
    }
}

namespace {
template <class T>
struct param
{
    param(T& inVar, std::string inName) :
        var(&inVar),
        parname(std::move(inName))
    {
    }
    T*              var;
    std::string     parname;
};
} // namespace

bool FrameGrabber_nws_ros2::setCamInfo(sensor_msgs::msg::CameraInfo& cameraInfo)
{
    yarp::sig::IntrinsicParams camData;
    if (!iRgbVisualParams->getRgbIntrinsicParam(camData)) {
        yCErrorThreadOnce(FRAMEGRABBER_NWS_ROS2) << "Unable to get intrinsic param from rgb sensor!";
        return false;
    }

    std::string distModel;

    switch (camData.distortionModel.type)
    {
    case yarp::sig::CameraDistortionType::YARP_DISTORTION_NONE:
        distModel = "none";
        break;
    case yarp::sig::CameraDistortionType::YARP_PLUMB_BOB:
        distModel = "plumb_bob";
        break;
    default:
        yCWarning(FRAMEGRABBER_NWS_ROS2) << "Unsupported distortion model";
        return false;
    }

    cameraInfo.header.frame_id      = m_frame_id;
    cameraInfo.width                = iRgbVisualParams->getRgbWidth();
    cameraInfo.height               = iRgbVisualParams->getRgbHeight();
    cameraInfo.distortion_model     = distModel;

    if (distModel != "none")
    {
        cameraInfo.d.resize(5);
        cameraInfo.d[0] = camData.distortionModel.k1;
        cameraInfo.d[1] = camData.distortionModel.k2;
        cameraInfo.d[2] = camData.distortionModel.t1;
        cameraInfo.d[3] = camData.distortionModel.t2;
        cameraInfo.d[4] = camData.distortionModel.k3;
    }

    cameraInfo.k[0]  = camData.focalLengthX;       cameraInfo.k[1] = 0;        cameraInfo.k[2] = camData.principalPointX;
    cameraInfo.k[3]  = 0;        cameraInfo.k[4] = camData.focalLengthY;       cameraInfo.k[5] = camData.principalPointY;
    cameraInfo.k[6]  = 0;        cameraInfo.k[7] = 0;        cameraInfo.k[8] = 1;

    /*
     * ROS documentation on cameraInfo message:
     * "Rectification matrix (stereo cameras only)
     * A rotation matrix aligning the camera coordinate system to the ideal
     * stereo image plane so that epipolar lines in both stereo images are
     * parallel."
     * useless in our case, it will be an identity matrix
     */

    cameraInfo.r[0]  = 1;        cameraInfo.r[1] = 0;        cameraInfo.r[2] = 0;
    cameraInfo.r[3]  = 0;        cameraInfo.r[4] = 1;        cameraInfo.r[5] = 0;
    cameraInfo.r[6]  = 0;        cameraInfo.r[7] = 0;        cameraInfo.r[8] = 1;

    cameraInfo.p[0]  = camData.focalLengthX;      cameraInfo.p[1] = 0;    cameraInfo.p[2]  = camData.principalPointX;  cameraInfo.p[3]  = 0;
    cameraInfo.p[4]  = 0;       cameraInfo.p[5] = camData.focalLengthY;   cameraInfo.p[6]  = camData.principalPointY;  cameraInfo.p[7]  = 0;
    cameraInfo.p[8]  = 0;       cameraInfo.p[9] = 0;    cameraInfo.p[10] = 1;   cameraInfo.p[11] = 0;

    cameraInfo.binning_x  = cameraInfo.binning_y = 0;
    cameraInfo.roi.height = cameraInfo.roi.width = cameraInfo.roi.x_offset = cameraInfo.roi.y_offset = 0;
    cameraInfo.roi.do_rectify = false;
    return true;
}
