/******************************************************************************/
/*                                                                            */
/*  Copyright (C) 2018, FLIR Systems                                          */
/*  All rights reserved.                                                      */
/*                                                                            */
/******************************************************************************/
#include <pluginlib/class_list_macros.h>
#include "flir_adk_ethernet/Util.h"
#include "flir_adk_ethernet/BaseCameraController.h"

using namespace cv;
using namespace flir_adk_ethernet;

BaseCameraController::BaseCameraController() : _cvImage()
{
}

BaseCameraController::~BaseCameraController()
{
    if(_camera) {
        delete _camera;
    }
}

void BaseCameraController::onInit()
{
    nh = getNodeHandle();
    pnh = getPrivateNodeHandle();

    it = std::shared_ptr<image_transport::ImageTransport>(new image_transport::ImageTransport(nh));
    _imagePublisher = it->advertiseCamera("image_raw", 1);
    setupExtraPubSub();

    setupCommandListeners();

    bool exit = false;

    std::string ip, cameraInfoStr, formatStr, camType;
    int width, height, xOffset, yOffset;

    pnh.param<std::string>("frame_id", frame_id, "boson_camera");
    pnh.param<std::string>("ip_addr", ip, "");
    pnh.param<std::string>("camera_type", camType, "");
    pnh.param<std::string>("camera_info_url", cameraInfoStr, "");
    pnh.param<std::string>("video_format", formatStr, "COLOR_8");
    pnh.param<int>("width", width, 0);
    pnh.param<int>("height", height, 0);
    pnh.param<int>("xOffset", xOffset, 0);
    pnh.param<int>("yOffset", yOffset, 0);
    pnh.param<bool>("flip", flip, false);

    ROS_INFO("flir_adk_ethernet - Got frame_id: %s.", frame_id.c_str());
    ROS_INFO("flir_adk_ethernet - Got IP: %s.", ip.c_str());
    ROS_INFO("flir_adk_ethernet - Got camera_info_url: %s.",
        cameraInfoStr.c_str());
    ROS_INFO("flir_adk_ethernet - Got video_format: %s.", formatStr.c_str());
    ROS_INFO("flir_adk_ethernet - Got camera_type: %s.", camType.c_str());
    ROS_INFO("flir_adk_ethernet - Got width: %d.", width);
    ROS_INFO("flir_adk_ethernet - Got height: %d.", height);
    ROS_INFO("flir_adk_ethernet - Got xOffset: %d.", xOffset);
    ROS_INFO("flir_adk_ethernet - Got yOffset: %d.", yOffset);

    EthernetCameraInfo info;
    info.ip = ip;
    info.camInfoPath = cameraInfoStr;
    info.pixelFormat = formatStr;
    info.camType = camType;
    info.width = width;
    info.height = height;
    info.xOffset = xOffset;
    info.yOffset = yOffset;

    auto sys = std::make_shared<SystemWrapper>(
        SystemWrapper(System::GetInstance()));

    _camera = new EthernetCamera(info, sys, nh);

    if (!exit) {
        exit = !_camera->openCamera() || exit;
    }

    if (exit)
    {
        ros::shutdown();
        return;
    }

    setupFramePublish();
}

void BaseCameraController::setupExtraPubSub() {
    // do nothing for base class;
}

void BaseCameraController::setupCommandListeners() {
    _pixelFormatListener = nh.subscribe<std_msgs::String>("pixel_format", 10,
        boost::bind(&BaseCameraController::setPixelFormat, this, _1));

    _autoFFCListener = nh.subscribe<std_msgs::Bool>("auto_ffc", 10,
        boost::bind(&BaseCameraController::setAutoFFC, this, _1));

    _ffcListener = nh.subscribe<std_msgs::Empty>("ffc", 10,
        boost::bind(&BaseCameraController::executeFFC, this));

    _setNodeListener = nh.subscribe<diagnostic_msgs::KeyValue>("set_node", 10,
        boost::bind(&BaseCameraController::setNode, this, _1));

    _setROIListener = nh.subscribe<sensor_msgs::RegionOfInterest>("set_roi", 10,
        boost::bind(&BaseCameraController::setROI, this, _1));

    _setCenterROIListener = nh.subscribe<sensor_msgs::RegionOfInterest>("set_center_roi", 10,
        boost::bind(&BaseCameraController::setCenterROI, this, _1));

    _getNodeService = nh.advertiseService<GetNode::Request, GetNode::Response>
        ("get_node", boost::bind(&BaseCameraController::getNode, this, _1, _2));
}

void BaseCameraController::setPixelFormat(const std_msgs::StringConstPtr& msg) {
    auto format = _camera->setPixelFormat(msg->data);
    ROS_INFO("%s - Changed format to %s.", getName().c_str(), format.c_str());
}

void BaseCameraController::setAutoFFC(const std_msgs::BoolConstPtr& msg) {
    auto ffcMode = _camera->setAutoFFC((bool)msg->data);
    if(ffcMode.empty()) {
        ROS_INFO("%s - Cannot set FFC mode", getName().c_str());
        return;
    }
    ROS_INFO("%s - Changed FFC mode to %s.", getName().c_str(), ffcMode.c_str());
}

void BaseCameraController::executeFFC() {
    auto ffc = _camera->performFFC();
    if(ffc.empty()) {
        ROS_INFO("%s - Cannot execute FFC", getName().c_str());
        return;
    }
    ROS_INFO("%s - FFC executed", getName().c_str());
}

void BaseCameraController::setNode(const diagnostic_msgs::KeyValueConstPtr& msg) {
    std::string nodeName = msg->key;
    std::string value = msg->value;
    if(_camera->setNodeValue(nodeName, value)) {
        ROS_INFO("Successfully set node %s", nodeName.c_str());
    } else {
        ROS_ERROR("Unable tp set node %s to value %s", nodeName.c_str(), value.c_str());
    }
}

bool BaseCameraController::getNode(GetNode::Request &req, GetNode::Response &res) {
    std::string result = _camera->getNodeValue(req.nodeName);
    if(result.empty()) {
        res.value = "Node: " + req.nodeName + "does not exist or is not readable";
        return false;
    }

    res.value = result;
    return true;
}

void BaseCameraController::setROI(const sensor_msgs::RegionOfInterestConstPtr msg) {
    int width = msg->width;
    int height = msg->height;
    int xOffset = msg->x_offset;
    int yOffset = msg->y_offset;

    _camera->stopCapture();
    _camera->setROI(xOffset, yOffset, width, height);
    _camera->startCapture();
}

void BaseCameraController::setCenterROI(const sensor_msgs::RegionOfInterestConstPtr msg) {
    int width = msg->width;
    int height = msg->height;

    _camera->stopCapture();
    _camera->setCenterROI(width, height);
    _camera->startCapture();
}



void BaseCameraController::publishImage(ros::Time timestamp) {
    sensor_msgs::CameraInfoPtr
        ci(new sensor_msgs::CameraInfo(_camera->getCameraInfo()));

    try {
        auto thermalMat = _camera->getImageMatrix();
        if (flip)  {
          cv::flip(thermalMat, thermalMat, 0);
          cv::flip(thermalMat, thermalMat, 1);
        }
        _cvImage.image = thermalMat;
        _cvImage.encoding = _camera->getEncoding();
        _cvImage.header.stamp = timestamp;
        _cvImage.header.seq = _seq;
        _cvImage.header.frame_id = frame_id;

        auto publishedImage = _cvImage.toImageMsg();

        ci->header.stamp = publishedImage->header.stamp;
        _imagePublisher.publish(publishedImage, ci);

        _seq++;
    } catch(exception e) {
        // just don't publish this frame
        std::cout << "Publish exception" << std::endl;
    }
}
