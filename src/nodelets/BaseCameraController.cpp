#include <pluginlib/class_list_macros.h>
#include "flir_boson_ethernet/Util.h"
#include "flir_boson_ethernet/BaseCameraController.h"

using namespace cv;
using namespace flir_boson_ethernet;

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

    ROS_INFO("flir_boson_ethernet - Got frame_id: %s.", frame_id.c_str());
    ROS_INFO("flir_boson_ethernet - Got IP: %s.", ip.c_str());
    ROS_INFO("flir_boson_ethernet - Got camera_info_url: %s.", 
        cameraInfoStr.c_str());
    ROS_INFO("flir_boson_ethernet - Got video_format: %s.", formatStr.c_str());
    ROS_INFO("flir_boson_ethernet - Got camera_type: %s.", camType.c_str());
    ROS_INFO("flir_boson_ethernet - Got width: %d.", width);
    ROS_INFO("flir_boson_ethernet - Got height: %d.", height);
    ROS_INFO("flir_boson_ethernet - Got xOffset: %d.", xOffset);
    ROS_INFO("flir_boson_ethernet - Got yOffset: %d.", yOffset);

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

    _getNodeListener = nh.subscribe<std_msgs::String>("get_node", 10,
        boost::bind(&BaseCameraController::getNode, this, _1));
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

void BaseCameraController::getNode(const std_msgs::StringConstPtr& msg) {
    std::string result = _camera->getNodeValue(msg->data);
    if(result.empty()) {
        ROS_ERROR("Could not get value for node: %s", msg->data);
        return;
    }

    ROS_INFO("Value for node %s is: %s", msg->data.c_str(), result.c_str());
}

void BaseCameraController::publishImage(ros::Time timestamp) {
        sensor_msgs::CameraInfoPtr
        ci(new sensor_msgs::CameraInfo(_camera->getCameraInfo()));

    auto thermalMat = _camera->getImageMatrix();
    _cvImage.image = thermalMat;
    _cvImage.encoding = _camera->getEncoding();
    _cvImage.header.stamp = timestamp;
    _cvImage.header.seq = _seq;
    _cvImage.header.frame_id = frame_id;

    auto publishedImage = _cvImage.toImageMsg();

    ci->header.stamp = publishedImage->header.stamp;
    _imagePublisher.publish(publishedImage, ci);

    _seq++;
}