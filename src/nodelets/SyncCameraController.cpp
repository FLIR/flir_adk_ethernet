#include <pluginlib/class_list_macros.h>
#include "flir_boson_ethernet/SyncCameraController.h"

PLUGINLIB_EXPORT_CLASS(flir_boson_ethernet::SyncCameraController, nodelet::Nodelet)

using namespace cv;
using namespace flir_boson_ethernet;

SyncCameraController::SyncCameraController() : _cvImage()
{
}

SyncCameraController::~SyncCameraController()
{
    if(_camera) {
        _camera->closeCamera();
        delete _camera;
    }
}

void SyncCameraController::onInit()
{
    nh = getNodeHandle();
    pnh = getPrivateNodeHandle();
    
    it = std::shared_ptr<image_transport::ImageTransport>(new image_transport::ImageTransport(nh));
    _imagePublisher = it->advertiseCamera("image_raw", 1);

    bool exit = false;

    std::string ip, cameraInfoStr;

    pnh.param<std::string>("frame_id", frame_id, "boson_camera");
    pnh.param<std::string>("ip_addr", ip, "");
    pnh.param<std::string>("camera_info_url", cameraInfoStr, "");

    ROS_INFO("flir_boson_ethernet - Got frame_id: %s.", frame_id.c_str());
    ROS_INFO("flir_boson_ethernet - Got IP: %s.", ip.c_str());
    ROS_INFO("flir_boson_ethernet - Got camera_info_url: %s.", cameraInfoStr.c_str());

    EthernetCameraInfo info;
    info.ip = ip;
    info.camInfoPath = cameraInfoStr;
    info.width = 800;
    info.height = 600;
    SystemWrapper sys(System::GetInstance());
    _camera = new EthernetCamera(info, sys, nh);

    if (!exit) {
        exit = !_camera->openCamera() || exit;
    }

    if (exit)
    {
        ros::shutdown();
        return;
    }
    
    _sub = nh.subscribe<std_msgs::Time>("image_sync", 1, 
        boost::bind(&SyncCameraController::publishImage, this, _1));
}

void SyncCameraController::publishImage(const std_msgs::Time::ConstPtr& message)
{
    sensor_msgs::CameraInfoPtr
        ci(new sensor_msgs::CameraInfo(_camera->getCameraInfo()));

    auto thermalMat = _camera->getImageMatrix();
    uint64_t actualCaptureTime = _camera->getActualTimestamp();
    _cvImage.image = thermalMat;
    _cvImage.encoding = "rgb8";
    _cvImage.header.stamp = message->data;
    // _cvImage.header.stamp = timeFromNSec(actualCaptureTime);
    _cvImage.header.frame_id = frame_id;

    auto publishedImage = _cvImage.toImageMsg();

    ci->header.stamp = publishedImage->header.stamp;
    _imagePublisher.publish(publishedImage, ci);
}

ros::Time SyncCameraController::timeFromNSec(uint64_t nsec) {
    return ros::Time(nsec / 1e9, nsec % (uint32_t)1e9);
}
