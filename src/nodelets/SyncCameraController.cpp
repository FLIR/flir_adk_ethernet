#include <pluginlib/class_list_macros.h>
#include "flir_boson_ethernet/SyncCameraController.h"

PLUGINLIB_EXPORT_CLASS(flir_boson_ethernet::SyncCameraController, nodelet::Nodelet)

using namespace cv;
using namespace flir_boson_ethernet;

SyncCameraController::SyncCameraController() : BaseCameraController()
{
}

SyncCameraController::~SyncCameraController()
{
    
}

void SyncCameraController::setupExtraPubSub() {
    _timePublisher = nh.advertise<flir_boson_ethernet::MultiTimeHeader>(
        "actual_timestamp", 1);
}

void SyncCameraController::setupFramePublish() {
    // listen to image_sync to get indicator to publish image
    _sub = nh.subscribe<std_msgs::Time>("image_sync", 1, 
        boost::bind(&SyncCameraController::publishImage, this, _1));
}

void SyncCameraController::publishImage(const std_msgs::Time::ConstPtr& message)
{
    BaseCameraController::publishImage(message->data);

    // publish the camera-reported timestamp
    uint64_t actualCaptureTime = _camera->getActualTimestamp();
    MultiTimeHeader timeHeader;
    timeHeader.header.frame_id = frame_id;
    timeHeader.header.seq = _cvImage.header.seq;
    timeHeader.header.stamp = _cvImage.header.stamp;
    timeHeader.actual_stamp = timeFromNSec(actualCaptureTime);
    _timePublisher.publish(timeHeader);
}

ros::Time SyncCameraController::timeFromNSec(uint64_t nsec) {
    return ros::Time(nsec / 1e9, nsec % (uint32_t)1e9);
}
