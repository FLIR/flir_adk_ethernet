/******************************************************************************/
/*                                                                            */
/*  Copyright (C) 2018, FLIR Systems                                          */
/*  All rights reserved.                                                      */
/*                                                                            */
/******************************************************************************/
#include <pluginlib/class_list_macros.h>
#include <fstream>
#include "flir_adk_ethernet/CameraController.h"

PLUGINLIB_EXPORT_CLASS(flir_adk_ethernet::CameraController, nodelet::Nodelet)

using namespace cv;
using namespace flir_adk_ethernet;

CameraController::CameraController() : BaseCameraController()
{
}

CameraController::~CameraController()
{
}

void CameraController::setupFramePublish() {
    pnh.param<float>("frame_rate", _frameRate, 60.0);
    ROS_INFO("flir_adk_ethernet - Got frame rate: %f.", _frameRate);

    capture_timer = nh.createTimer(ros::Duration(1.0 / _frameRate),
        boost::bind(&CameraController::captureAndPublish, this, _1));
}

void CameraController::captureAndPublish(const ros::TimerEvent &evt)
{
    publishImage(ros::Time::now());
}
