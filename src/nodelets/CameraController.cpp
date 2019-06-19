#include <pluginlib/class_list_macros.h>
#include <fstream>
#include "flir_boson_ethernet/CameraController.h"

PLUGINLIB_EXPORT_CLASS(flir_boson_ethernet::CameraController, nodelet::Nodelet)

using namespace cv;
using namespace flir_boson_ethernet;

CameraController::CameraController() : _cvImage()
{
}

CameraController::~CameraController()
{
    if(_camera) {
        _camera->closeCamera();
        delete _camera;
    }
}

void CameraController::onInit()
{
    nh = getNodeHandle();
    pnh = getPrivateNodeHandle();
    
    it = std::shared_ptr<image_transport::ImageTransport>(new image_transport::ImageTransport(nh));
    _imagePublisher = it->advertiseCamera("image_raw", 1);

    bool exit = false;

    std::string ip, cameraInfoStr;

    pnh.param<std::string>("frame_id", frame_id, "boson_camera");
    pnh.param<std::string>("ip_addr", ip, "");
    pnh.param<float>("frame_rate", _frameRate, 60.0);
    pnh.param<std::string>("video_mode", video_mode_str, "RAW16");
    pnh.param<bool>("zoon_enable", zoom_enable, false);
    // pnh.param<std::string>("sensor_type", sensor_type_str, "Boson_640");
    pnh.param<std::string>("camera_info_url", cameraInfoStr, "");

    ROS_INFO("flir_boson_ethernet - Got frame_id: %s.", frame_id.c_str());
    ROS_INFO("flir_boson_ethernet - Got IP: %s.", ip.c_str());
    ROS_INFO("flir_boson_ethernet - Got frame rate: %f.", _frameRate);
    ROS_INFO("flir_boson_ethernet - Got video mode: %s.", video_mode_str.c_str());
    ROS_INFO("flir_boson_ethernet - Got zoom enable: %s.", (zoom_enable ? "true" : "false"));
    // ROS_INFO("flir_boson_ethernet - Got sensor type: %s.", sensor_type_str.c_str());
    ROS_INFO("flir_boson_ethernet - Got camera_info_url: %s.", cameraInfoStr.c_str());

    if (video_mode_str == "RAW16")
    {
        _videoMode = Encoding::RAW16;
    }
    else if (video_mode_str == "YUV")
    {
        _videoMode = Encoding::YUV;
    }
    else
    {
        exit = true;
        ROS_ERROR("flir_boson_ethernet - Invalid video_mode value provided. Exiting.");
    }

    EthernetCameraInfo info;
    info.ip = ip;
    info.camInfoPath = cameraInfoStr;
    info.width = 800;
    info.height = 600;
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

    capture_timer = nh.createTimer(ros::Duration(1.0 / _frameRate),
                    boost::bind(&CameraController::captureAndPublish, this, _1));
}

void CameraController::captureAndPublish(const ros::TimerEvent &evt)
{
    // Size size(width, height);

    sensor_msgs::CameraInfoPtr
        ci(new sensor_msgs::CameraInfo(_camera->getCameraInfo()));

    // if (_videoMode == RAW16)
    // {
    //     // -----------------------------
    //     // RAW16 DATA
    //     // agcBasicLinear(thermal16, &thermal16_linear, height, width);

    //     // Display thermal after 16-bits AGC... will display an image
    //     if (!zoom_enable)
    //     {
    //         // Threshold using Otsu's method, then use the result as a mask on the original image
    //         Mat mask_mat, masked_img;
    //         threshold(thermal16_linear, mask_mat, 0, 255, CV_THRESH_BINARY | CV_THRESH_OTSU);
    //         thermal16_linear.copyTo(masked_img, mask_mat);

    //         // Normalize the pixel values to the range [0, 1] then raise to power (gamma). Then convert back for display.
    //         Mat d_out_img, norm_image, d_norm_image, gamma_corrected_image, d_gamma_corrected_image;
    //         double gamma = 0.8;
    //         masked_img.convertTo(d_out_img, CV_64FC1);
    //         normalize(d_out_img, d_norm_image, 0, 1, NORM_MINMAX, CV_64FC1);
    //         pow(d_out_img, gamma, d_gamma_corrected_image);
    //         d_gamma_corrected_image.convertTo(gamma_corrected_image, CV_8UC1);
    //         normalize(gamma_corrected_image, gamma_corrected_image, 0, 255, NORM_MINMAX, CV_8UC1);

    //         // Apply top hat filter
    //         int erosion_size = 5;
    //         Mat top_hat_img, kernel = getStructuringElement(MORPH_ELLIPSE,
    //                                                         Size(2 * erosion_size + 1, 2 * erosion_size + 1));
    //         morphologyEx(gamma_corrected_image, top_hat_img, MORPH_TOPHAT, kernel);

    //         cv_img.image = thermal16_linear;
    //         cv_img.header.stamp = ros::Time::now();
    //         cv_img.header.frame_id = frame_id;
    //         cv_img.encoding = "mono8";
    //         pub_image = cv_img.toImageMsg();

    //         ci->header.stamp = pub_image->header.stamp;
    //         image_pub.publish(pub_image, ci);
    //     }
    //     else
    //     {
    //         resize(thermal16_linear, thermal16_linear_zoom, size);

    //         cv_img.image = thermal16_linear_zoom;
    //         cv_img.header.stamp = ros::Time::now();
    //         cv_img.header.frame_id = frame_id;
    //         cv_img.encoding = "mono8";
    //         pub_image = cv_img.toImageMsg();

    //         ci->header.stamp = pub_image->header.stamp;
    //         image_pub.publish(pub_image, ci);
    //     }
    // }
    // else // Video is in 8 bits YUV
    // {
        // ---------------------------------
        // DATA in YUV
        auto thermalMat = _camera->getImageMatrix();
        _cvImage.image = thermalMat;
        _cvImage.encoding = "rgb8";
        _cvImage.header.stamp = ros::Time::now();
        _cvImage.header.frame_id = frame_id;

        _publishedImage = _cvImage.toImageMsg();

        ci->header.stamp = _publishedImage->header.stamp;
        _imagePublisher.publish(_publishedImage, ci);
    // }
}
