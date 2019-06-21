#include <pluginlib/class_list_macros.h>
#include <fstream>
#include "flir_boson_ethernet/CameraController.h"

PLUGINLIB_EXPORT_CLASS(flir_boson_ethernet::CameraController, nodelet::Nodelet)

using namespace cv;
using namespace flir_boson_ethernet;

CameraController::CameraController() : BaseCameraController()
{
}

CameraController::~CameraController()
{
    if(_camera) {
        _camera->closeCamera();
        delete _camera;
    }
}

void CameraController::setupFramePublish() {
    pnh.param<float>("frame_rate", _frameRate, 60.0);
    ROS_INFO("flir_boson_ethernet - Got frame rate: %f.", _frameRate);

    capture_timer = nh.createTimer(ros::Duration(1.0 / _frameRate),
        boost::bind(&CameraController::captureAndPublish, this, _1));
}

void CameraController::captureAndPublish(const ros::TimerEvent &evt)
{
    // Size size(width, height);

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
        publishImage(ros::Time::now());
    // }
}
