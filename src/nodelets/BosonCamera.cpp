/*
 * Copyright © 2019 AutonomouStuff, LLC
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this
 * software and associated documentation files (the “Software”), to deal in the Software
 * without restriction, including without limitation the rights to use, copy, modify,
 * merge, publish, distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all copies
 * or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
 * INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
 * PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE
 * LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE
 * OR OTHER DEALINGS IN THE SOFTWARE.
 */

#include <pluginlib/class_list_macros.h>
#include "flir_boson_ethernet/BosonCamera.h"

PLUGINLIB_EXPORT_CLASS(flir_boson_ethernet::BosonCamera, nodelet::Nodelet)

using namespace cv;
using namespace flir_boson_ethernet;


gcstring GetDottedAddress( int64_t value )
{
    // Helper function for formatting IP Address into the following format
    // x.x.x.x
    unsigned int inputValue = static_cast<unsigned int>( value );
    ostringstream convertValue;
    convertValue << ((inputValue & 0xFF000000) >> 24);
    convertValue << ".";
    convertValue << ((inputValue & 0x00FF0000) >> 16);
    convertValue << ".";
    convertValue << ((inputValue & 0x0000FF00) >> 8);
    convertValue << ".";
    convertValue << (inputValue & 0x000000FF);
    return convertValue.str().c_str();
}

BosonCamera::BosonCamera() : cv_img()
{
}

BosonCamera::~BosonCamera()
{
    delete pCam;
    delete buffer_start;
    closeCamera();
}

void BosonCamera::onInit()
{
    nh = getNodeHandle();
    pnh = getPrivateNodeHandle();
    camera_info = std::shared_ptr<camera_info_manager::CameraInfoManager>(
        new camera_info_manager::CameraInfoManager(nh));
    it = std::shared_ptr<image_transport::ImageTransport>(new image_transport::ImageTransport(nh));
    image_pub = it->advertiseCamera("image_raw", 1);

    bool exit = false;

    pnh.param<std::string>("frame_id", frame_id, "boson_camera");
    pnh.param<std::string>("ip_addr", ip_addr, "169.254.87.157");
    pnh.param<float>("frame_rate", frame_rate, 60.0);
    pnh.param<std::string>("video_mode", video_mode_str, "RAW16");
    pnh.param<bool>("zoon_enable", zoom_enable, false);
    // pnh.param<std::string>("sensor_type", sensor_type_str, "Boson_640");
    pnh.param<std::string>("camera_info_url", camera_info_url, "");

    ROS_INFO("flir_boson_ethernet - Got frame_id: %s.", frame_id.c_str());
    ROS_INFO("flir_boson_ethernet - Got IP: %s.", ip_addr.c_str());
    ROS_INFO("flir_boson_ethernet - Got frame rate: %f.", frame_rate);
    ROS_INFO("flir_boson_ethernet - Got video mode: %s.", video_mode_str.c_str());
    ROS_INFO("flir_boson_ethernet - Got zoom enable: %s.", (zoom_enable ? "true" : "false"));
    // ROS_INFO("flir_boson_ethernet - Got sensor type: %s.", sensor_type_str.c_str());
    ROS_INFO("flir_boson_ethernet - Got camera_info_url: %s.", camera_info_url.c_str());

    if (video_mode_str == "RAW16")
    {
        video_mode = RAW16;
    }
    else if (video_mode_str == "YUV")
    {
        video_mode = YUV;
    }
    else
    {
        exit = true;
        ROS_ERROR("flir_boson_ethernet - Invalid video_mode value provided. Exiting.");
    }

    // if (sensor_type_str == "Boson_320" ||
    //     sensor_type_str == "boson_320")
    // {
    //     sensor_type = Boson320;
    //     camera_info->setCameraName("Boson320");
    // }
    // else if (sensor_type_str == "Boson_640" ||
    //          sensor_type_str == "boson_640")
    // {
    //     sensor_type = Boson640;
    //     camera_info->setCameraName("Boson640");
    // }
    // else
    // {
    //     exit = true;
    //     ROS_ERROR("flir_boson_ethernet - Invalid sensor_type value provided. Exiting.");
    // }

    if (!exit) {
        exit = !openCamera() || exit;
    }

    ROS_INFO("OPENED CAMERA");
    if (exit)
    {
        ROS_INFO("SHUTTING DOWN");
        ros::shutdown();
        return;
    }
    else
    {
        capture_timer = nh.createTimer(ros::Duration(1.0 / frame_rate),
                                       boost::bind(&BosonCamera::captureAndPublish, this, _1));
    }
}

// AGC Sample ONE: Linear from min to max.
// Input is a MATRIX (height x width) of 16bits. (OpenCV mat)
// Output is a MATRIX (height x width) of 8 bits (OpenCV mat)
void BosonCamera::agcBasicLinear(const Mat &input_16,
                                 Mat *output_8,
                                 const int &height,
                                 const int &width)
{
    
}

bool BosonCamera::openCamera()
{
    system = System::GetInstance();
    CameraList camList = system->GetCameras();
    const unsigned int numCameras = camList.GetSize();

    if(numCameras == 0) {
        ROS_ERROR("flir_boson_ethernet - ERROR : NO_CAMERAS. No cameras found");
        camList.Clear();
        system->ReleaseInstance();
        return false;
    }

    findMatchingCamera(camList, numCameras);
    
    if(!pCam.IsValid()) {
        ROS_ERROR("flir_boson_ethernet - ERROR : OPEN. No device matches ip_addr: %s", ip_addr.c_str());
        return false;
    }
    pCam->Init();

    if (!setImageAcquisition())
    {
        ROS_ERROR("flir_boson_ethernet - ERROR : CAMERA_ACQUISITION. Cannot set image acquisition.");
        return false;
    }

    if(!initCamera()) {
        ROS_ERROR("flir_boson_ethernet - ERROR : INIT. Could not initialize camera");
        return false;
    }

    try {
        // get first image to set width, height and buffer size
        ImagePtr image = pCam->GetNextImage();
        width = image->GetWidth();
        height = image->GetHeight();
        buffer_start = new uint8_t[image->GetBufferSize()];
        ROS_INFO("Camera info - Width: %d, Height: %d, Image Size: %d", width, height, image->GetBufferSize());
    } catch(Spinnaker::Exception e) {
        ROS_ERROR("flir_boson_ethernet - ERROR : GET_CONFIGURATION. Cannot get image for setting dimensions");
        return false;
    }

    initOpenCVBuffers();
    setCameraInfo();

    return true;
}

void BosonCamera::findMatchingCamera(CameraList camList, const unsigned int numCams) {
    gcstring deviceIPAddress = "0.0.0.0";

    for (unsigned int i = 0; i < numCams; i++)
    {
        // Select camera
        CameraPtr cam = camList.GetByIndex(i);
        INodeMap &nodeMapTLDevice = cam->GetTLDeviceNodeMap();

        CIntegerPtr ptrIPAddress = nodeMapTLDevice.GetNode("GevDeviceIPAddress");
        if (IsAvailable(ptrIPAddress) && IsReadable(ptrIPAddress)) {
            deviceIPAddress = GetDottedAddress(ptrIPAddress->GetValue());
        }
        if(deviceIPAddress == ip_addr) {
            CStringPtr modelName = nodeMapTLDevice.GetNode("DeviceModelName");
            ROS_INFO("Found matching camera %s", modelName->ToString().c_str());
            pCam = cam;
        }
    }
}

bool BosonCamera::initCamera() {
    try {
        pCam->BeginAcquisition();
        ROS_INFO("Camera acquisition started...");
        return true;
    } catch(Spinnaker::Exception e) {
        return false;
    }
}

bool BosonCamera::setImageAcquisition() {
    INodeMap &nodeMapTLDevice = pCam->GetTLDeviceNodeMap();
    INodeMap &nodeMap = pCam->GetNodeMap();

    CEnumerationPtr ptrAcquisitionMode = nodeMap.GetNode("AcquisitionMode");
    if (!IsAvailable(ptrAcquisitionMode) || !IsWritable(ptrAcquisitionMode)) 
    {
        ROS_ERROR("Unable to set acquisition mode. Aborting...");
        return false;
    }

    // Retrieve entry node from enumeration node
    CEnumEntryPtr ptrAcquisitionModeContinuous = ptrAcquisitionMode->GetEntryByName("Continuous");
    if (!IsAvailable(ptrAcquisitionModeContinuous) || !IsReadable(ptrAcquisitionModeContinuous))
    {
        ROS_ERROR("Unable to set acquisition mode to continuous (entry retrieval). Aborting...");
        return false;
    }

    // Retrieve integer value from entry node
    const int64_t acquisitionModeContinuous = ptrAcquisitionModeContinuous->GetValue();

    // Set integer value from entry node as new value of enumeration node
    ptrAcquisitionMode->SetIntValue(acquisitionModeContinuous);

    ROS_INFO("Acquisition mode set to continuous...");
    return true;
}

void BosonCamera::initOpenCVBuffers() {
    // Declarations for RAW16 representation
    // Will be used in case we are reading RAW16 format
    // Boson320 , Boson 640
    // OpenCV input buffer  : Asking for all info: two bytes per pixel (RAW16)  RAW16 mode`
    thermal16 = Mat(height, width, CV_16U, reinterpret_cast<void*>(buffer_start));
    // OpenCV output buffer : Data used to display the video
    thermal16_linear = Mat(height, width, CV_8U, 1);

    // Declarations for 8bits YCbCr mode
    // Will be used in case we are reading YUV format
    // Boson320, 640 :  4:2:0
    int luma_height = height + height / 2;
    int luma_width = width;
    int color_space = CV_8UC1;

    // Declarations for Zoom representation
    // Will be used or not depending on program arguments
    thermal_luma = Mat(luma_height, luma_width, color_space, reinterpret_cast<void*>(buffer_start)); // OpenCV input buffer
    // OpenCV output buffer , BGR -> Three color spaces :
    // (640 - 640 - 640 : p11 p21 p31 .... / p12 p22 p32 ..../ p13 p23 p33 ...)
    thermal_rgb = Mat(height, width, CV_8UC3, 1);
}

void BosonCamera::setCameraInfo() {
    INodeMap &nodeMapTLDevice = pCam->GetTLDeviceNodeMap();
    CStringPtr modelName = nodeMapTLDevice.GetNode("DeviceModelName");

    camera_info->setCameraName(modelName->GetValue().c_str());
    if (camera_info->validateURL(camera_info_url)) {
        camera_info->loadCameraInfo(camera_info_url);
    } else {
        ROS_INFO("flir_boson_ethernet - camera_info_url could not be validated. Publishing with unconfigured camera.");
    }
}

bool BosonCamera::closeCamera()
{
    // Finish loop. Exiting.
    // Deactivate streaming
    // int type = bufferinfo.type;
    // if (ioctl(fd, VIDIOC_STREAMOFF, &type) < 0)
    // {
    //     ROS_ERROR("flir_boson_ethernet - VIDIOC_STREAMOFF error. Failed to disable streaming on the camera.");
    //     return false;
    // };

    // close(fd);
    if(pCam != nullptr) {
        pCam->EndAcquisition();
        pCam->DeInit();
    }

    return true;
}

void BosonCamera::captureAndPublish(const ros::TimerEvent &evt)
{
    Size size(width, height);

    sensor_msgs::CameraInfoPtr
        ci(new sensor_msgs::CameraInfo(camera_info->getCameraInfo()));

    ImagePtr resultImage = pCam->GetNextImage();
    memcpy(buffer_start, resultImage->GetData(), resultImage->GetBufferSize());
    ci->header.frame_id = std::to_string(resultImage->GetFrameID());
    // // Put the buffer in the incoming queue.
    // if (ioctl(fd, VIDIOC_QBUF, &bufferinfo) < 0)
    // {
    //     ROS_ERROR("flir_boson_ethernet - VIDIOC_QBUF error. Failed to queue the image buffer.");
    //     return;
    // }

    // // The buffer's waiting in the outgoing queue.
    // if (ioctl(fd, VIDIOC_DQBUF, &bufferinfo) < 0)
    // {
    //     ROS_ERROR("flir_boson_ethernet - VIDIOC_DQBUF error. Failed to dequeue the image buffer.");
    //     return;
    // }

    if (video_mode == RAW16)
    {
        // -----------------------------
        // RAW16 DATA
        // agcBasicLinear(thermal16, &thermal16_linear, height, width);

        // Display thermal after 16-bits AGC... will display an image
        if (!zoom_enable)
        {
            // Threshold using Otsu's method, then use the result as a mask on the original image
            Mat mask_mat, masked_img;
            threshold(thermal16_linear, mask_mat, 0, 255, CV_THRESH_BINARY | CV_THRESH_OTSU);
            thermal16_linear.copyTo(masked_img, mask_mat);

            // Normalize the pixel values to the range [0, 1] then raise to power (gamma). Then convert back for display.
            Mat d_out_img, norm_image, d_norm_image, gamma_corrected_image, d_gamma_corrected_image;
            double gamma = 0.8;
            masked_img.convertTo(d_out_img, CV_64FC1);
            normalize(d_out_img, d_norm_image, 0, 1, NORM_MINMAX, CV_64FC1);
            pow(d_out_img, gamma, d_gamma_corrected_image);
            d_gamma_corrected_image.convertTo(gamma_corrected_image, CV_8UC1);
            normalize(gamma_corrected_image, gamma_corrected_image, 0, 255, NORM_MINMAX, CV_8UC1);

            // Apply top hat filter
            int erosion_size = 5;
            Mat top_hat_img, kernel = getStructuringElement(MORPH_ELLIPSE,
                                                            Size(2 * erosion_size + 1, 2 * erosion_size + 1));
            morphologyEx(gamma_corrected_image, top_hat_img, MORPH_TOPHAT, kernel);

            cv_img.image = thermal16_linear;
            cv_img.header.stamp = ros::Time::now();
            cv_img.header.frame_id = frame_id;
            cv_img.encoding = "mono8";
            pub_image = cv_img.toImageMsg();

            ci->header.stamp = pub_image->header.stamp;
            image_pub.publish(pub_image, ci);
        }
        else
        {
            resize(thermal16_linear, thermal16_linear_zoom, size);

            cv_img.image = thermal16_linear_zoom;
            cv_img.header.stamp = ros::Time::now();
            cv_img.header.frame_id = frame_id;
            cv_img.encoding = "mono8";
            pub_image = cv_img.toImageMsg();

            ci->header.stamp = pub_image->header.stamp;
            image_pub.publish(pub_image, ci);
        }
    }
    else // Video is in 8 bits YUV
    {
        // ---------------------------------
        // DATA in YUV
        cvtColor(thermal_luma, thermal_rgb, COLOR_YUV2GRAY_I420, 0);

        cv_img.image = thermal_rgb;
        cv_img.encoding = "mono8";
        cv_img.header.stamp = ros::Time::now();
        cv_img.header.frame_id = frame_id;
        pub_image = cv_img.toImageMsg();

        ci->header.stamp = pub_image->header.stamp;
        image_pub.publish(pub_image, ci);
    }
}
