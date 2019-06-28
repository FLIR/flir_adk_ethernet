#include "flir_boson_ethernet/ImageEventHandler.h"

using namespace flir_boson_ethernet;

ImageEventHandler::ImageEventHandler(std::shared_ptr<CameraWrapper> pCam,
    PixelFormatEnums format) :
    _pCam(pCam),
    _format(format)
{
    // Retrieve device serial number
    INodeMap & nodeMap = pCam->GetTLDeviceNodeMap();
    m_deviceSerialNumber = "";
    CStringPtr ptrDeviceSerialNumber = nodeMap.GetNode("DeviceSerialNumber");
    if (IsAvailable(ptrDeviceSerialNumber) && IsReadable(ptrDeviceSerialNumber))
    {
        m_deviceSerialNumber = ptrDeviceSerialNumber->GetValue();
    }

    m_resultImage = nullptr;
    startTime = Clock::now();
}

ImageEventHandler::ImageEventHandler(const ImageEventHandler& handler) :
    ImageEventHandler(handler._pCam, handler._format)
{

}

ImageEventHandler::~ImageEventHandler() {}

// must be called after camera acquisition has been enabled
void ImageEventHandler::Init() {
    // need to wait for the first image to be received
    // (event based so not on this thread)
    while(m_resultImage == nullptr && ros::ok()) {}
}

ImageInfo ImageEventHandler::GetImageInfo() {
    return ImageInfo {m_resultImage->GetWidth(), m_resultImage->GetHeight(),
                m_resultImage->GetBufferSize()};
}

// int framesPerSecond = 0;
void ImageEventHandler::OnImageEvent(ImagePtr image) {
    // Check image retrieval status
    if (image->IsIncomplete()) {
        return;
    }
    m_mutex.lock();
    m_resultImage = image->Convert(_format, HQ_LINEAR);
    m_lastTimeStamp = image->GetTimeStamp();
    m_mutex.unlock();

    // optional: uncomment to print out actual capture rate of camera
    // this was useful to debug bandwidth issues for multiple high-resoultion
    // cameras
    // framesPerSecond++;
    // if(std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now() 
    //     - startTime).count() >= 1000) {
    //     startTime = Clock::now();
    //     std::cout << "FPS: " << framesPerSecond << std::endl;
    //     framesPerSecond = 0;
    // }
}

void *ImageEventHandler::GetImageData() {
    if(m_resultImage == nullptr) {
        throw "No image has been received";
    }

    m_mutex.lock();
    m_mutex.unlock();
    return m_resultImage->GetData();
}

uint64_t ImageEventHandler::GetCaptureTime() {
    return m_lastTimeStamp;
}

void ImageEventHandler::setPixelFormat(PixelFormatEnums format) {
    _format = format;
}