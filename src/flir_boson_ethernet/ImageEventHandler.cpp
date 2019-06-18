#include "flir_boson_ethernet/ImageEventHandler.h"

using namespace flir_boson_ethernet;

ImageEventHandler::ImageEventHandler(CameraWrapper *pCam) {
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

static int framesPerSecond = 0;
void ImageEventHandler::OnImageEvent(ImagePtr image) {
    // Check image retrieval status
    if (image->IsIncomplete()) {
        return;
    }
    m_mutex.lock();
    m_resultImage = image->Convert(PixelFormat_RGB8, HQ_LINEAR);
    m_lastTimeStamp = image->GetTimeStamp();
    m_mutex.unlock();

    framesPerSecond++;
    if(std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now() 
        - startTime).count() >= 1000) {
        startTime = Clock::now();
        std::cout << "FPS: " << framesPerSecond << std::endl;
        framesPerSecond = 0;
    }

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