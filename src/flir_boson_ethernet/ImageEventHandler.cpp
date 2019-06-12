#include "flir_boson_ethernet/ImageEventHandler.h"

ImageEventHandler::ImageEventHandler(CameraPtr pCam) {
    // Retrieve device serial number
    INodeMap & nodeMap = pCam->GetTLDeviceNodeMap();
    m_deviceSerialNumber = "";
    CStringPtr ptrDeviceSerialNumber = nodeMap.GetNode("DeviceSerialNumber");
    if (IsAvailable(ptrDeviceSerialNumber) && IsReadable(ptrDeviceSerialNumber))
    {
        m_deviceSerialNumber = ptrDeviceSerialNumber->GetValue();
    }

    m_resultImage = nullptr;
    // m_imageBuffer = nullptr;
    startTime = Clock::now();
}

ImageEventHandler::~ImageEventHandler() {}

ImageInfo ImageEventHandler::GetImageInfo() {
    // need to wait for the first image to be received
    // (event based so not on this thread)
    while(m_resultImage == nullptr && ros::ok()) {}

    return ImageInfo {m_resultImage->GetWidth(), m_resultImage->GetHeight(),
                m_resultImage->GetBufferSize()};
}

static int framesPerSecond = 0;
void ImageEventHandler::OnImageEvent(ImagePtr image) {
    // Check image retrieval status
    if (image->IsIncomplete()) {
        return;
    }
    m_resultImage = image->Convert(PixelFormat_RGB8, HQ_LINEAR);

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

    return m_resultImage->GetData();
}
