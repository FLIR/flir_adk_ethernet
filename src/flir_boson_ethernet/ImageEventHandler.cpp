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
    m_isValid = false;
}
ImageEventHandler::~ImageEventHandler() {}

void ImageEventHandler::Init(uint8_t *buffer) {
    m_bufferStart = buffer;
}

ImageInfo ImageEventHandler::GetImageInfo() {
    m_imageWriteMutex.lock();
    m_imageWriteMutex.unlock();
    return m_imageInfo;
}

void ImageEventHandler::OnImageEvent(ImagePtr image) {
    // Check image retrieval status
    if (image->IsIncomplete())
    {
        return;
    }
    else
    {
        m_imageWriteMutex.lock();

        // Convert image to mono 8
        ImagePtr resultImage = image->Convert(PixelFormat_Mono8, HQ_LINEAR);
        if(!m_isValid) {
            m_imageInfo = ImageInfo {image->GetWidth(), image->GetHeight(),
                image->GetBufferSize()};
            m_isValid = true;
        } else {
            memcpy(m_bufferStart, resultImage->GetData(), resultImage->GetBufferSize());
        }
        m_imageWriteMutex.unlock();
    }
}

bool ImageEventHandler::IsValid() {
    return m_isValid;
}
