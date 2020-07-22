/******************************************************************************/
/*                                                                            */
/*  Copyright (C) 2018, FLIR Systems                                          */
/*  All rights reserved.                                                      */
/*                                                                            */
/******************************************************************************/
#include "flir_adk_ethernet/Util.h"
#include "flir_adk_ethernet/EthernetCamera.h"

using namespace cv;
using namespace flir_adk_ethernet;

EthernetCamera::EthernetCamera(EthernetCameraInfo info, 
        std::shared_ptr<SystemWrapper> sys,
        ros::NodeHandle nh) :
    _ipAddr(info.ip), 
    _cameraInfoPath(info.camInfoPath),
    _width(info.width),
    _height(info.height),
    _xOffset(info.xOffset),
    _yOffset(info.yOffset),
    _camType(info.camType),
    _system(sys),
    _bufferStart(nullptr),
    _selectedFormat(ImageFormat(info.pixelFormat))
{
    _cameraInfo = std::shared_ptr<camera_info_manager::CameraInfoManager>(
        new camera_info_manager::CameraInfoManager(nh));
}

EthernetCamera::~EthernetCamera() {
    if(_bufferStart) {
        delete _bufferStart;
    }
    closeCamera();
    _pCam.reset();
}

void EthernetCamera::agcBasicLinear(const Mat &input_16,
                                    Mat *output_8,
                                    const int &height,
                                    const int &width)
{
    // unimplemented for now
}

bool EthernetCamera::openCamera()
{
    CameraListWrapper camList = _system->GetCameras();
    const unsigned int numCameras = camList.GetSize();

    if(numCameras == 0) {
        ROS_ERROR("flir_adk_ethernet - ERROR : NO_CAMERAS. No cameras found");
        camList.Clear();
        _system->ReleaseInstance();
        return false;
    }

    if(!findMatchingCamera(camList, numCameras) || !_pCam->IsValid()) {
        ROS_ERROR("flir_adk_ethernet - ERROR : OPEN. No device matches ip_addr: %s", _ipAddr.c_str());
        return false;
    }
    _pCam->Init();
    initPixelFormat();

    if(!setImageInfo()) {
        ROS_ERROR("flir_adk_ethernet - ERROR : GET_CONFIGURATION. Cannot get image for setting dimensions");
        return false;
    }
    setCameraEvents();

    if (!setImageAcquisition())
    {
        ROS_ERROR("flir_adk_ethernet - ERROR : CAMERA_ACQUISITION. Cannot set image acquisition.");
        return false;
    }

    _imageHandler->Init();

    initOpenCVBuffers();
    setCameraInfo();

    return true;
} 

bool EthernetCamera::findMatchingCamera(CameraListWrapper camList, const unsigned int numCams) {
    gcstring deviceIPAddress = "0.0.0.0";

    for (unsigned int i = 0; i < numCams; i++)
    {
        // Select camera
        CameraWrapper cam = camList.GetByIndex(i);
        INodeMap &nodeMapTLDevice = cam.GetTLDeviceNodeMap();

        if((!_ipAddr.empty() && ipMatches(_ipAddr, nodeMapTLDevice)) ||
           (_ipAddr.empty() && camTypeMatches(_camType, nodeMapTLDevice)))
        {
            CStringPtr modelName = nodeMapTLDevice.GetNode("DeviceModelName");
            ROS_INFO("Found matching camera %s", modelName->ToString().c_str());
            _pCam = std::make_shared<CameraWrapper>(cam);
            return true;
        }
    }

    return false;
}

bool EthernetCamera::ipMatches(string ip, INodeMap& nodeMapTLDevice) {
    CIntegerPtr ptrIPAddress = nodeMapTLDevice.GetNode("GevDeviceIPAddress");
    if (IsAvailable(ptrIPAddress) && IsReadable(ptrIPAddress)) {
        return ip == GetDottedAddress(ptrIPAddress->GetValue());
    }
    return false;    
}

bool EthernetCamera::camTypeMatches(string camType, INodeMap& nodeMapTLDevice) {
    CStringPtr modelName = nodeMapTLDevice.GetNode("DeviceModelName");
    if (IsAvailable(modelName) && IsReadable(modelName)) {
        auto found = toLower(modelName->ToString().c_str()).find(_camType);
        return found != std::string::npos;
    }
    
    return false;
}

bool EthernetCamera::setImageInfo() {
    try {
        // setBinning();
        setROI(_xOffset, _yOffset, _width, _height);

        return true;
    } catch(Spinnaker::Exception e) {
        ROS_ERROR("flir_adk_ethernet - ERROR : %s", e.what());
        return false;
    }
}

void EthernetCamera::setBinning() {
    INodeMap &nodeMap = _pCam->GetNodeMap();
    CIntegerPtr hNode = nodeMap.GetNode("BinningHorizontal");
    CIntegerPtr vNode = nodeMap.GetNode("BinningVertical");
    hNode->SetValue(1);
    vNode->SetValue(1);
}

void EthernetCamera::createBuffer() {
    _imageSize = _height * _width * getPixelSize();
    _bufferStart = new uint8_t[_imageSize];
}

void EthernetCamera::resetBuffer() {
    delete [] _bufferStart;
    createBuffer();
    initOpenCVBuffers();
}

int EthernetCamera::getPixelSize() {
    return _selectedFormat.getBytesPerPixel();
}

void EthernetCamera::initPixelFormat() {
    try {
        INodeMap &nodeMap = _pCam->GetNodeMap();
        CEnumerationPtr pixelFormatNode = nodeMap.GetNode("PixelFormat");

        pixelFormatNode->SetIntValue(_selectedFormat.getValue(pixelFormatNode));
    } catch(Spinnaker::Exception e) {
        ROS_INFO("Unable to set pixel format to: %s", _selectedFormat.toString().c_str());
    }
}

bool EthernetCamera::setCenterROI(int width, int height) {
    INodeMap& nodeMap = _pCam->GetNodeMap();
    CIntegerPtr maxWidthNode = nodeMap.GetNode("WidthMax");
    CIntegerPtr maxHeightNode = nodeMap.GetNode("HeightMax");
    int maxWidth = maxWidthNode->GetValue();
    int maxHeight = maxHeightNode->GetValue();

    if(width == 0) {
        width = maxWidth;
    }
    if(height == 0) {
        height = maxHeight;
    }

    int xOffset = max(0, (maxWidth - width) / 2);
    int yOffset = max(0, (maxHeight - height) / 2);
    return setROI(xOffset, yOffset, width, height);
}

bool EthernetCamera::setROI(int xOffset, int yOffset, int width, int height) {
    // spinnaker needs offsets to be even
    xOffset = roundToEven(xOffset);
    yOffset = roundToEven(yOffset);
    INodeMap& nodeMap = _pCam->GetNodeMap();

    CIntegerPtr maxWidthNode = nodeMap.GetNode("WidthMax");
    CIntegerPtr maxHeightNode = nodeMap.GetNode("HeightMax");
    int maxWidth = maxWidthNode->GetValue();
    int maxHeight = maxHeightNode->GetValue();

    width = min(width, maxWidth - xOffset);
    height = min(height, maxHeight - yOffset);

    if(width == 0) {
        width = maxWidth - xOffset;
    }
    if(height == 0) {
        height = maxHeight - yOffset;
    }

    CIntegerPtr widthNode = nodeMap.GetNode("Width");
    CIntegerPtr heightNode = nodeMap.GetNode("Height");
    CIntegerPtr xOffNode = nodeMap.GetNode("OffsetX");
    CIntegerPtr yOffNode = nodeMap.GetNode("OffsetY");

    try {
        // give it temporary small value to ensure offsets don't overflow
        // size limits
        widthNode->SetValue(8);
        heightNode->SetValue(8);

        xOffNode->SetValue(xOffset);
        yOffNode->SetValue(yOffset);
        widthNode->SetValue(width);
        heightNode->SetValue(height);

        _xOffset = xOffset;
        _yOffset = yOffset;
        _width = width;
        _height = height;

        if(!_bufferStart) {
            createBuffer();
        } else {
            resetBuffer();
        }

        ROS_INFO("Camera info - Width: %d, Height: %d, X Offset: %d, Y Offset: %d",
            _width, _height, _xOffset, _yOffset);
        return true;
    } catch (Spinnaker::Exception e) {
        ROS_ERROR("%s", e.what());

        widthNode->SetValue(8);
        heightNode->SetValue(8);

        xOffNode->SetValue(_xOffset);
        yOffNode->SetValue(_yOffset);
        widthNode->SetValue(_width);
        heightNode->SetValue(_height);

        return false;
    }
}

void EthernetCamera::setCameraEvents() {
    _imageHandler = std::make_shared<ImageEventHandler>(
        ImageEventHandler(_pCam, _selectedFormat.getFormat()));
    _pCam->RegisterEvent(*_imageHandler);
}

bool EthernetCamera::setImageAcquisition() {
    INodeMap &nodeMapTLDevice = _pCam->GetTLDeviceNodeMap();
    INodeMap &nodeMap = _pCam->GetNodeMap();

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
    
    startCapture();

    return true;
}

void EthernetCamera::initOpenCVBuffers() {
    _thermalImageMat = Mat(_height, _width, _selectedFormat.getMatType(), 
        reinterpret_cast<void *>(_bufferStart));
}

void EthernetCamera::setCameraInfo() {
    INodeMap &nodeMapTLDevice = _pCam->GetTLDeviceNodeMap();
    CStringPtr modelName = nodeMapTLDevice.GetNode("DeviceModelName");

    _cameraInfo->setCameraName(modelName->GetValue().c_str());
    if (_cameraInfo->validateURL(_cameraInfoPath)) {
        _cameraInfo->loadCameraInfo(_cameraInfoPath);
    } else {
        ROS_INFO("flir_adk_ethernet - camera_info_url could not be validated. Publishing with unconfigured camera.");
    }
}

bool EthernetCamera::closeCamera() {
    if(_pCam) {
        unsetCameraEvents();
        stopCapture();
        _pCam->DeInit();
    }

    return true;
}

void EthernetCamera::unsetCameraEvents() {
    try {
        _pCam->UnregisterEvent(*_imageHandler);
    } catch(Spinnaker::Exception e) {
        // if there's an error - the event is already unregistered, 
        // or the camera is invalid (no need to unset)
    }
}

cv::Mat EthernetCamera::getImageMatrix() {
    if(_isStreaming) {
        auto data = _imageHandler->GetImageData();

        // copy the image data to _bufferStart, which backs _thermalImageMat
        memcpy(_bufferStart, data, _imageSize);
    }
    return _thermalImageMat;
}

sensor_msgs::CameraInfo EthernetCamera::getCameraInfo() {
    return _cameraInfo->getCameraInfo();
}

uint64_t EthernetCamera::getActualTimestamp() {
    return _imageHandler->GetCaptureTime();
}

std::string EthernetCamera::setPixelFormat(std::string format) {
    stopCapture();

    // set this pixel format and associated properties
    _selectedFormat = ImageFormat(format);
    initPixelFormat();

    // set image handler pixel format to ensure the conversions are correct
    _imageHandler->setPixelFormat(_selectedFormat.getFormat());

    // replace the buffer due to the change in pixel size
    resetBuffer();

    // reset the openCV matrix
    initOpenCVBuffers();

    startCapture();

    // notify caller which format is active
    return _selectedFormat.toString();
}

std::string EthernetCamera::performFFC() {
    INodeMap &nodeMap = _pCam->GetNodeMap();
    CCommandPtr ffcNode = nodeMap.GetNode("BosonRunFfc");
    if (IsAvailable(ffcNode) && IsWritable(ffcNode)) {
        ffcNode->Execute();
        return "FFC";
    }

    return "";
}

std::string EthernetCamera::setAutoFFC(bool autoFFC) {
    INodeMap &nodeMap = _pCam->GetNodeMap();
    CEnumerationPtr ffcNode = nodeMap.GetNode("BosonFfcMode");
    if (IsAvailable(ffcNode) && IsReadable(ffcNode)) {
        gcstring nodeValName = autoFFC ? "Auto" : "Manual"; 
        int64_t nodeVal = ffcNode->GetEntryByName(nodeValName)->GetValue();
        ffcNode->SetIntValue(nodeVal);
        return nodeValName.c_str();
    }

    return "";
}

std::string EthernetCamera::getNodeValue(std::string nodeName) {
    INodeMap &nodeMap = _pCam->GetNodeMap();
    CNodePtr node = nodeMap.GetNode(nodeName.c_str());

    if(IsAvailable(node) && IsReadable(node)) {
        CValuePtr valueNode = static_cast<CValuePtr>(node);
        return valueNode->ToString().c_str();
    }

    return "";
}

bool EthernetCamera::setNodeValue(std::string nodeName, std::string value) {
    bool result = false;
    stopCapture();

    INodeMap &nodeMap = _pCam->GetNodeMap();
    CNodePtr node = nodeMap.GetNode(nodeName.c_str());

    try {
        if(IsAvailable(node) && IsWritable(node)) {
            switch (node->GetPrincipalInterfaceType())
            {
            case intfIString:
                result = setStringNode(node, value);
                break;
            case intfIInteger:
            {
                int *intVal;
                if(tryConvertStrInt(value, intVal)) {
                    result = setIntNode(node, *intVal);
                }
                break;
            }
            case intfIFloat:
            {
                float *floatValue;
                if(tryConvertStrFloat(value, floatValue)) {
                    result = setFloatNode(node, *floatValue);
                }
                break;
            }
            case intfIBoolean:
            {
                bool boolValue = (toLower(value) == "true");
                result = setBoolNode(node, boolValue);
                break;
            }
            case intfIEnumeration:
                result = setEnumNode(node, value);
                break;
            case intfICommand:
                result = setCommandNode(node);
                break;
            default:
                break;
            }
        }
    } catch(Spinnaker::Exception e) {
        ROS_ERROR("%s", e.what());
    }
    
    startCapture();
    return result;
}

bool EthernetCamera::setStringNode(CNodePtr node, std::string value) {
    CStringPtr stringNode = static_cast<CStringPtr>(node);
    stringNode->SetValue(value.c_str());
    return true;
}

bool EthernetCamera::setIntNode(CNodePtr node, int value) {
    CIntegerPtr intNode = static_cast<CIntegerPtr>(node);
    intNode->SetValue(value);
    return true;
}

bool EthernetCamera::setFloatNode(CNodePtr node, float value) {
    CFloatPtr floatNode = static_cast<CFloatPtr>(node);
    floatNode->SetValue(value);
    return true;
}

bool EthernetCamera::setBoolNode(CNodePtr node, bool value) {
    CBooleanPtr boolNode = static_cast<CBooleanPtr>(node);
    boolNode->SetValue(value);
    return true;
}

bool EthernetCamera::setEnumNode(CNodePtr node, std::string value) {
    CEnumerationPtr enumNode = static_cast<CEnumerationPtr>(node);
    int64_t nodeValue = enumNode->GetEntryByName(value.c_str())->GetValue();
    enumNode->SetIntValue(nodeValue);
    return true;
}

bool EthernetCamera::setCommandNode(CNodePtr node) {
    CCommandPtr commNode = static_cast<CCommandPtr>(node);
    commNode->Execute();
    return true;
}

void EthernetCamera::setPolarity(Polarity pol) {

}

void EthernetCamera::startCapture() {
    _pCam->BeginAcquisition();
    _isStreaming = true;
}

void EthernetCamera::stopCapture() {
    try {
        _isStreaming = false;
        _pCam->EndAcquisition();
    } catch(Spinnaker::Exception e) {
        ROS_ERROR("%s", e.what());
    }
}

std::string EthernetCamera::getEncoding() {
    return _selectedFormat.getImageEncoding();
}
