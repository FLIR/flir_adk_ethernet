#include "flir_boson_ethernet/Util.h"
#include "flir_boson_ethernet/EthernetCamera.h"

using namespace cv;
using namespace flir_boson_ethernet;

EthernetCamera::EthernetCamera(EthernetCameraInfo info, 
        std::shared_ptr<SystemWrapper> sys,
        ros::NodeHandle nh) :
    _ipAddr(info.ip), 
    _cameraInfoPath(info.camInfoPath),
    _width(info.width),
    _height(info.height),
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
        ROS_ERROR("flir_boson_ethernet - ERROR : NO_CAMERAS. No cameras found");
        camList.Clear();
        _system->ReleaseInstance();
        return false;
    }

    if(!findMatchingCamera(camList, numCameras) || !_pCam->IsValid()) {
        ROS_ERROR("flir_boson_ethernet - ERROR : OPEN. No device matches ip_addr: %s", _ipAddr.c_str());
        return false;
    }
    _pCam->Init();
    initPixelFormat();

    if(!setImageInfo()) {
        ROS_ERROR("flir_boson_ethernet - ERROR : GET_CONFIGURATION. Cannot get image for setting dimensions");
        return false;
    }
    setCameraEvents();

    if (!setImageAcquisition())
    {
        ROS_ERROR("flir_boson_ethernet - ERROR : CAMERA_ACQUISITION. Cannot set image acquisition.");
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
        INodeMap &nodeMap = _pCam->GetNodeMap();
        setWidthHeight(nodeMap);

        CIntegerPtr widthNode = nodeMap.GetNode("Width");
        widthNode->SetValue(_width);
        CIntegerPtr heightNode = nodeMap.GetNode("Height");
        heightNode->SetValue(_height);

        ROS_INFO("Camera info - Width: %d, Height: %d", _width, _height);

        createBuffer();

        return true;
    } catch(Spinnaker::Exception e) {
        ROS_ERROR("flir_boson_ethernet - ERROR : %s", e.what());
        return false;
    }
}

void EthernetCamera::setWidthHeight(INodeMap& nodeMap) {
    CIntegerPtr maxWidthNode = nodeMap.GetNode("WidthMax");
    CIntegerPtr maxHeightNode = nodeMap.GetNode("HeightMax");
    int maxWidth = maxWidthNode->GetValue();
    int maxHeight = maxHeightNode->GetValue();

    _width = min(_width, maxWidth);
    _height = min(_height, maxHeight);

    if(_width == 0) {
        _width = maxWidth;
    }
    if(_height == 0) {
        _height = maxHeight;
    }
}

void EthernetCamera::createBuffer() {
    _imageSize = _height * _width * getPixelSize();
    _bufferStart = new uint8_t[_imageSize];
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
        ROS_INFO("flir_boson_ethernet - camera_info_url could not be validated. Publishing with unconfigured camera.");
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
    auto data = _imageHandler->GetImageData();

    // copy the image data to _bufferStart, which backs _thermalImageMat
    memcpy(_bufferStart, data, _imageSize);
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
    auto oldAddress = _bufferStart;
    createBuffer();
    memcpy(oldAddress, _bufferStart, _imageSize);
    delete _bufferStart;
    _bufferStart = oldAddress;

    // reset the openCV matrix
    initOpenCVBuffers();

    startCapture();

    // notify caller which format is active
    return _selectedFormat.toString();
}

std::string EthernetCamera::performFFC() {
    INodeMap &nodeMap = _pCam->GetNodeMap();
    auto ffcNode = nodeMap.GetNode("BosonRunFfc");
    if (IsAvailable(ffcNode) && IsReadable(ffcNode)) {
        // ffcNode->Execute();
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

void EthernetCamera::setPolarity(Polarity pol) {

}

void EthernetCamera::startCapture() {
    _pCam->BeginAcquisition();
}

void EthernetCamera::stopCapture() {
    try {
        _pCam->EndAcquisition();
    } catch(Spinnaker::Exception e) {
        // pass
    }
}

std::string EthernetCamera::getEncoding() {
    return _selectedFormat.getImageEncoding();
}
