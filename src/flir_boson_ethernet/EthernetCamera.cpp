#include "flir_boson_ethernet/EthernetCamera.h"

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

std::string toLower(std::string s) {
    auto newStr = s;
    std::transform(newStr.begin(), newStr.end(), newStr.begin(), ::tolower);
    return newStr;
}

ImageFormat::ImageFormat(std::string format) {
    _format = PixelFormat_RGB8;
    if(toLower(format) == "mono_8")
        _format = PixelFormat_Mono8;
    if(toLower(format) == "mono_16")
        _format = PixelFormat_Mono16;
    if(toLower(format) == "color_8")
        _format = PixelFormat_RGB8;
    if(toLower(format) == "color_16")
        _format = PixelFormat_RGB16;
}

ImageFormat::ImageFormat(const ImageFormat& obj) : 
    _format(obj._format) {}

ImageFormat::~ImageFormat() {}

int ImageFormat::getValue(CEnumerationPtr nodePtr) {
    gcstring strValue = getNodeName();

    CEnumEntryPtr formatEntry = nodePtr->GetEntryByName(strValue);
    if (IsAvailable(formatEntry) && IsReadable(formatEntry)) {
        return formatEntry->GetValue();
    }

    return -1;
}

gcstring ImageFormat::getNodeName() {
    switch(_format) {
    case PixelFormat_Mono8:
        return "Mono8";
    case PixelFormat_Mono16:
        return "Mono16";
    case PixelFormat_RGB8:
        return "BayerRG8";
    case PixelFormat_RGB16:
        return "BayerRG16";
    }
}

int ImageFormat::getBytesPerPixel() {
    switch(_format) {
    case PixelFormat_Mono8:
        return 1;
    case PixelFormat_Mono16:
        return 2;
    case PixelFormat_RGB8:
        return 3;
    case PixelFormat_RGB16:
        return 6;
    }

    return 3;
}

int ImageFormat::getMatType() {
    switch(_format) {
    case PixelFormat_Mono8:
        return CV_8UC1;
    case PixelFormat_Mono16:
        return CV_16UC1;
    case PixelFormat_RGB8:
        return CV_8UC3;
    case PixelFormat_RGB16:
        return CV_16UC3;
    }

    return CV_8UC3;
}

std::string ImageFormat::toString() {
    switch(_format) {
    case PixelFormat_Mono8:
        return "MONO_8";
    case PixelFormat_Mono16:
        return "MONO_16";
    case PixelFormat_RGB8:
        return "COLOR_8";
    case PixelFormat_RGB16:
        return "COLOR_16";
    default:
        return "COLOR_8";
    }
}

std::string ImageFormat::getImageEncoding() {
    switch(_format) {
    case PixelFormat_Mono8:
        return "mono8";
    case PixelFormat_Mono16:
        return "mono16";
    case PixelFormat_RGB8:
        return "rgb8";
    case PixelFormat_RGB16:
        return "rgb16";
    default:
        return "rgb8";
    }
}

PixelFormatEnums ImageFormat::getFormat() {
    return _format;
}

EthernetCamera::EthernetCamera(EthernetCameraInfo info, 
        std::shared_ptr<SystemWrapper> sys,
        ros::NodeHandle nh) :
    _ipAddr(info.ip), 
    _cameraInfoPath(info.camInfoPath),
    _width(info.width),
    _height(info.height),
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
}

// AGC Sample ONE: Linear from min to max.
// Input is a MATRIX (height x width) of 16bits. (OpenCV mat)
// Output is a MATRIX (height x width) of 8 bits (OpenCV mat)
void EthernetCamera::agcBasicLinear(const Mat &input_16,
                                    Mat *output_8,
                                    const int &height,
                                    const int &width)
{
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
    std::cout << "INIT IMAGE HANDLER" << std::endl;

    initOpenCVBuffers();
    std::cout << "OPENCV BUFFERS" << std::endl;
    setCameraInfo();
    std::cout << "SET CAMERA INFO" << std::endl;

    return true;
} 

bool EthernetCamera::findMatchingCamera(CameraListWrapper camList, const unsigned int numCams) {
    gcstring deviceIPAddress = "0.0.0.0";

    for (unsigned int i = 0; i < numCams; i++)
    {
        // Select camera
        CameraWrapper cam = camList.GetByIndex(i);
        INodeMap &nodeMapTLDevice = cam.GetTLDeviceNodeMap();

        CIntegerPtr ptrIPAddress = nodeMapTLDevice.GetNode("GevDeviceIPAddress");
        if (IsAvailable(ptrIPAddress) && IsReadable(ptrIPAddress)) {
            deviceIPAddress = GetDottedAddress(ptrIPAddress->GetValue());
        }
        if(deviceIPAddress == _ipAddr) {
            CStringPtr modelName = nodeMapTLDevice.GetNode("DeviceModelName");
            ROS_INFO("Found matching camera %s", modelName->ToString().c_str());
            _pCam = std::make_shared<CameraWrapper>(cam);
            return true;
        }
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
        ROS_ERROR("ERROR: %s", e.what());
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
    // Declarations for Zoom representation
    // Will be used or not depending on program arguments
    // OpenCV output buffer , BGR -> Three color spaces :
    // (640 - 640 - 640 : p11 p21 p31 .... / p12 p22 p32 ..../ p13 p23 p33 ...)
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

bool EthernetCamera::closeCamera()
{
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
        // pass
    }
}

cv::Mat EthernetCamera::getImageMatrix() {
    auto data = _imageHandler->GetImageData();
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

    _selectedFormat = ImageFormat(format);
    initPixelFormat();

    _imageHandler->setPixelFormat(_selectedFormat.getFormat());

    auto oldAddress = _bufferStart;
    createBuffer();
    memcpy(oldAddress, _bufferStart, _imageSize);
    delete _bufferStart;
    _bufferStart = oldAddress;

    initOpenCVBuffers();

    startCapture();

    return _selectedFormat.toString();
}

void EthernetCamera::performFFC() {

}

void EthernetCamera::setAutoFFC(bool autoFFC) {

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
