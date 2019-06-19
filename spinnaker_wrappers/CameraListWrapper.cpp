#include "CameraListWrapper.h"

using namespace flir_boson_ethernet;

CameraListWrapper::CameraListWrapper(CameraList camList) : _camList(camList) {}

CameraListWrapper::CameraListWrapper(const CameraListWrapper& wrapper) {
    _camList = wrapper._camList;
}

CameraListWrapper::~CameraListWrapper() {
    
}

const unsigned int CameraListWrapper::GetSize() {
    return _camList.GetSize();
}

void CameraListWrapper::Clear() {
    std::cout << "CLEARING" << std::endl;
    _camList.Clear();
    std::cout << "CLEARED" << std::endl;
}

CameraWrapper CameraListWrapper::GetByIndex(unsigned int i) {
    CameraPtr ptr = _camList.GetByIndex(i);
    CameraWrapper wrapper(ptr);
    return wrapper;
}