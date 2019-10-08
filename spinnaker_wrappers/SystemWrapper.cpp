#include "SystemWrapper.h"

using namespace flir_adk_ethernet;

SystemWrapper::SystemWrapper(SystemPtr sys) : _sys(sys) {}

SystemWrapper::~SystemWrapper() {}

CameraListWrapper SystemWrapper::GetCameras() {
    CameraList lst = _sys->GetCameras();
    CameraListWrapper camList(lst);
    return camList;
}

void SystemWrapper::ReleaseInstance() {
    _sys->ReleaseInstance();
}