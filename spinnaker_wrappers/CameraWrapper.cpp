#include "CameraWrapper.h"

using namespace flir_boson_ethernet;

CameraWrapper::CameraWrapper(CameraPtr cam) : _cam(cam) {}

CameraWrapper::~CameraWrapper() {
    delete _cam;
}

void CameraWrapper::Init() {
    _cam->Init();
}

bool CameraWrapper::IsValid() {
    return _cam->IsValid();
}

INodeMap& CameraWrapper::GetNodeMap() {
    return _cam->GetNodeMap();
}

INodeMap& CameraWrapper::GetTLDeviceNodeMap() {
    return _cam->GetTLDeviceNodeMap();
}

void CameraWrapper::RegisterEvent(Spinnaker::Event &e) {
    _cam->RegisterEvent(e);
}

void CameraWrapper::BeginAcquisition() {
    _cam->BeginAcquisition();
}

void CameraWrapper::EndAcquisition() {
    _cam->EndAcquisition();
}

void CameraWrapper::DeInit() {
    _cam->DeInit();
}
