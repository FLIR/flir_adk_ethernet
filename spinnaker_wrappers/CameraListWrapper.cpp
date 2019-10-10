/******************************************************************************/
/*                                                                            */
/*  Copyright (C) 2018, FLIR Systems                                          */
/*  All rights reserved.                                                      */
/*                                                                            */
/******************************************************************************/
#include "CameraListWrapper.h"

using namespace flir_adk_ethernet;

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
    _camList.Clear();
}

CameraWrapper CameraListWrapper::GetByIndex(unsigned int i) {
    CameraPtr ptr = _camList.GetByIndex(i);
    CameraWrapper wrapper(ptr);
    return wrapper;
}