/******************************************************************************/
/*                                                                            */
/*  Copyright (C) 2018, FLIR Systems                                          */
/*  All rights reserved.                                                      */
/*                                                                            */
/******************************************************************************/
#ifndef SYSTEMWRAPPER_H
#define SYSTEMWRAPPER_H

#include "CameraListWrapper.h"

#include <spinnaker/Spinnaker.h>
#include <spinnaker/SpinGenApi/SpinnakerGenApi.h>

using namespace std;
using namespace Spinnaker;
using namespace Spinnaker::GenApi;
using namespace Spinnaker::GenICam;

namespace flir_adk_ethernet 
{

class SystemWrapper {
  public:
    SystemWrapper(SystemPtr sys);
    virtual ~SystemWrapper();
    virtual CameraListWrapper GetCameras();
    virtual void ReleaseInstance();
  private:
    SystemPtr _sys;
};

}

#endif