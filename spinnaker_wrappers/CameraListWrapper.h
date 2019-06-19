#ifndef CAMERALISTWRAPPER_H
#define CAMERALISTWRAPPER_H

#include "CameraWrapper.h"

#include <spinnaker/Spinnaker.h>
#include <spinnaker/SpinGenApi/SpinnakerGenApi.h>

using namespace std;
using namespace Spinnaker;
using namespace Spinnaker::GenApi;
using namespace Spinnaker::GenICam;

namespace flir_boson_ethernet {

class CameraListWrapper {
  public:
    CameraListWrapper(CameraList camList);
    CameraListWrapper(const CameraListWrapper& wrapper);
    virtual ~CameraListWrapper();
    virtual const unsigned int GetSize();
    virtual void Clear();
    virtual CameraWrapper GetByIndex(unsigned int i);
  private:
    CameraList _camList;
};

}

#endif