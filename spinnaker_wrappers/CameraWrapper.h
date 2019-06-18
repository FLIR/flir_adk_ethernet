#ifndef CAMERAWRAPPER_H
#define CAMERAWRAPPER_H

#include <spinnaker/Spinnaker.h>
#include <spinnaker/SpinGenApi/SpinnakerGenApi.h>

using namespace std;
using namespace Spinnaker;
using namespace Spinnaker::GenApi;
using namespace Spinnaker::GenICam;

namespace flir_boson_ethernet {

class CameraWrapper {
  public:
    CameraWrapper(CameraPtr cam);
    virtual ~CameraWrapper();
    virtual void Init() = 0;
    virtual bool IsValid() = 0;
    virtual INodeMap& GetNodeMap() = 0;
    virtual INodeMap& GetTLDeviceNodeMap() = 0;
    virtual void RegisterEvent(Spinnaker::Event &e) = 0;
    virtual void BeginAcquisition() = 0;
    virtual void EndAcquisition() = 0;
    virtual void DeInit() = 0;
  private:
    CameraPtr _cam;
};

}

#endif