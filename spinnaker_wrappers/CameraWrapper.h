#ifndef CAMERAWRAPPER_H
#define CAMERAWRAPPER_H

#include <spinnaker/Spinnaker.h>
#include <spinnaker/SpinGenApi/SpinnakerGenApi.h>

using namespace std;
using namespace Spinnaker;
using namespace Spinnaker::GenApi;
using namespace Spinnaker::GenICam;

namespace flir_adk_ethernet {

class CameraWrapper {
  public:
    CameraWrapper();
    CameraWrapper(CameraPtr cam);
    CameraWrapper(const CameraWrapper& wrapper);
    virtual ~CameraWrapper();
    virtual void Init();
    virtual bool IsValid();
    virtual INodeMap& GetNodeMap();
    virtual INodeMap& GetTLDeviceNodeMap();
    virtual void RegisterEvent(Spinnaker::Event &e);
    virtual void UnregisterEvent(Spinnaker::Event &e);
    virtual void BeginAcquisition();
    virtual void EndAcquisition();
    virtual void DeInit();
  private:
    CameraPtr _cam;
};

}

#endif