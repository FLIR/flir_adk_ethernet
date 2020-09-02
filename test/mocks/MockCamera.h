/******************************************************************************/
/*                                                                            */
/*  Copyright (C) 2018, FLIR Systems                                          */
/*  All rights reserved.                                                      */
/*                                                                            */
/******************************************************************************/
#include "gmock/gmock.h"
#include "../spinnaker_wrappers/CameraWrapper.h"
#include "MockNodeMap.h"

using namespace flir_adk_ethernet;

class MockCamera : public CameraWrapper {
  public:
    MOCK_METHOD0(Init, void());
    MOCK_METHOD0(IsValid, bool());
    MOCK_METHOD0(GetNodeMap, INodeMap&());
    MOCK_METHOD0(GetTLDeviceNodeMap, INodeMap&());
    MOCK_METHOD1(RegisterEvent, void(Spinnaker::Event& e));
    MOCK_METHOD1(UnregisterEvent, void(Spinnaker::Event& e));
    MOCK_METHOD0(BeginAcquisition, void());
    MOCK_METHOD0(EndAcquisition, void());
    MOCK_METHOD0(DeInit, void());
};