#include "gmock/gmock.h"
#include <spinnaker/Spinnaker.h>
#include <spinnaker/SpinGenApi/SpinnakerGenApi.h>
#include "spinnaker/SpinGenApi/INodeMap.h"
#include "spinnaker/SpinGenApi/GCString.h"

using namespace Spinnaker;
using namespace Spinnaker::GenICam;
using namespace Spinnaker::GenApi;

class MockNodeMap : public INodeMap {
  public:
    MOCK_CONST_METHOD1(GetNodes, void(NodeList_t &Nodes));
    MOCK_CONST_METHOD1(GetNode, INode*(const gcstring& Name));
    MOCK_CONST_METHOD0(InvalidateNodes, void());
    MOCK_CONST_METHOD2(Connect, bool(IPort* pPort, const gcstring& PortName));
    MOCK_CONST_METHOD1(Connect, bool(IPort* pPort));
    MOCK_METHOD0(GetDeviceName, gcstring());
    MOCK_METHOD1(Poll, void(int64_t ElapsedTime));
    MOCK_CONST_METHOD0(GetLock, CLock&());
    MOCK_CONST_METHOD0(GetNumNodes, uint64_t());
};