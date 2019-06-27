#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include "mocks/MockCamera.h"
#include "flir_boson_ethernet/ImageEventHandler.h"
#include <string>

using std::string;
using ::testing::Return;
using ::testing::ReturnRef;
using ::testing::_;
using namespace flir_boson_ethernet;


// ImageEvent tests
TEST(ImageEventTests, ConstructImageHandler) {
    MockCamera camera;
    MockNodeMap nodeMap;

    EXPECT_CALL(camera, GetTLDeviceNodeMap())
        .Times(1)
        .WillRepeatedly(ReturnRef(nodeMap));
    // EXPECT_CALL(camera, )

    auto camPtr = std::make_shared<CameraWrapper>(camera);
    ImageEventHandler handler = ImageEventHandler(camPtr, PixelFormat_RGB8);
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv); 
    return RUN_ALL_TESTS();
}