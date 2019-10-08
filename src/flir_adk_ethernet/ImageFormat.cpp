#include "flir_adk_ethernet/Util.h"
#include "flir_adk_ethernet/ImageFormat.h"

using namespace cv;
using namespace flir_adk_ethernet;


ImageFormat::ImageFormat(std::string format) {
    _format = PixelFormat_RGB8;
    if(toLower(format) == "mono_8")
        _format = PixelFormat_Mono8;
    if(toLower(format) == "mono_16")
        _format = PixelFormat_Mono16;
    if(toLower(format) == "color_8")
        _format = PixelFormat_RGB8;
    if(toLower(format) == "color_16")
        _format = PixelFormat_RGB16;
}

ImageFormat::ImageFormat(const ImageFormat& obj) : 
    _format(obj._format) {}

ImageFormat::~ImageFormat() {}

int ImageFormat::getValue(CEnumerationPtr nodePtr) {
    gcstring strValue = getNodeName();

    CEnumEntryPtr formatEntry = nodePtr->GetEntryByName(strValue);
    if (IsAvailable(formatEntry) && IsReadable(formatEntry)) {
        return formatEntry->GetValue();
    }

    return -1;
}

gcstring ImageFormat::getNodeName() {
    switch(_format) {
    case PixelFormat_Mono8:
        return "Mono8";
    case PixelFormat_Mono16:
        return "Mono16";
    case PixelFormat_RGB8:
        return "BayerRG8";
    case PixelFormat_RGB16:
        return "BayerRG16";
    }
}

int ImageFormat::getBytesPerPixel() {
    switch(_format) {
    case PixelFormat_Mono8:
        return 1;
    case PixelFormat_Mono16:
        return 2;
    case PixelFormat_RGB8:
        return 3;
    case PixelFormat_RGB16:
        return 6;
    }

    return 3;
}

int ImageFormat::getMatType() {
    switch(_format) {
    case PixelFormat_Mono8:
        return CV_8UC1;
    case PixelFormat_Mono16:
        return CV_16UC1;
    case PixelFormat_RGB8:
        return CV_8UC3;
    case PixelFormat_RGB16:
        return CV_16UC3;
    }

    return CV_8UC3;
}

std::string ImageFormat::toString() {
    switch(_format) {
    case PixelFormat_Mono8:
        return "MONO_8";
    case PixelFormat_Mono16:
        return "MONO_16";
    case PixelFormat_RGB8:
        return "COLOR_8";
    case PixelFormat_RGB16:
        return "COLOR_16";
    default:
        return "COLOR_8";
    }
}

std::string ImageFormat::getImageEncoding() {
    switch(_format) {
    case PixelFormat_Mono8:
        return "mono8";
    case PixelFormat_Mono16:
        return "mono16";
    case PixelFormat_RGB8:
        return "rgb8";
    case PixelFormat_RGB16:
        return "rgb16";
    default:
        return "rgb8";
    }
}

PixelFormatEnums ImageFormat::getFormat() {
    return _format;
}
