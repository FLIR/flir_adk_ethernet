#include "flir_boson_ethernet/Util.h"

using namespace flir_boson_ethernet;

const char *flir_boson_ethernet::GetDottedAddress( int64_t value )
{
    // Helper function for formatting IP Address into the following format
    // x.x.x.x
    unsigned int inputValue = static_cast<unsigned int>( value );
    ostringstream convertValue;
    convertValue << ((inputValue & 0xFF000000) >> 24);
    convertValue << ".";
    convertValue << ((inputValue & 0x00FF0000) >> 16);
    convertValue << ".";
    convertValue << ((inputValue & 0x0000FF00) >> 8);
    convertValue << ".";
    convertValue << (inputValue & 0x000000FF);
    return convertValue.str().c_str();
}

std::string flir_boson_ethernet::toLower(std::string s) {
    auto newStr = s;
    std::transform(newStr.begin(), newStr.end(), newStr.begin(), ::tolower);
    return newStr;
}

bool flir_boson_ethernet::tryConvertStrInt(std::string s, int *i) {
    std::stringstream converter;

    converter << s;
    converter >> *i;

    return !converter.fail();
}

bool flir_boson_ethernet::tryConvertStrFloat(std::string s, float *f) {
    std::stringstream converter;
    converter << s;
    converter >> *f;
    return !converter.fail();
}