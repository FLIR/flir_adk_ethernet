#ifndef FLIR_BOSON_ETHERNET_UTIL_H
#define FLIR_BOSON_ETHERNET_UTIL_H

#include <iostream>
#include <string>
#include <sstream>
#include <algorithm>

using namespace std;

namespace flir_boson_ethernet {

const char *GetDottedAddress(int64_t value);
std::string toLower(std::string s);

}

#endif