// file for utility functions used across application

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
bool tryConvertStrInt(std::string s, int *i);
bool tryConvertStrFloat(std::string s, float *f);
int roundToEven(int n);

}

#endif