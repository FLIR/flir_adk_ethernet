#ifndef FLIR_BOSON_ETHERNET_SHAREDTYPES_H
#define FLIR_BOSON_ETHERNET_SHAREDTYPES_H

namespace flir_boson_ethernet
{

enum Encoding
{
  YUV = 0,
  RAW16 = 1
};

enum SensorTypes
{
  Boson320,
  Boson640
};

}

#endif