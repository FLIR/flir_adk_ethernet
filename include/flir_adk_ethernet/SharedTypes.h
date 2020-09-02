/******************************************************************************/
/*                                                                            */
/*  Copyright (C) 2018, FLIR Systems                                          */
/*  All rights reserved.                                                      */
/*                                                                            */
/******************************************************************************/
#ifndef FLIR_ADK_ETHERNET_SHAREDTYPES_H
#define FLIR_ADK_ETHERNET_SHAREDTYPES_H

namespace flir_adk_ethernet
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