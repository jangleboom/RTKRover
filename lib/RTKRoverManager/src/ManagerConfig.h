#ifndef RTK_ROVER_STATION_CONFIG_H
#ifndef MANAGER_CONFIG_H
#define MANAGER_CONFIG_H
#include <Arduino.h>

// THIS WILL BE REPLACED BY THE RTKRoverConfig.h after merge

/******************************************************************************/
//                       Default Serial settings
/******************************************************************************/
//set to true for debug output, false for no debug output
#define DEBUGGING true 
#define DEBUG_SERIAL \
  if (DEBUGGING) Serial

#define BAUD                          115200




#endif  /*** MANAGER_CONFIG_H ***/
#endif