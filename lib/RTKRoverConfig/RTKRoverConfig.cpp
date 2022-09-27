#include "RTKRoverConfig.h"

uint32_t getChipId() {
  uint32_t chipId = 0;

  for(int i=0; i<17; i=i+8) {
	  chipId |= ((ESP.getEfuseMac() >> (40 - i)) & 0xff) << i;
	}
    DEBUG_SERIAL.print("chipId: ");
    DEBUG_SERIAL.println(chipId, HEX);
    return chipId;
}

String getDeviceName(const String &prefix) 
{ 
  String deviceName((char *)0);
  String suffix = String(getChipId(), HEX);

  unsigned int prefixLen = prefix.length();
  unsigned int suffixLen = suffix.length();
  
  deviceName.reserve(prefixLen + suffixLen);
  deviceName += prefix;
  deviceName += suffix;

  return deviceName;
}