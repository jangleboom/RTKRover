#include "config.h"

uint32_t getChipId() {
  uint32_t chipId = 0;

  for(int i=0; i<17; i=i+8) {
	  chipId |= ((ESP.getEfuseMac() >> (40 - i)) & 0xff) << i;
	}
    DEBUG_SERIAL.print("chipId: ");
    DEBUG_SERIAL.println(chipId, HEX);
    return chipId;
}

String getDeviceName(const String &prefix) {
  String deviceName((char *)0);
  unsigned int prefixSize = sizeof(prefix);
  deviceName.reserve(prefixSize + 7);
  deviceName = prefix + String(getChipId(), HEX);

  return deviceName;
}