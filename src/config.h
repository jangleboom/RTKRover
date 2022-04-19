#ifndef CONFIG_H
#define CONFIG_H
#include <Arduino.h>

// Deactivate brown out detection
#undef ESP_ERROR_CHECK
#define ESP_ERROR_CHECK(x)   do { esp_err_t rc = (x); if (rc != ESP_OK) { ESP_LOGE("err", "esp_err_t = %d", rc); assert(0 && #x);} } while(0);

/******************************************************************************/
//                       Default Serial settings
/******************************************************************************/
//set to true for debug output, false for no debug output
#define DEBUGGING true 
#define DEBUG_SERIAL \
  if (DEBUGGING) Serial

const int BAUD = 115200;

#ifdef DEBUGGING
#ifdef TESTING
#include "tests.h"
#endif
#endif

/******************************************************************************/
//                        Default WIFI settings
/******************************************************************************/
#define DEVICE_TYPE                  "HTRTK_"
#define DEFAULT_KEY                  "12345678"
String getDeviceName(const String &);
uint32_t getChipId(void);



/******************************************************************************/
//                        Default BLE settings
/******************************************************************************/
#define PAYLOAD_BUF_LEN              20
#define SERVICE_UUID                 "713D0000-503E-4C75-BA94-3148F18D941E"
#define CHARACTERISTIC_UUID_NOTIFY   "713D0002-503E-4C75-BA94-3148F18D941E"
#define LIN_ACCEL_Z_DECIMAL_DIGITS   2
#define DATA_STR_DELIMITER           " "

/******************************************************************************/
//                        Default BNO080 settings
/******************************************************************************/
#define BNO080_UPDATE_RATE_MS        20 // Time between sensor readings
/******************************************************************************/
//                        Help Functions
/******************************************************************************/


#endif /*** HTRTK_CONFIG_H ***/






