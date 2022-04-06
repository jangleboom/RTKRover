#ifndef RWAHT_RTK_CONFIG_H
#define RWAHT_RTK_CONFIG_H

#define DEVICE_NAME "rwaht_rtk_01"

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

/******************************************************************************/
//                        Default WIFI settings
/******************************************************************************/
#define DEFAULT_SSID "RWAHTRTK_Wifi_Manager"
#define DEFAULT_KEY "12345678"

/******************************************************************************/
//                        Default BLE settings
/******************************************************************************/
#define PAYLOAD_BUF_LEN 20
#define SERVICE_UUID          "713D0000-503E-4C75-BA94-3148F18D941E"
#define CHARACTERISTIC_UUID   "713D0002-503E-4C75-BA94-3148F18D941E"

#endif /*** RWAHT_RTK_CONFIG_H ***/






