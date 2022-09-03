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

#define BAUD                          115200

#ifdef DEBUGGING
#ifdef TESTING
#include "tests.h"
#endif
#endif

/*******************************************************************************
 *                         Default WiFi settings
 * ****************************************************************************/
#define DEVICE_TYPE                  "RTKRover_"
#define DEFAULT_KEY                  "12345678"
String getDeviceName(const String &);
uint32_t getChipId(void);

/*******************************************************************************
 *                         Default RTK settings
 * ****************************************************************************/
#define RTK_I2C_ADDR                0x42
#define RTK_SDA_PIN                 33
#define RTK_SCL_PIN                 32

/*******************************************************************************
 *                         Default BLE settings
 * ****************************************************************************/
#define PAYLOAD_BUF_LEN              20
#define SERVICE_UUID                            "713D0000-503E-4C75-BA94-3148F18D941E"
#define HEADTRACKER_CHARACTERISTIC_UUID         "713D0002-503E-4C75-BA94-3148F18D941E"
#define REALTIME_KINEMATICS_CHARACTERISTIC_UUID "713D0004-503E-4C75-BA94-3148F18D941E"
#define RTK_ACCURACY_CHARACTERISTIC_UUID        "713D0006-503E-4C75-BA94-3148F18D941E"
#define LIN_ACCEL_Z_DECIMAL_DIGITS   2
#define DATA_STR_DELIMITER           " "

/*******************************************************************************
 *                         Default BNO080 settings
 * ****************************************************************************/
/*  
INFO: The Qwiic VR IMU has onboard I2C pull up resistors; if multiple sensors are 
connected to the bus with the pull-up resistors enabled, the parallel 
equivalent resistance will create too strong of a pull-up for the bus to 
operate correctly. As a general rule of thumb, disable all but one pair of 
pull-up resistors if multiple devices are connected to the bus. If you need to 
disconnect the pull up resistors they can be removed by removing the solder on 
the corresponding jumpers labeled with "I2C" on the board.

BUT: we use here two I2C connections for real parallel computing on two cores.
*/
#define BNO080_I2C_ADDR                 0x4B
#define BNO080_SDA_PIN                  23
#define BNO080_SCL_PIN                  22
#define I2C_FREQUENCY_100K              100000  // 100 kHz
#define I2C_FREQUENCY_400K              400000  // 400 kHz
#define BNO080_ROT_VECT_UPDATE_RATE_MS  10      // Time between sensor readings
#define BNO080_LIN_ACCEL_UPDATE_RATE_MS 10      // 100 Hz
#define BNO080_STEP_CNT_UPDATE_RATE_MS  32      // 31.25 Hz lt. datasheet

/*******************************************************************************
 *                         FreeRTOS
 * ****************************************************************************/
#define RUNNING_CORE_0                0  // Low level WiFi code runs on core 0
#define RUNNING_CORE_1                1  // Use core 1 for all other tasks
// Each task is assigned a priority from 0 to ( configMAX_PRIORITIES - 1 ), 
// where configMAX_PRIORITIES is defined within FreeRTOSConfig.h.
#define GNSS_OVER_WIFI_PRIORITY       2  // GNSS should have a lower priority
#define BNO080_OVER_BLE_PRIORITY      1  // Headtracking: highest priority
#define BLE_TASK_INTERVAL_MS          10
#define RTK_REFRESH_INTERVAL_MS       20 
#define WIFI_TASK_INTERVAL_MS         100
#define MIN_ACCEPTABLE_ACCURACY_MM    1000 // Device will only send if accuray is better than this
/*******************************************************************************
 *                         Help Functions
 * ****************************************************************************/


#endif /*** HTRTK_CONFIG_H ***/






