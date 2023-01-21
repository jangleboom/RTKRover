#ifndef RTK_ROVER_CONFIG_H
#define RTK_ROVER_CONFIG_H
#include <Arduino.h>

// Deactivate brown out detection
#undef ESP_ERROR_CHECK
#define ESP_ERROR_CHECK(x)   do { esp_err_t rc = (x); if (rc != ESP_OK) { ESP_LOGE("err", "esp_err_t = %d", rc); assert(0 && #x);} } while(0);

/*
=================================================================================
                                Serial settings
=================================================================================
*/

// Set to true for debug mode, false for production mode
#define DEBUGGING true 
#define DBG \
  if (DEBUGGING) Serial

#ifdef DEBUGGING
#define TESTING
#endif

#define BAUD                          115200

/*
=================================================================================
                                WiFi settings
=================================================================================
*/
#define DEVICE_TYPE                  "rtkrover"
#define DEFAULT_KEY                  "12345678"

/*
=================================================================================
                                RTK settings
=================================================================================
*/
#define RTK_I2C_ADDR                0x42
#define RTK_SDA_PIN                 33
#define RTK_SCL_PIN                 32

/*
=================================================================================
                                BLE settings
=================================================================================
*/
#define PAYLOAD_BUF_LEN              20
#define SERVICE_UUID                            "713D0000-503E-4C75-BA94-3148F18D941E"
#define HEADTRACKER_CHARACTERISTIC_UUID         "713D0002-503E-4C75-BA94-3148F18D941E"
#define REALTIME_KINEMATICS_CHARACTERISTIC_UUID "713D0004-503E-4C75-BA94-3148F18D941E"
#define RTK_ACCURACY_CHARACTERISTIC_UUID        "713D0006-503E-4C75-BA94-3148F18D941E"
#define LIN_ACCEL_Z_DECIMAL_DIGITS   2
#define DATA_STR_DELIMITER           " "

/*
=================================================================================
                                BNO080 settings
=================================================================================
*/
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

/*
=================================================================================
                                FreeRTOS settings
=================================================================================
*/
#define RUNNING_CORE_0                                  0     // Low level WiFi code runs on core 0
#define RUNNING_CORE_1                                  1     // Use core 1 for all other tasks
// Each task is assigned a priority from 0 to ( configMAX_PRIORITIES - 1 ), 
// where configMAX_PRIORITIES is defined within FreeRTOSConfig.h.
#define TASK_RTK_GET_CORR_DATA_PRIORITY                 2     // GNSS should have a lower priority than BNO080 data transmission
#define TASK_RTK_GET_POSITION_PRIORITY                  2
#define TASK_BNO080_VIA_BLE_PRIORITY                    2     // Headtracking: highest priority for immersive audio
#define TASK_RTK_POSITION_VIA_BLE_PRIORITY              2     // Real Time Kinematics data to iOS app, (should not break head tracking)
#define TASK_RTK_BLE_INTERVAL_MS                      100    // Send position to iPhone
#define TASK_RTK_GET_POSITION_INTERVAL_MS             100
#define TASK_BNO_ORIENTATION_VIA_BLE_INTERVAL_MS       12
#define TASK_WIFI_RTK_DATA_INTERVAL_MS               1000  //200 Get fresh correction data from caster
#define MIN_ACCEPTABLE_ACCURACY_MM                   1000  // Device will only send if accuray is better than this
#define NAVIGATION_FREQUENCY_HZ                        20    // Set solution output to x times a second 
#define CONNECTION_TIMEOUT_MS                       10000
/* 
The module supports RTK update frequencies ranging from 8 Hz (BeiDou, Galileo, GLONASS, GPS) to 
20 Hz (GPS only), velocity and dynamic heading accuracies of 0.05 m/s and 0.3° respectively and a 
convergence time of less than 10 s. RTK performance is characterised by a circular error probable (CEP) 
to 10 mm + 1 ppm. The F9 engine supports a total of 184-channels (GPS L1C/A L2C, GLO L1OF L2OF, 
GAL E1B/C E5b, BDS B1I B2I, QZSS L1C/A L1S L2C and SBAS L1C/A).

Source: Broekman A, Gräbe PJ. A low-cost, mobile real-time kinematic geolocation service for engineering and 
research applications. HardwareX. 2021 May 19;10:e00203. doi: 10.1016/j.ohx.2021.e00203. PMID: 35607668; 
PMCID: PMC9123378. https://www.ncbi.nlm.nih.gov/pmc/articles/PMC9123378/
*/

/*
=================================================================================
                          Button(s) settings
=================================================================================
*/
// Button to press to wipe out stored WiFi and RTK credentials
#define WIPE_BUTTON_PIN                    15

// Reset button is just a hardware connection (EN -> GND)

#endif /*** RTK_ROVER_CONFIG_H ***/






