
/*******************************************************************************
 * @file main.cpp
 * @authors Markus Hädrich
 * <br>
 * @brief This is part of a distributed software, here: head tracker and GNSS 
 *        positioning using Sparkfun Real Time Kinematics
 * <br>
 * @todo  - Task to check / reconnect WiFi (independent of head tracking)
 *        - Calibration button (?)
 *        - Test: BNO080 found/connected
 *        - Test: BLE 
 *        - Status led for WiFi/BLE? on the device box or monitoring in app only?
 *        - Buzzer peep tone if lipo runs out of energy or show an blinky icon/notification in App
 *        
 * @note How to handle WiFi: 
 *        - Push the wipeButton, this will delete old entries in LittleFS files
 *        - Join the AP thats appearing 
 *            -# SSID: RTK-Rover now, later with more devices e. g. "RTKRover_" + ChipID 
 *            -# PW: e. g. "12345678"
 *        - Open address 192.168.4.1 in your browser and set credentials you are 
 *          using for you personal access point on your smartphone
 *        - If the process is done, the LED turns off and the device reboots
 *        - If there are no Wifi credentials stored in the LittleFS, the device 
 *          will jump in AP mode on startup
 * 
 *       How to measure battery:
 *        - First:  Since the ADC2 module is also used by the Wi-Fi, only one of 
 *                  them could get the preemption when using together, which means 
 *                  the adc2_get_raw() may get blocked until Wi-Fi stops, and 
 *                  vice versa. (https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/adc.html)
 *        - Second: Easiest way, use a fuel gauge breakout board e. g. Adafruit_LC709203F
 *                  Complicated way, implement an alternating usage of WiFi and ADC2
 * 
 * @version 0.43
 ******************************************************************************/


#include <Arduino.h>
#include <Wire.h> // BNO080 and uBlox GNSS
#include <BLEDevice.h>
#include <BLE2902.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
#include <SparkFun_BNO080_Arduino_Library.h>
#include <utility/imumaths.h>
#include <sdkconfig.h>
#include <RTKRoverConfig.h>
#include <CasterSecrets.h>
#include <RTKRoverManager.h>
#include <TestsRTKRover.h>

using namespace RTKRoverManager;

/*
=================================================================================
                                Buttons
=================================================================================
*/
#include "Button2.h"

// Button to press to wipe out stored WiFi credentials
Button2 wipeButton = Button2(WIPE_BUTTON_PIN, INPUT, false, false);

void buttonHandler(Button2 &btn);

/*
=================================================================================
                                Battery
=================================================================================
*/
// Messure half the battery voltage
#define BAT_PIN                    A13
/**
 * @brief Get the Battery Volts 
 * 
 * @return float Battery voltage
 */
float getBatteryVolts(void);

/*
=================================================================================
                                WiFi
=================================================================================
*/
AsyncWebServer server(80);
String scannedSSIDs[MAX_SSIDS];

/*
=================================================================================
                                Bluetooth LE
=================================================================================
*/
float bleConnected = false; // TODO: deglobalize this

class MyCharacteristicCallbacks: public BLECharacteristicCallbacks 
{
    void onWrite(BLECharacteristic *pHeadtrackerCharacteristic) 
    {   
        std::string value = pHeadtrackerCharacteristic->getValue(); // Here I get the commands from the App (client)

        if (value.length() > 0) 
        {
            DBG.println(F("*********"));
            DBG.print(F("New value: "));
            for (int i = 0; i < value.length(); i++)
                DBG.print(value[i]);

            DBG.println();
            DBG.println(F("*********"));
        }
     }

     void onConnect(BLEServer* pServer) 
     {
        bleConnected = true;
        DBG.print(F("bleConnected: "));
        DBG.println(bleConnected);
     };

    void onDisconnect(BLEServer* pServer) 
    {
        bleConnected = false;
        DBG.print(("bleConnected: "));
        DBG.println(bleConnected);
    }
};

class MyServerCallbacks: public BLEServerCallbacks 
{
    void onConnect(BLEServer* pServer) 
    {
        bleConnected = true;
        BLEDevice::stopAdvertising();
    };

    void onDisconnect(BLEServer* pServer) 
    {
        bleConnected = false;
        BLEDevice::startAdvertising();
    }
};

BLECharacteristic *pHeadtrackerCharacteristic;
BLECharacteristic *pRealtimeKinematicsCharacteristic;
BLECharacteristic *pRTKAccuracyCharacteristic;

void setupBLE(void);
/*
=================================================================================
                                BNO080
=================================================================================
*/
BNO080 bno080;
void setupBNO080(void);

/*
=================================================================================
                                GNSS
=================================================================================
*/
long lastTime = 0; //Simple local timer. Limits amount if I2C traffic to Ublox module.
bool beginPositioning = false;  // Wait with positioning for first correction data from caster

// The ESP32 core has a built in base64 library but not every platform does
// We'll use an external lib if necessary.
#if defined(ARDUINO_ARCH_ESP32)
#include "base64.h" //Built-in ESP32 library
#else
#include <Base64.h> //nfriendly library from https://github.com/adamvr/arduino-base64, will work with any platform
#endif


SFE_UBLOX_GNSS myGNSS;

/**
 * @brief Setup the ZED-F9D to a rover
 * 
 * @return true If succeeded
 * @return false If failed
 */
bool setupGNSS(void);

/**
 * @brief Start the
 * 
 */
void beginClient(void);

/**
 * @brief Get the Position
 * 
 */
void updatePosition(void);

/*
=================================================================================
                                FreeRTOS
=================================================================================
*/
typedef struct Coord
{
  int32_t lat;
  int8_t  latHp;
  int32_t lon;
  int8_t  lonHp;
} coord_t;

const uint8_t QUEUE_SIZE = 2;
xQueueHandle xQueueAccuracy, xQueueCoord;
static xSemaphoreHandle mutexSem;

/**
 * @brief Task to get the correction data from the caster server
 *        using WiFi
 * 
 * @param pvParameters Void pointer, no parameter used here
 */
void task_rtk_get_corrrection_data(void *pvParameters);

/**
 * @brief Task to get location data
 * 
 * @param pvParameters 
 */

void task_rtk_get_rover_position(void *pvParameters);
/**
 * @brief Task for sending the corrected location data to the
 *        iPhone using BLE
 * 
 * @param pvParameters Void pointer, no parameter used here
 */
void task_send_rtk_position_via_ble(void *pvParameters);

/**
 * @brief Task for sending the BNO080 position data to the
 *        iPhone using BLE
 * 
 * @param pvParameters Void pointer, no parameter used here
 */
void task_bno_orientation_via_ble(void *pvParameters);

/**
 * @brief Create the queues with the right size
 * 
 */
void xQueueSetup(void);

/**
 * @brief Function that blinks one time
 * 
 * @param blinkTime       Blink time in ms
 * @param doNotBlock  Kind of delay between blinking
 */
void blinkOneTime(int blinkTime, bool doNotBlock);

/**
 * @brief Deletes WiFi station SSID and PW from LittleFS
 * 
 */
void wipeWiFiCredentials(void);

void setup() 
{
  // Board LED used for error codes (written in README.md)
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  #ifdef DEBUGGING
  Serial.begin(BAUD);
  while (!Serial) {};
  #endif

 //===============================================================================
  // Initialize LittleFS
  // Use board_build.partitions in platformio.ini
  if (!setupLittleFS()) 
  {
    formatLittleFS();
    if (!setupLittleFS()) while (true) {};
  }

  // The following function clear the file system and that will cause a setup as AP to enter new data

  // Uncomment if you want to format (e. g after changing partition sizes)
  // (And dont forget to comment this again after one run ;)
  //formatLittleFS();

  // wipeWiFiCredentials();  // Use this for deleting WiFi data only
  // wipeLittleFSFiles();  // Use this for deleting all data
#ifdef DEBUGGING
  listFiles();
  delay(3000);
#endif

  //===============================================================================
  // Wifi setup AP or STATION, depending on data in LittleFS
  setupWiFi(&server);
  delay(1000);

  while (WiFi.getMode() == WIFI_AP) 
  {
    DBG.println(F("Enter Wifi credentials on webform:"));
    DBG.print(F("Connect your computer to SSID: "));
    DBG.println(WiFi.getHostname());
    DBG.print(F("Go with your Browser to IP: "));
    DBG.println(WiFi.softAPIP());
    blinkOneTime(1000, false);
    blinkOneTime(100, false);
  }
  if (WiFi.getMode() == WIFI_STA) 
  {
    while (! WiFi.isConnected())
    {
      DBG.println(F("setup(): Try reconnect to WiFi station"));
      WiFi.reconnect();
      DBG.printf("WiFi state: %s", WiFi.isConnected() ? "connected" : "disconnected");
      blinkOneTime(1000, false);
      blinkOneTime(100, false);
    }
  }
//===============================================================================
  
  setupBLE();
  
  DBG.print(F("Device type: ")); DBG.println(DEVICE_TYPE);
  DBG.print(F("Battery: "));
  DBG.print(getBatteryVolts());  
  DBG.println(" V");

  wipeButton.setPressedHandler(buttonHandler); // Pull down method is done in wipeButton init

  // FreeRTOS
  mutexSem = xSemaphoreCreateMutex();
  xQueueSetup();
/*  
  Stack sizes of the tasks. You have to measure the used size in the task (set a high value for first run) and 
  after that you can reduce the stack size to an fitting smaller value. This have to repeated if 
  the task code is changed. There are no rules, just measure and adjust (thats why its a magic number).
  For measurement you need to uncomment the uxHighWaterMark related code in the task (setup and loop).
  After measurement comment out it again.
*/
  int stack_size_task_rtk_get_corrrection_data = 1024 * 7;          // Last measurement: 
  int stack_size_task_rtk_get_rover_position = 1024 * 7;      // Last measurement: 5844 
  int stack_size_task_bno_orientation_via_ble = 1024 * 11;  // Last measurement:
  int stack_size_task_send_rtk_position_via_ble = 1024 * 10;     // Last measurement: 9480
  
  xTaskCreatePinnedToCore( &task_rtk_get_corrrection_data, "task_rtk_get_corrrection_data", stack_size_task_rtk_get_corrrection_data, NULL, TASK_RTK_GET_CORR_DATA_PRIORITY, NULL, RUNNING_CORE_0);
  xTaskCreatePinnedToCore( &task_rtk_get_rover_position, "task_rtk_get_rover_position", stack_size_task_rtk_get_rover_position, NULL, TASK_RTK_GET_POSITION_PRIORITY, NULL, RUNNING_CORE_0);
  xTaskCreatePinnedToCore( &task_bno_orientation_via_ble, "task_bno_orientation_via_ble", stack_size_task_bno_orientation_via_ble, NULL, TASK_BNO080_VIA_BLE_PRIORITY, NULL, RUNNING_CORE_1);
  xTaskCreatePinnedToCore( &task_send_rtk_position_via_ble, "task_send_rtk_position_via_ble", stack_size_task_send_rtk_position_via_ble, NULL, TASK_RTK_POSITION_VIA_BLE_PRIORITY, NULL, RUNNING_CORE_1);
  
  String thisBoard = ARDUINO_BOARD;
  DBG.print(F("Setup done on "));
  DBG.println(thisBoard);
}

void loop() 
{
  #ifdef DEBUGGING
  aunit::TestRunner::run();
  #endif

  wipeButton.loop();
}

/*
=================================================================================
                                GNSS
=======================================_==========================================
*/
bool setupGNSS() 
{
    while (!Wire1.begin(RTK_SDA_PIN, RTK_SCL_PIN)) 
    {
      DBG.println(F("I2C for RTK not running, check cable!"));
      delay(500);
    }

    Wire1.setClock(I2C_FREQUENCY_400K);

    while (myGNSS.begin(Wire1, RTK_I2C_ADDR) == false)
    {
      DBG.println(F("u-blox GNSS not detected at default I2C address. Please check wiring. Freezing loop."));
      blinkOneTime(500, false);
    }

    bool response = true;
    //Turn off NMEA noise
    response &= myGNSS.setI2COutput(COM_TYPE_UBX); 
    //Be sure RTCM3 input is enabled. UBX + RTCM3 is not a valid state.
    response &= myGNSS.setPortInput(COM_PORT_I2C, COM_TYPE_UBX | COM_TYPE_NMEA | COM_TYPE_RTCM3); 
    response &= myGNSS.setHighPrecisionMode(true);
    // Set output in Hz.
    response &= myGNSS.setNavigationFrequency(NAVIGATION_FREQUENCY_HZ); 
    byte rate = myGNSS.getNavigationFrequency(); //Get the update rate of this module
    DBG.print("Current update rate: ");
    DBG.println(rate);

    return response;
}

void updatePosition() 
{
  static int32_t old_accuracy = -1;
  coord_t coord;

  myGNSS.checkUblox();

  int32_t lat = myGNSS.getHighResLatitude();
  int8_t latHp = myGNSS.getHighResLatitudeHp();
  int32_t lon = myGNSS.getHighResLongitude();
  int8_t lonHp = myGNSS.getHighResLongitudeHp();
  int32_t accuracy = myGNSS.getPositionAccuracy();

  coord = {.lat = lat, .latHp = latHp, .lon = lon, .lonHp = lonHp};
  xQueueSend(xQueueCoord, &coord, portMAX_DELAY); 

  // Send accuracy if changed only
  if (accuracy != old_accuracy)
  {
    xQueueSend( xQueueAccuracy, &accuracy, portMAX_DELAY );
    old_accuracy = accuracy;
  }

}

/*
=================================================================================
                                FreeRTOS
=================================================================================
*/

void task_rtk_get_rover_position(void *pvParameters) 
{
  (void)pvParameters;

  // Measure stack size
  UBaseType_t uxHighWaterMark; 

  // Wait for first correction data
  while ( ! beginPositioning) { vTaskDelay(1000/portTICK_PERIOD_MS); }

  while (true)
  {
    if (xSemaphoreTake(mutexSem, portMAX_DELAY))
    {
      updatePosition();

      // Measure stack size (last was 2304)
      // uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
      // DBG.print(F("task_rtk_get_rover_position loop, uxHighWaterMark: "));
      // DBG.println(uxHighWaterMark);

      xSemaphoreGive(mutexSem);
    }
   
    vTaskDelay(TASK_RTK_GET_POSITION_INTERVAL_MS/portTICK_PERIOD_MS);
  }
  vTaskDelete(NULL);
}

void task_rtk_get_corrrection_data(void *pvParameters) 
{
  (void)pvParameters;

  if (!setupGNSS()) 
  { 
    DBG.println("setupGNSS() failed! Restart in 10 s");
    while (true) 
    {
      static const int timeToReboot = 10000;
      static int counter = 0;
      blinkOneTime(1000, true);
      counter++;
      if (counter > 5) ESP.restart();
    }
  };

  while ( ! checkConnectionToWifiStation() )
  {
    DBG.println(F("task setup: Not connected to WiFi station"));
    blinkOneTime(1000, true);
    blinkOneTime(100, true);
  }

//=========================================================================
  // 5 RTCM messages take approximately ~300ms to arrive at 115200bps
  long lastReceivedRTCM_ms = 0; 
  // If we fail to get a complete RTCM frame after 10s, then disconnect from caster
  const int maxTimeBeforeHangup_ms = 10000; 

  // Measure stack size
  UBaseType_t uxHighWaterMark; 

  // Read RTK credentials
  String casterHost = readFile(LittleFS, getPath(PARAM_RTK_CASTER_HOST).c_str());
  String casterPort = readFile(LittleFS, getPath(PARAM_RTK_CASTER_PORT).c_str());
  String casterUser = readFile(LittleFS, getPath(PARAM_RTK_CASTER_USER).c_str());
  String mountPoint =  readFile(LittleFS, getPath(PARAM_RTK_MOINT_POINT).c_str());
  String casterUserPW = kCasterUserPw; // No password needed, but it is defined in CasterSecrets.h
  
  // Check RTK credentials
  bool credentialsExists = true;
  credentialsExists &= !casterHost.isEmpty();
  credentialsExists &= !casterPort.isEmpty();
  credentialsExists &= !casterUser.isEmpty();
  credentialsExists &= !mountPoint.isEmpty();

  while (!credentialsExists) 
  {
    DBG.println(F("RTK credentials incomplete!\nFreezing RTK task."));
    blinkOneTime(2000, true);
  }

  WiFiClient ntripClient;
  long rtcmCount = 0;

  while (true) // Task loop begins
  {
    /** This ist most of the content beginServing() func from the
     * Sparkfun u-blox GNSS Arduino Library/ZED-F9P/Example15-NTRIPClient 
     * Because I did not wanted to change the code too much if you want to compare
     * with the Example14 I used of the "evil" goto as a replace for the return command.
     * (A task must not return.)
     */
    
      if (ntripClient.connected() == false)
      {
        // First check WiFi connection
        while ( ! checkConnectionToWifiStation() ) 
        {
          DBG.println(F("task loop: Not connected to WiFi station"));
          blinkOneTime(1000, true);
          blinkOneTime(100, true);
        }

        DBG.print(F("Opening socket to "));
        DBG.println(casterHost.c_str());

        // Attempt connection
        if (ntripClient.connect( casterHost.c_str(), (uint16_t)casterPort.toInt() ) == false) 
        {
          DBG.println(F("Connection to caster failed, retry in 5s"));
          vTaskDelay(5000/portTICK_PERIOD_MS);
      
          goto task_end; // replaces the return command from the SparkFun example (a task must not return)
        }
        else
        {
          DBG.print(F("Connected to "));
          DBG.print(casterHost.c_str());
          DBG.print(F(": "));
          DBG.println((uint16_t)casterPort.toInt());

          DBG.print(F("Requesting NTRIP Data from mount point "));
          DBG.println(mountPoint.c_str());

          const int SERVER_BUFFER_SIZE = 512;
          char serverRequest[SERVER_BUFFER_SIZE];

          snprintf(serverRequest, SERVER_BUFFER_SIZE, "GET /%s HTTP/1.0\r\nUser-Agent: NTRIP SparkFun u-blox Client v1.0\r\n",
                  mountPoint.c_str());

          char credentials[512];
          if (strlen(casterUser.c_str()) == 0)
          {
            strncpy(credentials, "Accept: */*\r\nConnection: close\r\n", sizeof(credentials));
          }
          else
          {
            //Pass base64 encoded user:pw
            char userCredentials[(casterUser.length()+1) + sizeof(casterUserPW) + 1]; //The ':' takes up a spot
            snprintf(userCredentials, sizeof(userCredentials), "%s:%s", casterUser.c_str(), casterUserPW);

            DBG.print(F("Sending credentials: "));
            DBG.println(userCredentials);

  #if defined(ARDUINO_ARCH_ESP32)
            // Encode with ESP32 built-in library
            base64 b;
            String strEncodedCredentials = b.encode(userCredentials);
            char encodedCredentials[strEncodedCredentials.length() + 1];
            strEncodedCredentials.toCharArray(encodedCredentials, sizeof(encodedCredentials)); //Convert String to char array
            snprintf(credentials, sizeof(credentials), "Authorization: Basic %s\r\n", encodedCredentials);
  #else
            // Encode with nfriendly library
            int encodedLen = base64_enc_len(strlen(userCredentials));
            char encodedCredentials[encodedLen]; //Create array large enough to house encoded data
            base64_encode(encodedCredentials, userCredentials, strlen(userCredentials)); //Note: Input array is consumed
  #endif
          }

          // This warning comes because source and destination have the same size, 
          // but it is large enough and the buffer should not be full at any time.
          strncat(serverRequest, credentials, SERVER_BUFFER_SIZE);
          strncat(serverRequest, "\r\n", SERVER_BUFFER_SIZE);
          DBG.printf("serverRequest len: %d", strlen(serverRequest));
          DBG.print(F("serverRequest size: "));
          DBG.print(strlen(serverRequest));
          DBG.print(F(" of "));
          DBG.print(sizeof(serverRequest));
          DBG.println(F(" bytes available"));

          DBG.println(F("Sending server request:"));
          DBG.println(serverRequest);
          ntripClient.write(serverRequest, strlen(serverRequest));
          
          // Wait for response
          unsigned long timeout = millis();
          while (ntripClient.available() == 0)
          {
            if (millis() - timeout > CONNECTION_TIMEOUT_MS)
            {
              ntripClient.stop(); // Too many requests with wrong settings will lead to bann, stop here
              Serial.println(F("Caster timed out!"));

              goto task_end;
            }
            vTaskDelay(1000/portTICK_PERIOD_MS);
          }

          // Check reply
          bool connectionSuccess = false;
          char response[512];
          int responseSpot = 0;

          while (ntripClient.available())
          {
            if (responseSpot == sizeof(response) - 1) break;

            response[responseSpot++] = ntripClient.read();
            if (strstr(response, "200") > 0) // Look for 'ICY 200 OK'
              connectionSuccess = true;
            if (strstr(response, "401") > 0) // Look for '401 Unauthorized'
            {
              DBG.println(F("Your credentials look bad!\nCheck you caster username, password and ban status (got email from rtk2go?)"));
              connectionSuccess = false;
            }
          }
          response[responseSpot] = '\0';

          DBG.print(F("Caster responded with: "));
          DBG.println(response);

          if (connectionSuccess == false)
          {
            DBG.print(F("Failed to connect to "));
            DBG.print(casterHost.c_str());
            DBG.print(F(": "));
            DBG.println(response);

            goto task_end; // replaces the return command from the SparkFun example (a task must not return)
          }
          else
          {
            DBG.print(F("Connected to "));
            DBG.println(casterHost.c_str());
            lastReceivedRTCM_ms = millis(); // Reset timeout
          }
        } // End attempt to connect
      } // End connected == false

      if (ntripClient.connected() == true)
      {
        uint8_t rtcmData[512 * 4]; // Most incoming data is around 500 bytes but may be larger
        rtcmCount = 0;

        //Print any available RTCM data
        while (ntripClient.available())
        {
          //DBG.write(ntripClient.read()); // Pipe to serial port is fine but beware, it's a lot of binary data
          rtcmData[rtcmCount++] = ntripClient.read();
          if (rtcmCount == sizeof(rtcmData)) break;
        }

        if (rtcmCount > 0)
        {
          //Push RTCM to GNSS module over I2C
          if (xSemaphoreTake(mutexSem, portMAX_DELAY))
          {
            myGNSS.pushRawData(rtcmData, rtcmCount, false);
            beginPositioning = true;
            xSemaphoreGive(mutexSem);
            DBG.print(F("RTCM pushed to ZED: "));
            DBG.println(rtcmCount);
            uint32_t currentTime = millis();
            DBG.print(F("Last data before ms: "));
            DBG.println(currentTime - lastReceivedRTCM_ms);
            lastReceivedRTCM_ms = currentTime;
          }

          
          // updatePosition(); //This is done now in a dedicated task
        }
      }   // End (ntripClient.connected() == true)

      // Close socket if we don't have new data for 10s
      if (millis() - lastReceivedRTCM_ms > maxTimeBeforeHangup_ms)
      {
        DBG.println(F("RTCM timeout. Disconnecting..."));
        if (ntripClient.connected() == true)
          ntripClient.stop();
      }

      // Measure stack size (last was 19320)
      // uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
      // DBG.print(F("task_rtk_get_corrrection_data loop, uxHighWaterMark: "));
      // DBG.println(uxHighWaterMark);
    // } /*** End if (xSemaphoreTake(mutexSem, portMAX_DELAY)) ***/
    
    task_end:
    
    // Wait a bit before the next request will be started
    vTaskDelay(TASK_WIFI_RTK_DATA_INTERVAL_MS/portTICK_PERIOD_MS);
  }

  // Delete self task
  vTaskDelete(NULL);

} /*** end task_rtk_get_corrrection_data ***/

/*
=================================================================================
                                BLE
=================================================================================
*/
void setupBLE(void)
{   
    String deviceName = getDeviceName(DEVICE_TYPE);
    BLEDevice::init(deviceName.c_str());
    BLEServer *pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks()); 
    BLEService *pService = pServer->createService(SERVICE_UUID);
    // Create characteristics
    pHeadtrackerCharacteristic = pService->createCharacteristic(
                                         HEADTRACKER_CHARACTERISTIC_UUID,
                                        //  BLECharacteristic::PROPERTY_READ   |
                                        //  BLECharacteristic::PROPERTY_WRITE  |
                                        //  BLECharacteristic::PROPERTY_INDICATE |
                                         BLECharacteristic::PROPERTY_NOTIFY  // We only use notify characteristic (fastest -> no response)
                                       );

    pRealtimeKinematicsCharacteristic = pService->createCharacteristic(
                                         REALTIME_KINEMATICS_CHARACTERISTIC_UUID,
                                        //  BLECharacteristic::PROPERTY_READ   |
                                        //  BLECharacteristic::PROPERTY_WRITE  |
                                        //  BLECharacteristic::PROPERTY_INDICATE |
                                         BLECharacteristic::PROPERTY_NOTIFY  // We only use notify characteristic (fastest -> no response)
                                       );
                                    
    pRTKAccuracyCharacteristic = pService->createCharacteristic(
                                RTK_ACCURACY_CHARACTERISTIC_UUID,
                                //  BLECharacteristic::PROPERTY_READ   |
                                //  BLECharacteristic::PROPERTY_WRITE  |
                                //  BLECharacteristic::PROPERTY_INDICATE |
                                BLECharacteristic::PROPERTY_NOTIFY // We only use notify characteristic (fastest -> no response)
                                );

    pHeadtrackerCharacteristic->addDescriptor(new BLE2902());
    pHeadtrackerCharacteristic->setCallbacks(new MyCharacteristicCallbacks());
    pHeadtrackerCharacteristic->setValue(deviceName.c_str());

    pRealtimeKinematicsCharacteristic->addDescriptor(new BLE2902());
    // pRealtimeKinematicsCharacteristic->setCallbacks(new MyCharacteristicCallbacks());
    // pRealtimeKinematicsCharacteristic->setValue(deviceName.c_str());
    
    pRTKAccuracyCharacteristic->addDescriptor(new BLE2902());

    pService->start();
    BLEAdvertising *pAdvertising = pServer->getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    pAdvertising->setScanResponse(true);
    pAdvertising->setMinPreferred(0x12);  // 0x06 x 1.25 ms = 7.5 ms, functions that help with iPhone connections issue
    pAdvertising->setMaxPreferred(0x24);  // 30 ms
    //pAdvertising->start();
    BLEDevice::startAdvertising();
    DBG.println(F("Characteristic defined! Now you can read it in your phone!"));
}

void setupBNO080()
{   
  Wire.begin();
  while (!bno080.begin()) 
  {
    // Wait
    DBG.println(F("BNO080 not ready, waiting for I2C..."));
    delay(500);
  }
    
  // Activate IMU functionalities
  bno080.enableARVRStabilizedRotationVector(BNO080_ROT_VECT_UPDATE_RATE_MS); 
  // bno080.enableARVRStabilizedGameRotationVector(BNO080_ROT_VECT_UPDATE_RATE_MS);
  // bno080.enableRotationVector(BNO080_ROT_VECT_UPDATE_RATE_MS);  
  // bno080.enableGameRotationVector(BNO080_ROT_VECT_UPDATE_RATE_MS);  
  
  bno080.enableAccelerometer(BNO080_LIN_ACCEL_UPDATE_RATE_MS);    
  bno080.enableLinearAccelerometer(BNO080_LIN_ACCEL_UPDATE_RATE_MS);    
  // bno080.enableStepCounter(20);   // Thomas: Funktioniert sehr schlecht.. 
  
  // Markus: --> timeBetweenReports should not be 20 ms ;)  try this: 31.25 Hz
  // bno080.enableStepCounter(32);      
}

void xQueueSetup() 
{
  xQueueCoord  = xQueueCreate( QUEUE_SIZE, sizeof( coord_t ) );
  xQueueAccuracy  = xQueueCreate( QUEUE_SIZE, sizeof( long ) );
}

void task_send_rtk_position_via_ble(void *pvParameters) 
{
  (void)pvParameters;
  
  String latLonStr((char *)0);
  // String accuracyMmStr((char *)0);
  String accuracyStr((char *)0);
  // Latitude: 9, delimiter: 1, latitudeHp: 2, longitude: 9, delimiter: 1, longitudeHp: 2,
  latLonStr.reserve(27);
  accuracyStr.reserve(5); 
 
  coord_t coord;
  int32_t lat, lon, accuracy;
  int8_t latHp, lonHp;

  while (!bleConnected) blinkOneTime(100, true);

  UBaseType_t uxHighWaterMark;
  // uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
  // DBG.print(F("task_send_rtk_position_via_ble setup, uxHighWaterMark: "));
  // DBG.println(uxHighWaterMark);

  while (true) 
  {
    if (bleConnected) 
    {
      if (xQueueReceive( xQueueCoord, &coord, ( TickType_t ) 10 ) == pdPASS) 
      {
        // DBG.print("Received coord.lat = ");
        // DBG.print(coord.lat);
        // DBG.print(", coord.latHp = ");
        // DBG.print(coord.latHp);
        // DBG.print(" coord.lon = ");
        // DBG.print(coord.lon);
        // DBG.print(", coord.lonHp = ");
        // DBG.println(coord.lonHp);
        lat = coord.lat;
        latHp = coord.latHp;
        lon = coord.lon;
        lonHp = coord.lonHp;

        // Send coords
        latLonStr = String(lat);
        latLonStr += DATA_STR_DELIMITER;
        latLonStr += String(latHp);
        latLonStr += DATA_STR_DELIMITER;
        latLonStr += String(lon);
        latLonStr += DATA_STR_DELIMITER;
        latLonStr += String(lonHp);
        // DBG.print(F("latLonStr.length(): "));DBG.println(latLonStr.length());
        pRealtimeKinematicsCharacteristic->setValue(latLonStr.c_str());
        pRealtimeKinematicsCharacteristic->notify();
      }

      if (xQueueReceive( xQueueAccuracy, &accuracy, ( TickType_t ) 10 ) == pdPASS) 
      {
        accuracyStr = String(accuracy);
        // DBG.print(F("accuracyStr.length(): "));DBG.println(accuracyStr.length());
        pRTKAccuracyCharacteristic->setValue(accuracyStr.c_str());
        pRTKAccuracyCharacteristic->notify();

        // DBG.print(F("Received accuracy = "));
        // DBG.print(accuracy);
        // DBG.println(F(" mm"));
      }
    
      /*  Measure stack size (last was 9356) */
      // uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
      // DBG.print(F("task_send_rtk_position_via_ble loop, uxHighWaterMark: "));
      // DBG.println(uxHighWaterMark);

    } /*** if (bleConnected) ends ***/
    else
    {
      blinkOneTime(100, true);
    }
 

    vTaskDelay(TASK_RTK_BLE_INTERVAL_MS/portTICK_PERIOD_MS);
    // taskYIELD();
  } // while (true) ends
  
  // Delete self task
  vTaskDelete(NULL);
}

void task_bno_orientation_via_ble(void *pvParameters) 
{
    (void)pvParameters;

    while (!bleConnected) 
    {
      DBG.println(F("BNO tasks setup: Open RWA to connect BLE"));
      vTaskDelay(1000/portTICK_PERIOD_MS);
    }

    setupBNO080();

    float quatI, quatJ, quatK, quatReal, yawDegreeF, pitchDegreeF, linAccelZF;// rollDegreeF;
    int pitchDegree, yawDegree;// rollDegree;
    String dataStr((char *)0);
    // String size: (yaw: 3, delimiter: 1, pitch: 3, delimiter: 1, linAccelZF: 4) = 12 + LIN_ACCEL_Z_DECIMAL_DIGITS
    dataStr.reserve(12 + LIN_ACCEL_Z_DECIMAL_DIGITS);
    
    // Measure stack size
    UBaseType_t uxHighWaterMark;
    // uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
    // DBG.print(F("task_bno_orientation_via_ble setup, uxHighWaterMark: "));
    // DBG.println(uxHighWaterMark);

    while (true) 
    {
      if (!bleConnected) 
      {
        DBG.println(F("BNO tasks loop: Please connect BLE"));
        vTaskDelay(1000/portTICK_PERIOD_MS);
      }
      else 
      {
        // TODO: Separate reading values from sending values
        if (bno080.dataAvailable()) 
        {         
          quatI = bno080.getQuatI();
          quatJ = bno080.getQuatJ();
          quatK = bno080.getQuatK();
          quatReal = bno080.getQuatReal();       

          imu::Quaternion quat = imu::Quaternion(quatReal, quatI, quatJ, quatK);
          quat.normalize();
          imu::Vector<3> q_to_euler = quat.toEuler();     
          yawDegreeF = q_to_euler.x();
          yawDegreeF = yawDegreeF * -180.0 / M_PI;   // conversion to Degree
              
          if ( yawDegreeF < 0 ) yawDegreeF += 359.0; // convert negative to positive angles
          
          yawDegree = (int)(round(yawDegreeF));  
          
          pitchDegreeF = q_to_euler.z();
          pitchDegreeF = pitchDegreeF * -180.0 / M_PI;
          pitchDegree = (int)(round(pitchDegreeF));

          // rollDegreeF = q_to_euler.y();
          // rollDegreeF = rollDegreeF * -180.0 / M_PI;
          // rollDegree = (int)(round(rollDegreeF));

          // Seems to be much slower than bno080.getAccelZ()
          linAccelZF = bno080.getLinAccelZ();  

          dataStr = String(yawDegree) + DATA_STR_DELIMITER + String(pitchDegree) \
                  + DATA_STR_DELIMITER + String(linAccelZF, LIN_ACCEL_Z_DECIMAL_DIGITS);
          pHeadtrackerCharacteristic->setValue(dataStr.c_str());
          pHeadtrackerCharacteristic->notify();
          // DBG.println(linAccelZF);      
          } 
          else 
          {
            DBG.println(F("Ready for BNO080 dataAvailable"));
            vTaskDelay(1000/portTICK_PERIOD_MS);
          }
          // Measure stack size
          // uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
          // DBG.print(F("task_bno_orientation_via_ble loop, uxHighWaterMark: "));
          // DBG.println(uxHighWaterMark);

        }
        vTaskDelay(TASK_BNO_ORIENTATION_VIA_BLE_INTERVAL_MS/portTICK_PERIOD_MS);
        // taskYIELD(); // 11.25 ms is the BLE connection interval, makes no sense to try to send faster
     }
    // Delete self task
    vTaskDelete(NULL);

} /*** end task_bno_orientation_via_ble ***/

/*
=================================================================================
                                Battery
=================================================================================
*/
float getBatteryVolts() 
{
  // Vout = Dout * Vmax / Dmax 
  // Because battery volts are higher than Vmax, we use the voltage devider on 
  // Pin A13 (Huzzah ESP32, it may be different on other boards) 
  float batteryVolts = 2.0 * (analogRead(BAT_PIN) * 3.3 / 4095.0);
  return batteryVolts;
}

/*
=================================================================================
                                Button(s)
=================================================================================
*/
void buttonHandler(Button2 &btn) 
{
  if (btn == wipeButton) 
  {
    digitalWrite(LED_BUILTIN, HIGH);
    
    // Clear whole memory
    //DBG.println(F("Wiping whole memory..."));
    //wipeLittleFSFiles();

    // OR
    // clear just WiFi credentials
    DBG.println(F("Wiping WiFi credentials from memory..."));
    clearPath(getPath(PARAM_WIFI_SSID).c_str());
    clearPath(getPath(PARAM_WIFI_PASSWORD).c_str());
    
    ESP.restart();
  }
}

void wipeWiFiCredentials()
{
  clearPath(getPath(PARAM_WIFI_SSID).c_str());
  clearPath(getPath(PARAM_WIFI_PASSWORD).c_str());
}

void blinkOneTime(int blinkTime, bool doNotBlock)
{
  digitalWrite(LED_BUILTIN, HIGH);
  doNotBlock ? vTaskDelay(blinkTime) : delay(blinkTime);
  digitalWrite(LED_BUILTIN, LOW);
  doNotBlock ? vTaskDelay(blinkTime) : delay(blinkTime);
}