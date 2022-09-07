
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
 *        - Push the wipeButton, this will delete old entries in SPIFFS files
 *        - Join the AP thats appearing 
 *            -# SSID: RTK-Rover now, later with more devices e. g. "RTKRover_" + ChipID 
 *            -# PW: e. g. "12345678"
 *        - Open address 192.168.4.1 in your browser and set credentials you are 
 *          using for you personal access point on your smartphone
 *        - If the process is done, the LED turns off and the device reboots
 *        - If there are no Wifi credentials stored in the SPIFFS, the device 
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
#include <imumaths.h>
#include <sdkconfig.h>
#include <RTKRoverConfig.h>
#include <CasterSecrets.h>
#include <RTKRoverManager.h>

using namespace RTKRoverManager;

String deviceName = getDeviceName(DEVICE_TYPE);

/*******************************************************************************
 *                                 Button(s)
 * ****************************************************************************/
#include "Button2.h"
// Button to press to wipe out stored WiFi credentials
const int BUTTON_PIN = 15;
Button2 wipeButton = Button2(BUTTON_PIN, INPUT, false, false);

void buttonHandler(Button2 &btn);

/*******************************************************************************
 *                                   Battery
 * ****************************************************************************/
// Messure half the battery voltage
#define BAT_PIN                    A13
float getBatteryVolts(void);

/*******************************************************************************
 *                                 WiFi
 * ****************************************************************************/
AsyncWebServer server(80);
String scannedSSIDs[MAX_SSIDS];

/*******************************************************************************
 *                                 Bluetooth LE
 * ****************************************************************************/
float bleConnected = false; // TODO: deglobalize this

class MyCharacteristicCallbacks: public BLECharacteristicCallbacks 
{
    void onWrite(BLECharacteristic *pHeadtrackerCharacteristic) 
    {   
        std::string value = pHeadtrackerCharacteristic->getValue(); // Here I get the commands from the App (client)

        if (value.length() > 0) 
        {
            DEBUG_SERIAL.println(F("*********"));
            DEBUG_SERIAL.print(F("New value: "));
            for (int i = 0; i < value.length(); i++)
                DEBUG_SERIAL.print(value[i]);

            DEBUG_SERIAL.println();
            DEBUG_SERIAL.println(F("*********"));
        }
     }

     void onConnect(BLEServer* pServer) 
     {
        bleConnected = true;
        DEBUG_SERIAL.print(F("bleConnected: "));
        DEBUG_SERIAL.println(bleConnected);
     };

    void onDisconnect(BLEServer* pServer) 
    {
        bleConnected = false;
        DEBUG_SERIAL.print(("bleConnected: "));
        DEBUG_SERIAL.println(bleConnected);
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

/*******************************************************************************
 *                                 BNO080
 * ****************************************************************************/
BNO080 bno080;
void setupBNO080(void);

/*******************************************************************************
 *                                 GNSS
 * ****************************************************************************/
long lastTime = 0; //Simple local timer. Limits amount if I2C traffic to Ublox module.

//The ESP32 core has a built in base64 library but not every platform does
//We'll use an external lib if necessary.
#if defined(ARDUINO_ARCH_ESP32)
#include "base64.h" //Built-in ESP32 library
#else
#include <Base64.h> //nfriendly library from https://github.com/adamvr/arduino-base64, will work with any platform
#endif


SFE_UBLOX_GNSS myGNSS;

bool setupGNSS(void);
void beginClient(void);
void getPosition(void);

/*******************************************************************************
 *                               FreeRTOS
 * ****************************************************************************/
xQueueHandle xQueueLatitude, xQueueLongitude, xQueueAccuracy;

void task_send_bno080_data_over_ble(void *pvParameters);
void task_get_rtk_corrections_over_wifi(void *pvParameters);
void task_send_rtk_corrections_over_ble(void *pvParameters);
void xQueueSetup(void);

// Globals
WiFiClient ntripClient;

void setup() {
  Wire.begin();
  #ifdef DEBUGGING
  Serial.begin(BAUD);
  while (!Serial) {};
  #endif

  setupBLE();
  setupBNO080();

  bool format = false;
  if (!setupSPIFFS(format)) {
    DEBUG_SERIAL.println(F("setupSPIFFS failed, freezing..."));
    while (true) {};
  }
  DEBUG_SERIAL.print(F("Device name: "));DEBUG_SERIAL.println(DEVICE_NAME);
  
  DEBUG_SERIAL.print(F("Battery: "));
  DEBUG_SERIAL.print(getBatteryVolts());  
  DEBUG_SERIAL.println(" V");

  wipeButton.setPressedHandler(buttonHandler); // INPUT_PULLUP is set too here  
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  
  WiFi.setHostname(DEVICE_NAME);
  // Check if we have credentials for a available network
  String lastSSID = readFile(SPIFFS, PATH_WIFI_SSID);
  String lastPassword = readFile(SPIFFS, PATH_WIFI_PASSWORD);

  if (!savedNetworkAvailable(lastSSID) || lastPassword.isEmpty() ) {
    setupAPMode(AP_SSID, AP_PASSWORD);
    delay(500);
  } else {
   setupStationMode(lastSSID.c_str(), lastPassword.c_str(), DEVICE_NAME);
   delay(500);
 }
  startServer(&server);

  // FreeRTOS
  xQueueSetup();
  xTaskCreatePinnedToCore( &task_get_rtk_corrections_over_wifi, "task_get_rtk_corrections_over_wifi", 1024 * 7, NULL, GNSS_OVER_WIFI_PRIORITY, NULL, RUNNING_CORE_0);
  xTaskCreatePinnedToCore( &task_send_bno080_data_over_ble, "task_send_bno080_data_over_ble", 1024 * 11, NULL, BNO080_OVER_BLE_PRIORITY, NULL, RUNNING_CORE_1);
  xTaskCreatePinnedToCore( &task_send_rtk_corrections_over_ble, "task_send_rtk_corrections_over_ble", 1024 * 10, NULL, BNO080_OVER_BLE_PRIORITY, NULL, RUNNING_CORE_1);
  
  String thisBoard= ARDUINO_BOARD;
  DEBUG_SERIAL.print(F("Setup done on "));
  DEBUG_SERIAL.println(thisBoard);
}

void loop() {
    #ifdef DEBUGGING
    #ifdef TESTING
    DEBUG_SERIAL.println(F("Running Tests..."))
    aunit::TestRunner::run();
    #endif
    #endif

    wipeButton.loop();
}

/*******************************************************************************
 *                                 GNSS
 * ****************************************************************************/

bool setupGNSS() {
    while (myGNSS.begin(Wire1, RTK_I2C_ADDR) == false) {
        DEBUG_SERIAL.println(F("u-blox GNSS not detected at default I2C address. Please check wiring. Freezing loop."));
        delay(1000);
      }
    bool response = true;
    response &= myGNSS.setI2COutput(COM_TYPE_UBX); //Turn off NMEA noise
    response &= myGNSS.setPortInput(COM_PORT_I2C, COM_TYPE_UBX | COM_TYPE_NMEA | COM_TYPE_RTCM3); //Be sure RTCM3 input is enabled. UBX + RTCM3 is not a valid state.
    response &= myGNSS.setNavigationFrequency(NAVIGATION_FREQUENCY_HZ); // Set output in Hz.
    response &= myGNSS.setHighPrecisionMode(true); // TODO: test this
    #ifdef DEBUGGING
    myGNSS.setNMEAOutputPort(Serial);
    #endif

    return response;
}

void getPosition() {
  static long lastRun = millis();
  if (millis() - lastRun > RTK_REFRESH_INTERVAL_MS) {
    lastTime = millis(); //Update the timer

    long latitude = myGNSS.getLatitude();
    xQueueSend( xQueueLatitude, &latitude, portMAX_DELAY );
    DEBUG_SERIAL.print(F("Lat: "));
    DEBUG_SERIAL.print(latitude);

    long longitude = myGNSS.getLongitude();
    xQueueSend( xQueueLongitude, &longitude, portMAX_DELAY );
    DEBUG_SERIAL.print(F(" Lon: "));
    DEBUG_SERIAL.print(longitude);
    DEBUG_SERIAL.println(F(" (degrees * 10^-7)"));

    // long altitude = myGNSS.getAltitude();
    // DEBUG_SERIAL.print(F("Alt: "));
    // DEBUG_SERIAL.print(altitude);
    // DEBUG_SERIAL.println(F(" mm"));

    long accuracy = myGNSS.getPositionAccuracy();
    xQueueSend( xQueueAccuracy, &accuracy, portMAX_DELAY );
    // DEBUG_SERIAL.print(F("3D Positional Accuracy: "));
    // DEBUG_SERIAL.print(accuracy);
    // DEBUG_SERIAL.println(F(" mm"));
  }
}

/*******************************************************************************
 *                                 FreeRTOS
 * ****************************************************************************/

void task_get_rtk_corrections_over_wifi(void *pvParameters) {
    (void)pvParameters;

    // 5 RTCM messages take approximately ~300ms to arrive at 115200bps
    long lastReceivedRTCM_ms = 0; 
    //If we fail to get a complete RTCM frame after 10s, then disconnect from caster
    const int maxTimeBeforeHangup_ms = 10000; 

    while (!Wire1.begin(RTK_SDA_PIN, RTK_SCL_PIN)) {
        DEBUG_SERIAL.println(F("I2C for RTK not running, check cable..."));
        vTaskDelay(1000/portTICK_PERIOD_MS);
    }
    Wire1.setClock(I2C_FREQUENCY_100K);
    if (!setupGNSS()) { 
      DEBUG_SERIAL.println("setupGNSS() failed, freezing");
      while (1) {};
    };

    // Measure stack size
    UBaseType_t uxHighWaterMark; 
    // uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
    // DEBUG_SERIAL.print(F("task_get_rtk_corrections_over_wifi setup, uxHighWaterMark: "));
    // DEBUG_SERIAL.println(uxHighWaterMark);

    // Read credentials
    String casterHost = readFile(SPIFFS, PATH_RTK_CASTER_HOST);
    String casterPort = readFile(SPIFFS, PATH_RTK_CASTER_PORT);
    String casterUser = readFile(SPIFFS, PATH_RTK_CASTER_USER);
    String mountPoint =  readFile(SPIFFS, PATH_RTK_MOINT_POINT);

    // Check RTK credentials
    bool credentialsExists = true;
    credentialsExists &= !casterHost.isEmpty();
    credentialsExists &= !casterPort.isEmpty();
    credentialsExists &= !casterUser.isEmpty();
    credentialsExists &= !mountPoint.isEmpty();

    if (!credentialsExists) {
      DEBUG_SERIAL.println("RTK credentials incomplete, please fill out the web form and reboot!\nFreezing RTK task. ");
      while (true) {};
    }

    // WiFiClient ntripClient;
    long rtcmCount = 0;

    while (true) {
      /** This ist mostl the content beginServing() func from the
       * Sparkfun u-blox GNSS Arduino Library/ZED-F9P/Example15-NTRIPClient 
       * Because I did not wanted to change the code too much if you want to compare
       * with the Example14 I used of the evil goto as a replace for the return command.
       * (A task must not return.)
       */
      taskStart:

      if (ntripClient.connected() == false)
      {
        DEBUG_SERIAL.print(F("Opening socket to "));
        DEBUG_SERIAL.println(casterHost.c_str());

        if (ntripClient.connect(casterHost.c_str(), (uint16_t)casterPort.toInt()) == false) //Attempt connection
        {
          DEBUG_SERIAL.println(F("Connection to caster failed"));
          
          // First check your WiFi connection
          while (!checkConnectionToWifiStation()) {
            delay(1000);
          }
          
          /** Yes, never use goto! But here: it just jumps to the beginning of the task, 
          * because return is forbiddden in tasks. This is the only use of goto in this code.
          */
          goto taskStart; // replaces the return command from the SparkFun example (a task must not return)
        }
        else
        {
          DEBUG_SERIAL.print(F("Connected to "));
          DEBUG_SERIAL.print(casterHost.c_str());
          DEBUG_SERIAL.print(F(": "));
          DEBUG_SERIAL.println((uint16_t)casterPort.toInt());

          DEBUG_SERIAL.print(F("Requesting NTRIP Data from mount point "));
          DEBUG_SERIAL.println(mountPoint.c_str());

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

            DEBUG_SERIAL.print(F("Sending credentials: "));
            DEBUG_SERIAL.println(userCredentials);

  #if defined(ARDUINO_ARCH_ESP32)
            //Encode with ESP32 built-in library
            base64 b;
            String strEncodedCredentials = b.encode(userCredentials);
            char encodedCredentials[strEncodedCredentials.length() + 1];
            strEncodedCredentials.toCharArray(encodedCredentials, sizeof(encodedCredentials)); //Convert String to char array
            snprintf(credentials, sizeof(credentials), "Authorization: Basic %s\r\n", encodedCredentials);
  #else
            //Encode with nfriendly library
            int encodedLen = base64_enc_len(strlen(userCredentials));
            char encodedCredentials[encodedLen]; //Create array large enough to house encoded data
            base64_encode(encodedCredentials, userCredentials, strlen(userCredentials)); //Note: Input array is consumed
  #endif
          }
          strncat(serverRequest, credentials, SERVER_BUFFER_SIZE);
          strncat(serverRequest, "\r\n", SERVER_BUFFER_SIZE);

          DEBUG_SERIAL.print(F("serverRequest size: "));
          DEBUG_SERIAL.print(strlen(serverRequest));
          DEBUG_SERIAL.print(F(" of "));
          DEBUG_SERIAL.print(sizeof(serverRequest));
          DEBUG_SERIAL.println(F(" bytes available"));

          DEBUG_SERIAL.println(F("Sending server request:"));
          DEBUG_SERIAL.println(serverRequest);
          ntripClient.write(serverRequest, strlen(serverRequest));

          //Wait for response
          unsigned long timeout = millis();
          while (ntripClient.available() == 0)
          {
            if (millis() - timeout > CONNECTION_TIMEOUT_MS)
            {
              ntripClient.stop(); // Too many requests with wrong settings will lead to bann, stop here
              Serial.println(F("Caster timed out!"));
            }
            // vTaskDelay(1000/portTICK_PERIOD_MS);
            delay(1000);
          }

          //Check reply
          bool connectionSuccess = false;
          char response[512];
          int responseSpot = 0;
          while (ntripClient.available())
          {
            if (responseSpot == sizeof(response) - 1) break;

            response[responseSpot++] = ntripClient.read();
            if (strstr(response, "200") > 0) //Look for 'ICY 200 OK'
              connectionSuccess = true;
            if (strstr(response, "401") > 0) //Look for '401 Unauthorized'
            {
              Serial.println(F("Hey - your credentials look bad! Check you caster username and password."));
              connectionSuccess = false;
            }
          }
          response[responseSpot] = '\0';

          DEBUG_SERIAL.print(F("Caster responded with: "));
          DEBUG_SERIAL.println(response);

          if (connectionSuccess == false)
          {
            DEBUG_SERIAL.print(F("Failed to connect to "));
            DEBUG_SERIAL.print(casterHost.c_str());
            DEBUG_SERIAL.print(F(": "));
            DEBUG_SERIAL.println(response);

            /** Yes, never use goto! But here: it just jumps to the beginning of the task, 
            * because return is forbiddden in tasks. This is the only use of goto in this code.
            */
            goto taskStart; // replaces the return command from the SparkFun example (a task must not return)
          }
          else
          {
            DEBUG_SERIAL.print(F("Connected to "));
            DEBUG_SERIAL.println(casterHost.c_str());
            lastReceivedRTCM_ms = millis(); //Reset timeout
          }
        } //End attempt to connect
      } //End connected == false

      if (ntripClient.connected() == true)
      {
        uint8_t rtcmData[512 * 4]; //Most incoming data is around 500 bytes but may be larger
        rtcmCount = 0;

        //Print any available RTCM data
        while (ntripClient.available())
        {
          //DEBUG_SERIAL.write(ntripClient.read()); //Pipe to serial port is fine but beware, it's a lot of binary data
          rtcmData[rtcmCount++] = ntripClient.read();
          if (rtcmCount == sizeof(rtcmData)) break;
        }

        if (rtcmCount > 0)
        {
          //Push RTCM to GNSS module over I2C
          myGNSS.pushRawData(rtcmData, rtcmCount, false);
          DEBUG_SERIAL.print(F("RTCM pushed to ZED: "));
          DEBUG_SERIAL.println(rtcmCount);
          uint32_t currentTime = millis();
          DEBUG_SERIAL.print(F("Last data before ms: "));
          DEBUG_SERIAL.println(currentTime - lastReceivedRTCM_ms);
          lastReceivedRTCM_ms = currentTime;
          // myGNSS.checkUblox();
          getPosition();
        // Measure stack size (last was 2304)
        // uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
        // DEBUG_SERIAL.print(F("task_get_rtk_corrections_over_wifi loop, uxHighWaterMark: "));
        // DEBUG_SERIAL.println(uxHighWaterMark);
        }
      }

      //Close socket if we don't have new data for 10s
      if (millis() - lastReceivedRTCM_ms > maxTimeBeforeHangup_ms)
      {
        DEBUG_SERIAL.println(F("RTCM timeout. Disconnecting..."));
        if (ntripClient.connected() == true)
          ntripClient.stop();

        // while (!checkConnectionToWifiStation()) {
        //   delay(1000);
        // }
        /** Yes, never use goto! But here: it just jumps to the beginning of the task, 
        * because return is forbiddden in tasks. This is the only use of goto in this code.
        */  
        goto taskStart; // replaces the return command from the SparkFun example (a task must not return)
      }


        /************************************************************************/
        // Measure stack size (last was 19320)
        // uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
        // DEBUG_SERIAL.print(F("task_get_rtk_corrections_over_wifi loop, uxHighWaterMark: "));
        // DEBUG_SERIAL.println(uxHighWaterMark);
        vTaskDelay(WIFI_TASK_INTERVAL_MS/portTICK_PERIOD_MS);

    }
    // Delete self task
    vTaskDelete(NULL);
}

/*******************************************************************************
 *                                 BLE
 * ****************************************************************************/
void setupBLE(void)
{
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
                                         BLECharacteristic::PROPERTY_NOTIFY 
                                         // We only use notify characteristic
                                       );

    pRealtimeKinematicsCharacteristic = pService->createCharacteristic(
                                         REALTIME_KINEMATICS_CHARACTERISTIC_UUID,
                                        //  BLECharacteristic::PROPERTY_READ   |
                                        //  BLECharacteristic::PROPERTY_WRITE  |
                                        //  BLECharacteristic::PROPERTY_INDICATE |
                                         BLECharacteristic::PROPERTY_NOTIFY 
                                         // We only use notify characteristic
                                       );
                                    
    pRTKAccuracyCharacteristic = pService->createCharacteristic(
                                RTK_ACCURACY_CHARACTERISTIC_UUID,
                                //  BLECharacteristic::PROPERTY_READ   |
                                //  BLECharacteristic::PROPERTY_WRITE  |
                                //  BLECharacteristic::PROPERTY_INDICATE |
                                BLECharacteristic::PROPERTY_NOTIFY 
                                // We only use notify characteristic
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
    DEBUG_SERIAL.println(F("Characteristic defined! Now you can read it in your phone!"));
}

void setupBNO080()
{   
    while (!bno080.begin()) {
        // Wait
        delay(1000);
        DEBUG_SERIAL.println(F("BNO080 not ready, waiting for I2C..."));
    }
    
    // Activate IMU functionalities
    
    bno080.enableRotationVector(BNO080_ROT_VECT_UPDATE_RATE_MS);   
    bno080.enableAccelerometer(BNO080_LIN_ACCEL_UPDATE_RATE_MS);    
    bno080.enableLinearAccelerometer(BNO080_LIN_ACCEL_UPDATE_RATE_MS);    
    // bno080.enableStepCounter(20);   // Funktioniert sehr schlecht.. 
    // --> timeBetweenReports should not be 20 ms ;)  try this: 31.25 Hz
    // bno080.enableStepCounter(32);
}

void xQueueSetup() {
  xQueueLatitude  = xQueueCreate( 2, sizeof( long ) );
  xQueueLongitude = xQueueCreate( 2, sizeof( long ) );
  xQueueAccuracy  = xQueueCreate( 2, sizeof( long ) );
}

void task_send_rtk_corrections_over_ble(void *pvParameters) {
  (void)pvParameters;

  String latLongStr((char *)0);
  String accuracyMmStr((char *)0);
  String accuracyStr((char *)0);
  // Latitude: 9, delimiter: 1, longitude: 9, delimiter: 1, accuracy: 3
  latLongStr.reserve(20);
  accuracyMmStr.reserve(5);
  accuracyStr.reserve(5);
  long latitude, longitude, accuracy;

  UBaseType_t uxHighWaterMark;
  // uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
  // DEBUG_SERIAL.print(F("task_send_rtk_corrections_over_ble setup, uxHighWaterMark: "));
  // DEBUG_SERIAL.println(uxHighWaterMark);

  while (true) {
    while (bleConnected) {
      if (xQueueReceive( xQueueLatitude, &latitude, portMAX_DELAY ) == pdPASS) {
        // DEBUG_SERIAL.print("Received latitude = ");
        // DEBUG_SERIAL.println(latitude);
      }
      if (xQueueReceive( xQueueLongitude, &longitude, portMAX_DELAY ) == pdPASS) {
        // DEBUG_SERIAL.print( "Received longitude = ");
        // DEBUG_SERIAL.print(longitude);
        // DEBUG_SERIAL.println(F(" (degrees * 10^-7)"));
      }
      if (xQueueReceive( xQueueAccuracy, &accuracy, portMAX_DELAY ) == pdPASS) {
        DEBUG_SERIAL.print( "Received accuracy = ");
        DEBUG_SERIAL.print(accuracy);
        DEBUG_SERIAL.println(F(" mm"));
        // Send position if accuracy is better than MIN_ACCEPTABLE_ACCURACY_MM
        if (accuracy < MIN_ACCEPTABLE_ACCURACY_MM) {
          accuracyStr = String(accuracy);
          latLongStr = String(latitude);
          latLongStr +=",";
          latLongStr += String(longitude);
          pRealtimeKinematicsCharacteristic->setValue(latLongStr.c_str());
          pRealtimeKinematicsCharacteristic->notify();
          pRTKAccuracyCharacteristic->setValue(accuracyStr.c_str());
          pRTKAccuracyCharacteristic->notify();
          DEBUG_SERIAL.print(F("BLE sent lat./long.: "));
          DEBUG_SERIAL.println(latLongStr);
        } else {
          DEBUG_SERIAL.print(F("No position data sent! Accuracy bad: "));
          DEBUG_SERIAL.print(accuracyStr);
        }
      }
      taskYIELD();
      // Measure stack size (last was 10300)
      // uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
      // DEBUG_SERIAL.print(F("task_send_rtk_corrections_over_ble loop, uxHighWaterMark: "));
      // DEBUG_SERIAL.println(uxHighWaterMark);

    } // while (bleConnected) ends 

    while (!bleConnected) {
      DEBUG_SERIAL.println(F("Waiting for BLE connection"));
      vTaskDelay(1000/portTICK_PERIOD_MS);
    }

    vTaskDelay(1000/portTICK_PERIOD_MS);
  } // while (true) ends
  
  // Delete self task
  vTaskDelete(NULL);
}

void task_send_bno080_data_over_ble(void *pvParameters) {
    (void)pvParameters;

    while (!bleConnected) {
        DEBUG_SERIAL.println(F("Waiting for BLE connection"));
        delay(1000);
        }
    
    float quatI, quatJ, quatK, quatReal, yawDegreeF, pitchDegreeF, linAccelZF;// rollDegreeF;
    int pitchDegree, yawDegree;// rollDegree;
    String dataStr((char *)0);
    // yaw: 3, delimiter: 1, pitch: 3, delimiter: 1, linAccelZF: 4 + LIN_ACCEL_Z_DECIMAL_DIGITS
    dataStr.reserve(12 + LIN_ACCEL_Z_DECIMAL_DIGITS);
    
    // Measure stack size
    UBaseType_t uxHighWaterMark;
    // uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
    // DEBUG_SERIAL.print(F("task_send_bno080_data_over_ble setup, uxHighWaterMark: "));
    // DEBUG_SERIAL.println(uxHighWaterMark);

    while (true) {
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
            // DEBUG_SERIAL.println(linAccelZF);      
        } else {
            DEBUG_SERIAL.println(F("Waiting for BNO080 dataAvailable"));
            vTaskDelay(1000/portTICK_PERIOD_MS);
            }
        // Measure stack size
        // uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
        // DEBUG_SERIAL.print(F("task_send_bno080_data_over_ble loop, uxHighWaterMark: "));
        // DEBUG_SERIAL.println(uxHighWaterMark);
        vTaskDelay(BLE_TASK_INTERVAL_MS/portTICK_PERIOD_MS);
    }
    // Delete self task
    vTaskDelete(NULL);
}

/*******************************************************************************
 *                                 Further system components
 * ****************************************************************************/
float getBatteryVolts() {
    // Vout = Dout * Vmax / Dmax 
    // Because battery volts are higher than Vmax, we use the voltage devider on 
    // Pin A13 (Huzzah ESP32, it may be different on other boards) 
    float batteryVolts = 2.0 * (analogRead(BAT_PIN) * 3.3 / 4095.0);
    return batteryVolts;
}

void buttonHandler(Button2 &btn) 
{
  if (btn == wipeButton) {
    digitalWrite(LED_BUILTIN, HIGH);
    DEBUG_SERIAL.println(F("Wiping WiFi credentials and RTK settings from memory..."));
    wipeSpiffsFiles();
    ESP.restart();
  }
}