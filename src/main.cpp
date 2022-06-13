
/*******************************************************************************
 * @file main.cpp
 * @authors Markus Hädrich && Thomas Resch 
 * <br>
 * @brief This is part of a distributed software, here: head tracker and GNSS 
 *        positioning using Sparkfun Real Time Kinematics
 * <br>
 * @todo  - Task to check / reconnect WiFi (independent of head tracking)
 *        - Calibration button (?)
 *        - Test: BNO080 found/connected
 *        - Test: BLE 
 *        - Status led for WiFi/BLE? on the device box or monitoring in app only?
 *        
 * @note How to handle WiFi: 
 *        - Push the button 
 *        - Join the AP thats appearing 
 *            -# SSID: e. g. "RTKRover_" + ChipID 
 *            -# PW: e. g. "12345678"
 *        - Open address 192.168.4.1 in your browser and set credentials you are 
 *          using for you personal access point on your smartphone
 *        - If the process is done, the LED turns off and the device reboots
 *        - If there are no Wifi credentials stored in the EEPROM, the device 
 *          will jump in this mode on startup
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
#include <config.h>
#include <secrets.h>
#include <WiFiManager.h>


String deviceName = getDeviceName(DEVICE_TYPE);

/*******************************************************************************
 *                                 Button(s)
 * ****************************************************************************/
#include "Button2.h"
// Button to press to wipe out stored WiFi credentials
const int BUTTON_PIN = 15;
Button2 button = Button2(BUTTON_PIN, INPUT, false, false);

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

void connectToWiFiAP(const String& ssid, const String& key);

/*******************************************************************************
 *                                 Bluetooth LE
 * ****************************************************************************/
float bleConnected = false; // TODO: deglobalize this

class MyCharacteristicCallbacks: public BLECharacteristicCallbacks 
{
    void onWrite(BLECharacteristic *pCharacteristicTracking) 
    {   
        std::string value = pCharacteristicTracking->getValue(); // Here I get the commands from the App (client)

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

BLECharacteristic *pCharacteristicTracking;
BLECharacteristic *pCharacteristicPositioning;
void setupBLE(void);

/*******************************************************************************
 *                                 BNO080
 * ****************************************************************************/
bool sendYaw = true;
bool sendPitch = true;
bool sendRoll = false;
bool sendLinAccX = false;
bool sendLinAccY = false;
bool sendLinAccZ = false;

BNO080 bno080;

void setupBNO080(void);
void task_send_bno080_ble(void *pvParameters);

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

long lastReceivedRTCM_ms = 0; //5 RTCM messages take approximately ~300ms to arrive at 115200bps
int maxTimeBeforeHangup_ms = 10000; //If we fail to get a complete RTCM frame after 10s, then disconnect from caster

SFE_UBLOX_GNSS myGNSS;

void setupGNSS(void);
void beginClient(void);
void getPosition(void);

/**
 * FreeRTOS
 */
xQueueHandle xQueueLatitude, xQueueLongitude, xQueueAccuracy;
// void printGNSSData(void);
void task_get_rtcm_wifi(void *pvParameters);
void task_send_rtk_ble(void *pvParameters);
void xQueueSetup(void);

void setup() {
    #ifdef DEBUGGING
    Serial.begin(BAUD);
    while (!Serial) {};
    #endif
    Wire.begin();
    setupBLE();
    setupBNO080();

    DEBUG_SERIAL.print(F("Device name: "));
    DEBUG_SERIAL.println(deviceName);
    DEBUG_SERIAL.print(F("Battery: "));
    DEBUG_SERIAL.print(getBatteryVolts());  // TODO: Buzzer peep tone while low power
    DEBUG_SERIAL.println(" V");

    button.setPressedHandler(buttonHandler); // INPUT_PULLUP is set too here  
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);
    
    // TODO: make the WiFi setup a primary task
    EEPROM.begin(400);
    // wipeEEPROM();
    if (!checkWiFiCreds()) {
        digitalWrite(LED_BUILTIN, HIGH);
        DEBUG_SERIAL.println(F("No WiFi credentials stored in memory. Loading form..."));
        while (loadWiFiCredsForm());
    }  else {
    // Then log into WiFi
    String ssid = EEPROM.readString(SSID_ADDR);
    String key = EEPROM.readString(KEY_ADDR);
    connectToWiFiAP(ssid, key);
    }
    
    xQueueSetup();
    xTaskCreatePinnedToCore( &task_get_rtcm_wifi, "task_get_rtcm_wifi", 1024 * 7, NULL, GNSS_OVER_WIFI_PRIORITY, NULL, RUNNING_CORE_0);
    xTaskCreatePinnedToCore( &task_send_bno080_ble, "task_send_bno080_ble", 1024 * 11, NULL, BNO080_OVER_BLE_PRIORITY, NULL, RUNNING_CORE_1);
    xTaskCreatePinnedToCore( &task_send_rtk_ble, "task_send_rtk_ble", 1024 * 11, NULL, BNO080_OVER_BLE_PRIORITY, NULL, RUNNING_CORE_1);
    
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

    button.loop();
}

/*******************************************************************************
 *                                 GNSS
 * ****************************************************************************/

void setupGNSS() {
    while (myGNSS.begin(Wire1, RTK_I2C_ADDR) == false) {
        DEBUG_SERIAL.println(F("u-blox GNSS not detected at default I2C address. Please check wiring. Freezing loop."));
        delay(1000);
      }
    
    myGNSS.setI2COutput(COM_TYPE_UBX); //Turn off NMEA noise
    myGNSS.setPortInput(COM_PORT_I2C, COM_TYPE_UBX | COM_TYPE_NMEA | COM_TYPE_RTCM3); //Be sure RTCM3 input is enabled. UBX + RTCM3 is not a valid state.
    myGNSS.setNavigationFrequency(1); //Set output in Hz.
    #ifdef DEBUGGING
    myGNSS.setNMEAOutputPort(Serial);
    #endif
}

void getPosition() {
  static long lastRun = millis();
  if (millis() - lastRun > 1000) {
    lastTime = millis(); //Update the timer

    long latitude = myGNSS.getLatitude();
    xQueueSend( xQueueLatitude, &latitude, portMAX_DELAY );
    // DEBUG_SERIAL.print(F("Lat: "));
    // DEBUG_SERIAL.print(latitude);

    long longitude = myGNSS.getLongitude();
    xQueueSend( xQueueLongitude, &longitude, portMAX_DELAY );
    // DEBUG_SERIAL.print(F(" Long: "));
    // DEBUG_SERIAL.print(longitude);
    // DEBUG_SERIAL.println(F(" (degrees * 10^-7)"));

    // long altitude = myGNSS.getAltitude();
    // DEBUG_SERIAL.print(F("Alt: "));
    // DEBUG_SERIAL.print(altitude);
    // DEBUG_SERIAL.println(F(" (mm)"));

    long accuracy = myGNSS.getPositionAccuracy();
    xQueueSend( xQueueAccuracy, &accuracy, portMAX_DELAY );
    // DEBUG_SERIAL.print(F("3D Positional Accuracy: "));
    // DEBUG_SERIAL.print(accuracy);
    // DEBUG_SERIAL.println(F(" (mm)"));
  }
}

/*******************************************************************************
 *                                 WiFi
 * ****************************************************************************/

void connectToWiFiAP(const String& ssid, const String& key) {
  delay(10);
  // We start by connecting to a WiFi network
  DEBUG_SERIAL.print(F("\nConnecting to "));
  DEBUG_SERIAL.println(ssid);
  WiFi.mode(WIFI_STA);
  WiFi.softAPdisconnect(true);
  WiFi.begin(ssid.c_str(), key.c_str());
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    DEBUG_SERIAL.print(".");
  }
  DEBUG_SERIAL.println("");
  DEBUG_SERIAL.println("WiFi connected");
  DEBUG_SERIAL.println("IP address: ");
  DEBUG_SERIAL.println(WiFi.localIP());
}

void task_get_rtcm_wifi(void *pvParameters) {
    (void)pvParameters;

    while (!Wire1.begin(RTK_SDA_PIN, RTK_SCL_PIN)) {
        DEBUG_SERIAL.println(F("I2C for RTK not running, check cable..."));
        vTaskDelay(1000/portTICK_PERIOD_MS);
    }
    Wire1.setClock(I2C_FREQUENCY_100K);
    setupGNSS();
    // Measure stack size
    UBaseType_t uxHighWaterMark; 
    // uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
    // DEBUG_SERIAL.print(F("task_get_rtcm_wifi setup, uxHighWaterMark: "));
    // DEBUG_SERIAL.println(uxHighWaterMark);
    WiFiClient ntripClient;
    long rtcmCount = 0;

    while (true) {

      if (ntripClient.connected() == false)
      {
        Serial.print(F("Opening socket to "));
        Serial.println(casterHost);

        if (ntripClient.connect(casterHost, casterPort) == false) //Attempt connection
        {
          Serial.println(F("Connection to caster failed"));
          // return;
        }
        else
        {
          Serial.print(F("Connected to "));
          Serial.print(casterHost);
          Serial.print(F(": "));
          Serial.println(casterPort);

          Serial.print(F("Requesting NTRIP Data from mount point "));
          Serial.println(mountPoint);

          const int SERVER_BUFFER_SIZE  = 512;
          char serverRequest[SERVER_BUFFER_SIZE];

          snprintf(serverRequest, SERVER_BUFFER_SIZE, "GET /%s HTTP/1.0\r\nUser-Agent: NTRIP SparkFun u-blox Client v1.0\r\n",
                  mountPoint);

          char credentials[512];
          if (strlen(casterUser) == 0)
          {
            strncpy(credentials, "Accept: */*\r\nConnection: close\r\n", sizeof(credentials));
          }
          else
          {
            //Pass base64 encoded user:pw
            char userCredentials[sizeof(casterUser) + sizeof(casterUserPW) + 1]; //The ':' takes up a spot
            snprintf(userCredentials, sizeof(userCredentials), "%s:%s", casterUser, casterUserPW);

            Serial.print(F("Sending credentials: "));
            Serial.println(userCredentials);

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

          Serial.print(F("serverRequest size: "));
          Serial.print(strlen(serverRequest));
          Serial.print(F(" of "));
          Serial.print(sizeof(serverRequest));
          Serial.println(F(" bytes available"));

          Serial.println(F("Sending server request:"));
          Serial.println(serverRequest);
          ntripClient.write(serverRequest, strlen(serverRequest));

          //Wait for response
          unsigned long timeout = millis();
          while (ntripClient.available() == 0)
          {
            if (millis() - timeout > CONNECTION_TIMEOUT_MS)
            {
              Serial.println(F("Caster timed out!"));
              ntripClient.stop();
            
            }
            delay(10);
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

          Serial.print(F("Caster responded with: "));
          Serial.println(response);

          if (connectionSuccess == false)
          {
            Serial.print(F("Failed to connect to "));
            Serial.print(casterHost);
            Serial.print(F(": "));
            Serial.println(response);
            // return;
          }
          else
          {
            Serial.print(F("Connected to "));
            Serial.println(casterHost);
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
          //Serial.write(ntripClient.read()); //Pipe to serial port is fine but beware, it's a lot of binary data
          rtcmData[rtcmCount++] = ntripClient.read();
          if (rtcmCount == sizeof(rtcmData)) break;
        }

        if (rtcmCount > 0)
        {
          lastReceivedRTCM_ms = millis();

          //Push RTCM to GNSS module over I2C
          myGNSS.pushRawData(rtcmData, rtcmCount, false);
          Serial.print(F("RTCM pushed to ZED: "));
          Serial.println(rtcmCount);
          // myGNSS.checkUblox();
          getPosition();
                  // Measure stack size (last was 7420)
        // uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
        // DEBUG_SERIAL.print(F("task_get_rtcm_wifi loop, uxHighWaterMark: "));
        // DEBUG_SERIAL.println(uxHighWaterMark);
        }
      }

      //Close socket if we don't have new data for 10s
      if (millis() - lastReceivedRTCM_ms > maxTimeBeforeHangup_ms)
      {
        Serial.println(F("RTCM timeout. Disconnecting..."));
        if (ntripClient.connected() == true)
          ntripClient.stop();
        // return;
      }


        /************************************************************************/
        // Measure stack size (last was 19320)
        // uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
        // DEBUG_SERIAL.print(F("task_get_rtcm_wifi loop, uxHighWaterMark: "));
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
    pCharacteristicTracking = pService->createCharacteristic(
                                         CHARACTERISTIC_UUID_BNO080,
                                        //  BLECharacteristic::PROPERTY_READ   |
                                        //  BLECharacteristic::PROPERTY_WRITE  |
                                        //  BLECharacteristic::PROPERTY_INDICATE |
                                         BLECharacteristic::PROPERTY_NOTIFY 
                                         // We only use notify characteristic
                                       );

    pCharacteristicTracking->addDescriptor(new BLE2902());
    pCharacteristicTracking->setCallbacks(new MyCharacteristicCallbacks());
    pCharacteristicTracking->setValue(deviceName.c_str());

    pCharacteristicPositioning = pService->createCharacteristic(
                                         CHARACTERISTIC_UUID_RTK,
                                        //  BLECharacteristic::PROPERTY_READ   |
                                        //  BLECharacteristic::PROPERTY_WRITE  |
                                        //  BLECharacteristic::PROPERTY_INDICATE |
                                         BLECharacteristic::PROPERTY_NOTIFY 
                                         // We only use notify characteristic
                                       );

    pCharacteristicPositioning->addDescriptor(new BLE2902());
    // pCharacteristicPositioning->setCallbacks(new MyCharacteristicCallbacks());
    // pCharacteristicPositioning->setValue(deviceName.c_str());

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
    //bno080.calibrateAll();
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

void task_send_rtk_ble(void *pvParameters) {
  (void)pvParameters;
  while (!bleConnected) {
    DEBUG_SERIAL.println(F("Waiting for BLE connection"));
    delay(1000);
    }
  String latLongStr((char *)0);
  String accuracyMmStr((char *)0);
  // Latitude: 9, delimiter: 1, longitude: 9, delimiter: 1, accuracy: 3
  latLongStr.reserve(20);
  accuracyMmStr.reserve(5);
  long latitude, longitude, accuracy;

  UBaseType_t uxHighWaterMark;
  // uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
  // DEBUG_SERIAL.print(F("task_send_rtk_ble setup, uxHighWaterMark: "));
  // DEBUG_SERIAL.println(uxHighWaterMark);

  while (true) {
    while (bleConnected) {
      if (xQueueReceive( xQueueLatitude, &latitude, portMAX_DELAY ) == pdPASS) {
        DEBUG_SERIAL.print("Received latitude = ");
        DEBUG_SERIAL.println(latitude);
      }
      if (xQueueReceive( xQueueLongitude, &longitude, portMAX_DELAY ) == pdPASS) {
        DEBUG_SERIAL.print( "Received longitude = ");
        DEBUG_SERIAL.print(longitude);
        DEBUG_SERIAL.println(F(" (degrees * 10^-7)"));
      }
      if (xQueueReceive( xQueueAccuracy, &accuracy, portMAX_DELAY ) == pdPASS) {
        DEBUG_SERIAL.print( "Received accuracy = ");
        DEBUG_SERIAL.print(accuracy);
        DEBUG_SERIAL.println(F(" (mm)"));
        // Send position if accuracy is better than 10 cm
        if (accuracy < 100) {
          latLongStr = String(latitude);
          latLongStr +=",";
          latLongStr += String(longitude);
          pCharacteristicPositioning->setValue(latLongStr.c_str());
          pCharacteristicPositioning->notify();
          DEBUG_SERIAL.print(F("BLE Sent: "));
          DEBUG_SERIAL.println(latLongStr);
        }
      }
      taskYIELD();
      // Measure stack size (last was 10300)
      // uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
      // DEBUG_SERIAL.print(F("task_send_rtk_ble loop, uxHighWaterMark: "));
      // DEBUG_SERIAL.println(uxHighWaterMark);

    } // while (bleConnected) ends 
    vTaskDelay(1000/portTICK_PERIOD_MS);

  } // while (true) ends
  
  // Delete self task
  vTaskDelete(NULL);
}

void task_send_bno080_ble(void *pvParameters) {
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
    // DEBUG_SERIAL.print(F("task_send_bno080_ble setup, uxHighWaterMark: "));
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
            pCharacteristicTracking->setValue(dataStr.c_str());
            pCharacteristicTracking->notify();
            // DEBUG_SERIAL.println(linAccelZF);      
        } else {
            DEBUG_SERIAL.println(F("Waiting for BNO080 data"));
            vTaskDelay(1000/portTICK_PERIOD_MS);
            }
        // Measure stack size
        // uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
        // DEBUG_SERIAL.print(F("task_send_bno080_ble loop, uxHighWaterMark: "));
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
  if (btn == button) {
    digitalWrite(LED_BUILTIN, HIGH);
    DEBUG_SERIAL.println(F("Wiping WiFi credentials from memory..."));
    wipeEEPROM();
    while (loadWiFiCredsForm()) {};
  }
}