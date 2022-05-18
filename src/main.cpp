
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

void setupWiFi(const String& ssid, const String& key);

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
void task_bluetooth_connection(void *pvParameters);

/*******************************************************************************
 *                                 GNSS
 * ****************************************************************************/
#include <MicroNMEA.h> //http://librarymanager/All#MicroNMEA
char nmeaBuffer[100];
MicroNMEA nmea(nmeaBuffer, sizeof(nmeaBuffer));

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
// void printGNSSData(void);
void task_rtk_wifi_connection(void *pvParameters);

void setup() {
    #ifdef DEBUGGING
    Serial.begin(BAUD);
    while (!Serial) {};
    #endif

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
    setupWiFi(ssid, key);
    };

    
    // Wire1.begin(RTK_SDA_PIN, RTK_SCL_PIN, I2C_FREQUENCY);

    // Needed to execute before tasks 
    
    // TwoWire I2C_RTK = TwoWire(1);
    // while (!I2C_RTK.begin(RTK_SDA_PIN, RTK_SCL_PIN, I2C_FREQUENCY)) {
    //     DEBUG_SERIAL.print(F("No I2C for RTK, check cable on Pin: "));
    //     DEBUG_SERIAL.print(RTK_SDA_PIN);
    //     DEBUG_SERIAL.print(F(" and "));
    //     DEBUG_SERIAL.println(RTK_SCL_PIN);
    //     delay(500);
    // }
    //I2C_RTK.begin(RTK_SDA_PIN, RTK_SCL_PIN, 100000);
    // TwoWire I2C_BNO080 = TwoWire(0);
    // while (!I2C_BNO080.begin(BNO080_SDA_PIN, BNO080_SCL_PIN, I2C_FREQUENCY)) {
    //     DEBUG_SERIAL.print(F("No I2C for BNO080, check cable on Pin: "));
    //     DEBUG_SERIAL.print(BNO080_SDA_PIN);
    //     DEBUG_SERIAL.print(F(" and "));
    //     DEBUG_SERIAL.println(BNO080_SCL_PIN);
    //     delay(500);
    // }
    // I2C_BNO080.begin(BNO080_SDA_PIN, BNO080_SCL_PIN, I2C_FREQUENCY);
    
    xTaskCreatePinnedToCore( &task_rtk_wifi_connection, "task_rtk_wifi_connection", 20480, NULL, GNSS_OVER_WIFI_PRIORITY, NULL, RUNNING_CORE_0);
    xTaskCreatePinnedToCore( &task_bluetooth_connection, "task_bluetooth_connection", 10240, NULL, BNO080_OVER_BLE_PRIORITY, NULL, RUNNING_CORE_1);
    
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
    
    myGNSS.setI2COutput(COM_TYPE_UBX | COM_TYPE_NMEA); //Set the I2C port to output both NMEA and UBX messages
    myGNSS.saveConfigSelective(VAL_CFG_SUBSEC_IOPORT); //Save (only) the communications port settings to flash and BBR
    myGNSS.setProcessNMEAMask(SFE_UBLOX_FILTER_NMEA_ALL); // Make sure the library is passing all NMEA messages to processNMEA
    myGNSS.setProcessNMEAMask(SFE_UBLOX_FILTER_NMEA_GGA); // Or, we can be kind to MicroNMEA and _only_ pass the GGA messages to it
    // myGNSS.setI2COutput(COM_TYPE_UBX); //Turn off NMEA noise
    myGNSS.setPortInput(COM_PORT_I2C, COM_TYPE_UBX | COM_TYPE_NMEA | COM_TYPE_RTCM3); //Be sure RTCM3 input is enabled. UBX + RTCM3 is not a valid state.
    myGNSS.setNavigationFrequency(1); //Set output in Hz.
    #ifdef DEBUGGING
    myGNSS.setNMEAOutputPort(Serial);
    #endif
}

//This function gets called from the SparkFun u-blox Arduino Library
//As each NMEA character comes in you can specify what to do with it
//Useful for passing to other libraries like tinyGPS, MicroNMEA, or even
//a buffer, radio, etc.
void SFE_UBLOX_GNSS::processNMEA(char incoming)
{
  //Take the incoming char from the u-blox I2C port and pass it on to the MicroNMEA lib
  //for sentence cracking
  nmea.process(incoming);
}

//Connect to NTRIP Caster, receive RTCM, and push to ZED module over I2C
void beginClient() {
  WiFiClient ntripClient;
  long rtcmCount = 0;

  Serial.println(F("Subscribing to Caster. Press key to stop"));
  delay(10); //Wait for any serial to arrive
  while (Serial.available()) Serial.read(); //Flush

  while (Serial.available() == 0)
  {
    //Connect if we are not already. Limit to 5s between attempts.
    if (ntripClient.connected() == false)
    {
      Serial.print(F("Opening socket to "));
      Serial.println(casterHost);

      if (ntripClient.connect(casterHost, casterPort) == false) //Attempt connection
      {
        Serial.println(F("Connection to caster failed"));
        return;
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
          if (millis() - timeout > 5000)
          {
            Serial.println(F("Caster timed out!"));
            ntripClient.stop();
            return;
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
          return;
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
        myGNSS.checkUblox();

          if(nmea.isValid() == true)
  {
    long latitude_mdeg = nmea.getLatitude();
    long longitude_mdeg = nmea.getLongitude();

    Serial.print("Latitude (deg): ");
    Serial.println(latitude_mdeg / 1000000., 6);
    Serial.print("Longitude (deg): ");
    Serial.println(longitude_mdeg / 1000000., 6);

    nmea.clear(); // Clear the MicroNMEA storage to make sure we are getting fresh data
  }
  else
  {
    Serial.println("Waiting for fresh data");
  }
      }
    }

    //Close socket if we don't have new data for 10s
    if (millis() - lastReceivedRTCM_ms > maxTimeBeforeHangup_ms)
    {
      Serial.println(F("RTCM timeout. Disconnecting..."));
      if (ntripClient.connected() == true)
        ntripClient.stop();
      return;
    }

    delay(10);
  }

  Serial.println(F("User pressed a key"));
  Serial.println(F("Disconnecting..."));
  ntripClient.stop();
}


/*******************************************************************************
 *                                 WiFi
 * ****************************************************************************/

void setupWiFi(const String& ssid, const String& key) {
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

void task_rtk_wifi_connection(void *pvParameters) {
    (void)pvParameters;

    while (!Wire1.begin(RTK_SDA_PIN, RTK_SCL_PIN)) {
        DEBUG_SERIAL.println(F("I2C for RTK not running, check cable..."));
        vTaskDelay(1000/portTICK_PERIOD_MS);
    }
    Wire1.setClock(I2C_FREQUENCY_100K);
    setupGNSS();
    // Measure stack size
    // UBaseType_t uxHighWaterMark; 
    // uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
    // DEBUG_SERIAL.print(F("task_rtk_wifi_connection setup, uxHighWaterMark: "));
    // DEBUG_SERIAL.println(uxHighWaterMark);
    
    while (1) {
        beginClient();
        // Measure stack size (last was 19320)
        // uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
        // DEBUG_SERIAL.print(F("task_rtk_wifi_connection loop, uxHighWaterMark: "));
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

void task_bluetooth_connection(void *pvParameters) {
    (void)pvParameters;

    Wire.begin();
    setupBNO080();
    
    setupBLE();
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
    // UBaseType_t uxHighWaterMark;
    // uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
    // DEBUG_SERIAL.print(F("task_bluetooth_connection setup, uxHighWaterMark: "));
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
        // DEBUG_SERIAL.print(F("task_bluetooth_connection loop, uxHighWaterMark: "));
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