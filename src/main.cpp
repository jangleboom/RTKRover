
/*******************************************************************************
 * @file main.cpp
 * @authors Markus Hädrich && Thomas Resch 
 * <br>
 * @brief This is part of a distributed software, here: head tracker and GNSS 
 *        positioning using Sparkfun Real Time Kinematics
 * <br>
 * @todo  - FREE RTOS
 *        - Calibration button (?)
 *        - Test: BNO080 found/connected
 *        - Test: BLE 
 *        
 * @note How to handle Wifi: 
 *        - Push the button 
 *        - Join the AP thats appearing 
 *            -# SSID: e. g. "HTRTK_" + ChipID 
 *            -# PW: e. g. "12345678"
 *        - Open address 192.168.4.1 in your browser and set credentials you are 
 *          using for you personal access point on your smartphone
 *        - If the process is done, the LED turns off and the device reboots
 *        - If there are no Wifi credentials stored in the EEPROM, the device 
 *          will jump in this mode on startup
 * 
 * @version 0.43
 ******************************************************************************/


#include <Arduino.h>
#include <Wire.h>
#include "SparkFun_BNO080_Arduino_Library.h"
#include "utility/imumaths.h"
#include <BLEDevice.h>
#include <BLE2902.h>
#include "sdkconfig.h"
#include "SparkFun_Ublox_Arduino_Library.h" //http://librarymanager/All#SparkFun_Ublox_GPS
#include "config.h"
#include "WiFiManager.h"


String deviceName = getDeviceName(DEVICE_TYPE);

SFE_UBLOX_GPS myGPS;
long lastTime = 0; //Simple local timer. Limits amount if I2C traffic to Ublox module.


/******************************************************************************/
//                                Bluetooth LE
/******************************************************************************/
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
String dataStr((char *)0);

void setupBLE(void);

/******************************************************************************/
//                                BNO080
/******************************************************************************/
bool sendYaw = true;
bool sendPitch = true;
bool sendRoll = false;
bool sendLinAccX = false;
bool sendLinAccY = false;
bool sendLinAccZ = false;

void setupBNO080(void);

BNO080 bno080;

/******************************************************************************/
//                                Button(s)
/******************************************************************************/
#include "Button2.h"
// Button to press to wipe out stored WiFi credentials
const int BUTTON_PIN = 15;
Button2 button = Button2(BUTTON_PIN, INPUT, false, false);

void buttonHandler(Button2 &btn);

/******************************************************************************/
//                                  Battery
/******************************************************************************/
// Messure half the battery voltage
const int BAT_PIN = 13; 
float getBatteryVolts(void);


void setup() {
    #ifdef DEBUGGING
    Serial.begin(BAUD);
    while (!Serial) {};
    #endif

    DEBUG_SERIAL.print(F("deviceName: "));
    DEBUG_SERIAL.println(deviceName);

    EEPROM.begin(400);
    // wipeEEPROM();
    if (!checkWiFiCreds()) {
        digitalWrite(LED_BUILTIN, HIGH);
        DEBUG_SERIAL.println(F("No WiFi credentials stored in memory. Loading form..."));
        while (loadWiFiCredsForm());
    }

    Wire.begin();
    Wire.setClock(400000); //Increase I2C data rate to 400kHz

    setupBLE(); 
    setupBNO080();
    button.setPressedHandler(buttonHandler); // INPUT_PULLUP is set too here  
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);
                
    DEBUG_SERIAL.print(F("Battery: "));
    DEBUG_SERIAL.print(getBatteryVolts());  // TODO: Buzzer peep tone while low power
    DEBUG_SERIAL.println(" V");

    // yaw: 3, delimiter: 1, pitch: 3, delimiter: 1, linAccelZF: 4 + LIN_ACCEL_Z_DECIMAL_DIGITS
    dataStr.reserve(12 + LIN_ACCEL_Z_DECIMAL_DIGITS);

    String thisBoard= ARDUINO_BOARD;
    DEBUG_SERIAL.print(F("Running on "));
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
    while (!bleConnected) {
        DEBUG_SERIAL.println(F("Waiting for BLE connection"));
        vTaskDelay(1000/portTICK_PERIOD_MS);
    }
    // TODO: Separate reading values from sending values
    if (bno080.dataAvailable())
        {         
            float quatI, quatJ, quatK, quatReal, yawDegreeF, pitchDegreeF, linAccelZF;// rollDegreeF;
            int pitchDegree, yawDegree;// rollDegree;
            
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

            linAccelZF = bno080.getLinAccelZ(); 

            
            dataStr = String(yawDegree) + DATA_STR_DELIMITER + String(pitchDegree) \
                    + DATA_STR_DELIMITER + String(linAccelZF, LIN_ACCEL_Z_DECIMAL_DIGITS);
            pCharacteristicTracking->setValue(dataStr.c_str());
            pCharacteristicTracking->notify();
            // DEBUG_SERIAL.print(dataStr);      
        } else {
            DEBUG_SERIAL.println(F("Waiting for BNO080 data"));
            vTaskDelay(1000/portTICK_PERIOD_MS);
        }
        vTaskDelay(10/portTICK_PERIOD_MS);
}


void setupBLE(void)
{
    BLEDevice::init(deviceName.c_str());
    BLEServer *pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks()); 
    BLEService *pService = pServer->createService(SERVICE_UUID);
  
    pCharacteristicTracking = pService->createCharacteristic(
                                         CHARACTERISTIC_UUID_NOTIFY,
                                        //  BLECharacteristic::PROPERTY_READ   |
                                        //  BLECharacteristic::PROPERTY_WRITE  |
                                        //  BLECharacteristic::PROPERTY_INDICATE |
                                         BLECharacteristic::PROPERTY_NOTIFY 
                                         
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

void setupBNO080(void)
{   
    bno080.begin(); // Activate IMU functionalities
    //bno080.calibrateAll();
    bno080.enableRotationVector(BNO080_UPDATE_RATE_MS);       
    bno080.enableLinearAccelerometer(BNO080_UPDATE_RATE_MS);    
    // bno080.enableStepCounter(20);   // Funktioniert sehr schlecht..  
}

float getBatteryVolts() {
    float batteryVolts = 2.0 * analogRead(BAT_PIN) * 3.3 / 4095.0;
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