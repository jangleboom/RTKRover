
/*******************************************************************************
 * @file main.cpp
 * @authors Markus Hädrich && Thomas Resch 
 * <br>
 * @brief This is part of a distributed software, here: head tracker and GNSS 
 *        positioning using Sparkfun Real Time Kinematics
 * <br>
 * @todo  - FREE RTOS or Zephyr
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
#include "htrtk_config.h"


String deviceName = getDeviceName(DEVICE_TYPE);

SFE_UBLOX_GPS myGPS;

long lastTime = 0; //Simple local timer. Limits amount if I2C traffic to Ublox module.

BNO080 bno080;
byte byteBuffer[PAYLOAD_BUF_LEN];
char charBuffer[PAYLOAD_BUF_LEN] = {0x00};

const int BAT_PIN = A13; // Messure half the battery voltage
float batVoltage = 0.;  

bool bleConnected = false;
bool sendYaw = true;
bool sendPitch = true;
bool sendRoll = false;
bool sendLinAccX = false;
bool sendLinAccY = false;
bool sendLinAccZ = false;

int yaw_degrees = 0;
int lastYaw_degrees = -10;
int pitch_degrees = 0;
int lastPitch_degrees = -400;
int roll_degrees = 0;
int lastRoll_degrees = -10;
int lastSteps = -1;

/******************************************************************************/
//                                Bluetooth LE
/******************************************************************************/

class MyCharacteristicCallbacks: public BLECharacteristicCallbacks 
{
    void onWrite(BLECharacteristic *pCharacteristicTracking) 
    {   
        std::string value = pCharacteristicTracking->getValue(); // Here I get the commands from the App (client)

        if (value.length() > 0) 
        {
            DEBUG_SERIAL.println("*********");
            DEBUG_SERIAL.print("New value: ");
            for (int i = 0; i < value.length(); i++)
                DEBUG_SERIAL.print(value[i]);

            DEBUG_SERIAL.println();
            DEBUG_SERIAL.println("*********");
        }
     }

     void onConnect(BLEServer* pServer) 
     {
        bleConnected = true;
        DEBUG_SERIAL.print("bleConnected: ");
        DEBUG_SERIAL.println(bleConnected);
     };

    void onDisconnect(BLEServer* pServer) 
    {
        bleConnected = false;
        DEBUG_SERIAL.print("bleConnected: ");
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

/******************************************************************************/
//                                BNO080
/******************************************************************************/
void setupBNO080(void);

#include "WiFiManager.h"
#ifdef DEBUGGING
#include "tests.h"
#endif

/******************************************************************************/
//                                Button(s)
/******************************************************************************/
#include "Button2.h"
// Button to press to wipe out stored wifi credentials
const int BUTTON_PIN = 15;
Button2 button = Button2(BUTTON_PIN, INPUT, false, false);

void buttonHandler(Button2 &btn);

void setup() {
  #ifdef DEBUGGING
  Serial.begin(BAUD);
  while (!Serial) {};
  #endif

  EEPROM.begin(400);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  button.setPressedHandler(buttonHandler); // INPUT_PULLUP is set too here
  DEBUG_SERIAL.print("deviceName: ");
  DEBUG_SERIAL.println(deviceName);

  if (!CheckWIFICreds()) {
    digitalWrite(LED_BUILTIN, HIGH);
    DEBUG_SERIAL.println(F("No WIFI credentials stored in memory. Loading form..."));
    while (loadWIFICredsForm());
  }

  Wire.begin();
  Wire.setClock(400000); //Increase I2C data rate to 400kHz

  setupBLE(); 
  setupBNO080();               
  
  batVoltage = 0.002 * analogRead(BAT_PIN);
  DEBUG_SERIAL.print("Battery: ");
  DEBUG_SERIAL.print(batVoltage);  // TODO: Buzzer peep tone while low power
  DEBUG_SERIAL.println(" V");
}

void loop() {
  #ifdef DEBUGGING
  aunit::TestRunner::run();
  #endif

  button.loop();

  if (bno080.dataAvailable() == true)       // TODO: Separate getting values from sending values
    {
        float quatI = bno080.getQuatI();
        float quatJ = bno080.getQuatJ();
        float quatK = bno080.getQuatK();
        float quatReal = bno080.getQuatReal();          
        float yaw_degrees_f, pitch_degrees_f, roll_degrees_f, lin_accel_z_f;           

        imu::Quaternion quat = imu::Quaternion(quatReal, quatI, quatJ, quatK);
        quat.normalize();
        imu::Vector<3> q_to_euler = quat.toEuler();     
        yaw_degrees_f = q_to_euler.x();
        yaw_degrees_f = yaw_degrees_f * -180.0 / M_PI; // conversion to degrees
        if( yaw_degrees_f < 0 ) 
            yaw_degrees_f += 360.0; // convert negative to positive angles
      
        yaw_degrees = (int)(round(yaw_degrees_f));  
         
        pitch_degrees_f = q_to_euler.z();
        pitch_degrees_f = pitch_degrees_f * -180.0/ M_PI;
        pitch_degrees = (int)(round(pitch_degrees_f));

        roll_degrees_f = q_to_euler.y();
        roll_degrees_f = roll_degrees_f * -180.0/ M_PI;
        roll_degrees = (int)(round(roll_degrees_f));

        lin_accel_z_f = 0.0;
        lin_accel_z_f = bno080.getLinAccelZ(); // float   

        // unsigned int steps = 0;
 
        String yaw_degrees_str, pitch_degrees_str, lin_accel_z_str, dataStr; //step_str, ;
        yaw_degrees_str = String(yaw_degrees);
        pitch_degrees_str = String(pitch_degrees);
        lin_accel_z_str = String(lin_accel_z_f);
        // step_str = String(steps);
        // TODO: dataStr St
        dataStr = yaw_degrees_str + " " + pitch_degrees_str + " " + lin_accel_z_str; //+ " " + step_str;
        pCharacteristicTracking->setValue(dataStr.c_str());
        pCharacteristicTracking->notify();
        // DEBUG_SERIAL.println(dataStr);           
    } 

    delay(10); // Maybee reduce delay time?
}


void buttonHandler(Button2 &btn) {
  if (btn == button) {
    digitalWrite(LED_BUILTIN, HIGH);
    DEBUG_SERIAL.println(F("Wiping WiFi credentials from memory..."));
    wipeEEPROM();
    while (loadWIFICredsForm()) {};
  }
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
    DEBUG_SERIAL.println("Characteristic defined! Now you can read it in your phone!");
}

void setupBNO080(void)
{
    bno080.begin(); // Activate IMU functionalities
    //bno080.calibrateAll();
    bno080.enableRotationVector(20);       
    bno080.enableLinearAccelerometer(20);    
   // bno080.enableStepCounter(20);   // Funktioniert sehr schlecht..  
    delay(1000);     // why? 
}