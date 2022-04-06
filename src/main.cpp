
/**
 * @file main.cpp
 * @authors Markus Hädrich && Thomas Resch 
 * <br>
 * @brief This is part of a distributed software, here: head tracker and GNNS 
 * positioning using Sparkfun Real Time Kinematics
 * <br>
 * @todo  - FREE RTOS or Zephyr
 *        
 * @note How to handle Wifi: Push the button, join the AP thats appearing 
 * SSID: "RWAHT_WiFi_Manager", PW: "12345678", open 192.168.4.1 in your browser 
 * and set credentials you are using for you personal access point on your 
 * smartphone. If the process is done, the LED turns off and the device reboots.
 * If there are no Wifi credentials stored in the EEPROM, the LED turns on and the 
 * device will jump in this mode by itself after startup.
 */


#include <Arduino.h>
#include <Wire.h>
#include "SparkFun_BNO080_Arduino_Library.h"
#include "utility/imumaths.h"
#include <BLEDevice.h>
#include <BLE2902.h>
// #include <WiFi.h>
// #include <WiFiUdp.h>
// #include <OSCMessage.h>
#include "sdkconfig.h"
//#include "SparkFun_Ublox_Arduino_Library.h" //http://librarymanager/All#SparkFun_Ublox_GPS

//SFE_UBLOX_GPS myGPS;

long lastTime = 0; //Simple local timer. Limits amount if I2C traffic to Ublox module.

#define BUF_LEN 20
#define PORTNUMBER 30009  // use last two digits for headtracker id
#define SERVICE_UUID          "713D0000-503E-4C75-BA94-3148F18D941E"
#define CHARACTERISTIC_UUID   "713D0002-503E-4C75-BA94-3148F18D941E"

// enum dataFormat 
// {
//     stringFormat, // current format
//     byteFormat,
//     oscFormat,
//     customFormat  
// };

// WiFiUDP udp;

const char ssid[] = "OSPW";
const char password[] = "12OSPW3456";
const char *udpAddress = "192.168.43.205";
IPAddress ipAdress(192,168,43,205);

BNO080 bno080;//test
byte byteBuffer[BUF_LEN];
char charBuffer[BUF_LEN] = {0x00};
String deviceName = "rwaht84";
const int LED_PIN = 4;  // Show BLE connection status
const int BAT_PIN = A13; // Messure half the battery voltage
float batVoltage = 0;  

bool withSerial = true;
bool withBle = true;
bool withWifi = false;
bool bleConnected = false;
bool wifiConnected = false;
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
    
// dataFormat currentDataFormat = oscFormat;
BLECharacteristic *pCharacteristicTracking;

// Deactivate brown out detection
#undef ESP_ERROR_CHECK
#define ESP_ERROR_CHECK(x)   do { esp_err_t rc = (x); if (rc != ESP_OK) { ESP_LOGE("err", "esp_err_t = %d", rc); assert(0 && #x);} } while(0);
//=============================================

class MyCharacteristicCallbacks: public BLECharacteristicCallbacks 
{
    void onWrite(BLECharacteristic *pCharacteristicTracking) 
    {   
        std::string value = pCharacteristicTracking->getValue(); // Here I get the commands from the App (client)

        if (value.length() > 0) 
        {
            Serial.println("*********");
            Serial.print("New value: ");
            for (int i = 0; i < value.length(); i++)
                Serial.print(value[i]);

            Serial.println();
            Serial.println("*********");
        }
     }

     void onConnect(BLEServer* pServer) 
     {
        bleConnected = true;
     };

    void onDisconnect(BLEServer* pServer) 
    {
        bleConnected = false;
    }
};

class MyServerCallbacks: public BLEServerCallbacks 
{
    void onConnect(BLEServer* pServer) 
    {
        bleConnected = true;
    };

    void onDisconnect(BLEServer* pServer) 
    {
        bleConnected = false;
    }
};


void setupBLE(void)
{
    BLEDevice::init(deviceName.c_str());
    BLEServer *pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks()); 
    BLEService *pService = pServer->createService(SERVICE_UUID);
  
    pCharacteristicTracking = pService->createCharacteristic(
                                         CHARACTERISTIC_UUID,
                                         BLECharacteristic::PROPERTY_READ   |
                                         BLECharacteristic::PROPERTY_WRITE  |
                                         BLECharacteristic::PROPERTY_NOTIFY |
                                         BLECharacteristic::PROPERTY_INDICATE
                                       );

    pCharacteristicTracking->addDescriptor(new BLE2902());
    pCharacteristicTracking->setCallbacks(new MyCharacteristicCallbacks());
    pCharacteristicTracking->setValue(deviceName.c_str());
    pService->start();
    BLEAdvertising *pAdvertising = pServer->getAdvertising();
    pAdvertising->start();  
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
 


// void sendOscInt(IPAddress& address, char *adress, int value) 
// {
//     OSCMessage msg(adress);
//     msg.add((int)value);
//     udp.beginPacket(address, PORTNUMBER); 
//     msg.send(udp);
//     udp.endPacket();
//     msg.empty();
// }

// void sendOscFloat(IPAddress& address, char *adress, float value) 
// {
//     OSCMessage msg(adress);
//     msg.add((float)value);
//     udp.beginPacket(address, PORTNUMBER); 
//     msg.send(udp);
//     udp.endPacket();
//     msg.empty();
// }
    






#include "WiFiManager.h"
// #ifdef DEBUGGING
// #include "tests.h"
// #endif
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
  Serial.print("Battery: ");
  Serial.print(batVoltage);  // TODO: Buzzer peep tone while low power 
}

void loop() {
  // #ifdef DEBUGGING
  // aunit::TestRunner::run();
  // #endif

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

        unsigned int steps = 0;
 
        if(withBle && bleConnected) // BLE IS RWA STRING FORMAT FOR NOW
        {          
            String yaw_degrees_str, pitch_degrees_str, lin_accel_z_str, step_str, dataStr;
            yaw_degrees_str = String(yaw_degrees);
            pitch_degrees_str = String(pitch_degrees);
            lin_accel_z_str = String(lin_accel_z_f);
            step_str = String(steps);
            dataStr = yaw_degrees_str + " " + pitch_degrees_str + " " + lin_accel_z_str + " " + step_str;
            pCharacteristicTracking->setValue(dataStr.c_str());
            pCharacteristicTracking->notify();
            Serial.println(dataStr);         
        }
        
        // if(withWifi && wifiConnected) // WIFE SENDS SINGLE VALUE OSC MESSAGES
        // {
        //     if(abs(lastYaw_degrees - yaw_degrees) >= 2)
        //     {             
        //         sendOscInt(ipAdress, "/azi", yaw_degrees); 
        //         lastYaw_degrees = yaw_degrees;  
        //     }

        //     if(abs(lastPitch_degrees - pitch_degrees) >= 2)
        //     {             
        //         sendOscInt(ipAdress, "/ele", pitch_degrees); 
        //         lastPitch_degrees = pitch_degrees;  
        //     }  

        //     if(abs(lastRoll_degrees - roll_degrees) >= 2)
        //     {             
        //         sendOscInt(ipAdress, "/roll", roll_degrees); 
        //         lastRoll_degrees = roll_degrees;  
        //     } 
        // }      
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