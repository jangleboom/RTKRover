
/**
 * @file main.cpp
 * @authors Markus Hädrich && Thomas Resch 
 * <br>
 * @brief This is part of a distributed software, here: head tracker and GNNS 
 * positioning using Sparkfun Real Time Kinematics
 * <br>
 * @todo  - a lot
 * @note How to handle Wifi: Push the button, join the AP thats appearing 
 * SSID: "RWAHT_WiFi_Manager", PW: "12345678", open 192.168.4.1 in your browser 
 * and set credentials you are using for you personal access point on your 
 * smartphone. If the process is done, the LED turns off and the device reboots.
 * If there are no Wifi credentials stored in the EEPROM, the LED turns on and the 
 * device will jump in this mode by itself after startup.
 */

#include "WiFiManager.h"

const int DELAY_MS = 1000;

/******************************************************************************/
//                        Serial settings
/******************************************************************************/
//set to true for debug output, false for no debug output
#define DEBUGGING true 
#define DEBUG_SERIAL \
  if (DEBUGGING) Serial

const int BAUD = 115200;

/******************************************************************************/
//                                Button(s)
/******************************************************************************/
#include "Button2.h"
// Button to press to wipe out stored wifi credentials
const int BUTTON_PIN = 15;
Button2 button = Button2(BUTTON_PIN, INPUT, false, false);

void buttonHandler(Button2 &btn);


void setup() {
  Serial.begin(BAUD);
  while (!Serial) {};
  EEPROM.begin(400);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  button.setPressedHandler(buttonHandler); // INPUT_PULLUP is set too here

  if (!CheckWIFICreds()) {
    digitalWrite(LED_BUILTIN, HIGH);
    Serial.println("No WIFI credentials stored in memory. Loading form...");
    while (loadWIFICredsForm());
  }
}

void loop() {
  button.loop();
}


void buttonHandler(Button2 &btn) {
  if (btn == button) {
    digitalWrite(LED_BUILTIN, HIGH);
    DEBUG_SERIAL.println("button clicked");
    DEBUG_SERIAL.println("Wiping WiFi credentials from memory...");
    wipeEEPROM();
    while (loadWIFICredsForm()) {};
  }
}