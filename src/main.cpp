
/**
 * @file main.cpp
 * @authors Markus Hädrich && Thomas Resch 
 * <br>
 * @brief This is part of a distributed software, here: head tracker and GNNS positioning
 * using Sparkfun Real Time Kinematics
 * <br>
 * @todo  - a lot
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


void buttonHandler(Button2 &btn) {
  if (btn == button) {
    digitalWrite(LED_BUILTIN, HIGH);
    DEBUG_SERIAL.println("button clicked");
    DEBUG_SERIAL.println("Wiping WiFi credentials from memory...");
    wipeEEPROM();
    while (loadWIFICredsForm()) {};
  }
}

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
  // if(digitalRead(BUTTON_PIN) == HIGH) {
  //   Serial.println("Wiping WiFi credentials from memory...");
  //   wipeEEPROM();
  //   while (loadWIFICredsForm());
  // }
  // digitalWrite(LED_BUILTIN,HIGH);
  // delay(DELAY_MS);
  // digitalWrite(LED_BUILTIN,LOW);
  // delay(DELAY_MS);
}