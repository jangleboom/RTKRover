
#include "WiFiManager.h"

// Button to press to wipe out stored wifi credentials
#define WIFI_CRED_RESET_PIN 15 
#define DELAY_MS 1000
#define BAUD 115200




void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(WIFI_CRED_RESET_PIN, INPUT); //for resetting WiFi creds
  EEPROM.begin(400);
  Serial.begin(BAUD);
  if (!CheckWIFICreds()) {
    Serial.println("No WIFI credentials stored in memory. Loading form...");
    digitalWrite(LED_BUILTIN, HIGH);
    while (loadWIFICredsForm());
  }
}

void loop() {
  if(digitalRead(WIFI_CRED_RESET_PIN) == HIGH) {
    Serial.println("Wiping WiFi credentials from memory...");
    wipeEEPROM();
    while (loadWIFICredsForm());
  }
  digitalWrite(LED_BUILTIN,HIGH);
  delay(DELAY_MS);
  digitalWrite(LED_BUILTIN,LOW);
  delay(DELAY_MS);
}