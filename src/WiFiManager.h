#ifndef WIFI_MANAGER_H
#define WIFI_MANAGER_H

#include <Arduino.h>
#include "EEPROM.h"
#include "HTML.h"
#include <WebServer.h>

#define SSID_ADDR 100
#define KEY_ADDR 200
#define END_ADDR 400

/**
 * @brief Function to handle unknown URLs
 */
void handleNotFound();

/**
 * @brief Function for writing WiFi creds to EEPROM
 * @return true if save successful, false if unsuccessful
 */
bool writeToMemory(String ssid, String pass);


/**
 * @brief Function for handling form
 */
void handleSubmit();

/**
 * @brief Function for home page
 */
void handleRoot();

/**
 * @brief Function for loading form
 * @return false if no WiFi creds in EEPROM
 */
bool loadWIFICredsForm();

/**
 * @brief Function checking WiFi creds in memory 
 * @return: true if not empty, false if empty
 */
bool CheckWIFICreds();

/**
 * @brief Wipes out the stored WIFI credentials from EEPROM
 * 
 */
void wipeEEPROM();

#endif /* WIFI_MANAGER_H */ 