#include <RTKRoverManager.h>


/********************************************************************************
*                             WiFi
* ******************************************************************************/
#pragma region: WIFI

void RTKRoverManager::setupStationMode(const char* ssid, const char* password, const char* deviceName) {
  WiFi.mode(WIFI_STA);
  WiFi.begin( ssid, password);
  if (WiFi.waitForConnectResult() != WL_CONNECTED) {
    // TODO:  - count reboots and stop after 3 times (save in SPIFFS)
    //        - display state
    Serial.println("WiFi failed! Reboot in 10 s!");
    delay(10000);
    ESP.restart();
  }
  Serial.println();

  if (!MDNS.begin(deviceName)) {
      Serial.println("Error starting mDNS, use local IP instead!");
  } else {
    Serial.printf("Starting mDNS, find me under <http://www.%s.local>\n", DEVICE_NAME);
  }

  Serial.print("Wifi client started: ");
  Serial.println(WiFi.getHostname());
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
}

void RTKRoverManager::setupAPMode(const char* apSsid, const char* apPassword) {
    Serial.print("Setting soft-AP ... ");
    WiFi.mode(WIFI_AP);
    Serial.println(WiFi.softAP(apSsid, apPassword) ? "Ready" : "Failed!");
    Serial.print("Access point started: ");
    Serial.println(AP_SSID);
    Serial.print("IP address: ");
    Serial.println(WiFi.softAPIP());
}

bool RTKRoverManager::savedNetworkAvailable(const String& ssid) {
  if (ssid.isEmpty()) return false;

  uint8_t nNetworks = (uint8_t) WiFi.scanNetworks();
  Serial.print(nNetworks);  Serial.println(F(" networks found."));
    for (uint8_t i=0; i<nNetworks; i++) {
    if (ssid.equals(String(WiFi.SSID(i)))) {
      Serial.print(F("A known network with SSID found: ")); 
      Serial.print(WiFi.SSID(i));
      Serial.print(F(" (")); 
      Serial.print(WiFi.RSSI(i)); 
      Serial.println(F(" dB), connecting..."));
      return true;
    }
  }
  return false;
}

#pragma endregion

/********************************************************************************
*                             Web server
* ******************************************************************************/

void RTKRoverManager::startServer(AsyncWebServer *server) {
  server->on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send_P(200, "text/html", INDEX_HTML, processor);
  });

  server->on("/actionUpdateData", HTTP_POST, actionUpdateData);
  server->on("/actionWipeData", HTTP_POST, actionWipeData);
  server->on("/actionRebootESP32", HTTP_POST, actionRebootESP32);

  server->onNotFound(notFound);
  server->begin();
}
  
void RTKRoverManager::notFound(AsyncWebServerRequest *request) {
  request->send(404, "text/plain", "Not found");
}

void RTKRoverManager::actionRebootESP32(AsyncWebServerRequest *request) {
  Serial.println("ACTION actionRebootESP32!");
  request->send_P(200, "text/html", REBOOT_HTML, RTKRoverManager::processor);
  delay(3000);
  ESP.restart();
}

void RTKRoverManager::actionWipeData(AsyncWebServerRequest *request) {
  Serial.println("ACTION actionWipeData!");
  int params = request->params();
  Serial.printf("params: %d\n", params);
  for (int i = 0; i < params; i++) {
    AsyncWebParameter* p = request->getParam(i);
    Serial.printf("%d. POST[%s]: %s\n", i+1, p->name().c_str(), p->value().c_str());
    if (strcmp(p->name().c_str(), "wipe_button") == 0) {
      if (p->value().length() > 0) {
        Serial.printf("wipe command received: %s",p->value().c_str());
        wipeSpiffsFiles();
      } 
     }
    } 

  Serial.print(F("Data in SPIFFS was wiped out!"));
  request->send_P(200, "text/html", INDEX_HTML, processor);
}

void RTKRoverManager::actionUpdateData(AsyncWebServerRequest *request) {
  Serial.println("ACTION actionUpdateData!");

  int params = request->params();
  for (int i = 0; i < params; i++) {
    AsyncWebParameter* p = request->getParam(i);
    Serial.printf("%d. POST[%s]: %s\n", i+1, p->name().c_str(), p->value().c_str());

    if (strcmp(p->name().c_str(), PARAM_WIFI_SSID) == 0) {
      if (p->value().length() > 0) {
        writeFile(SPIFFS, PATH_WIFI_SSID, p->value().c_str());
     } 
    }

    if (strcmp(p->name().c_str(), PARAM_WIFI_PASSWORD) == 0) {
      if (p->value().length() > 0) {
        writeFile(SPIFFS, PATH_WIFI_PASSWORD, p->value().c_str());
     } 
    }

    if (strcmp(p->name().c_str(), PARAM_RTK_CASTER_HOST) == 0) {
      if (p->value().length() > 0) {
        writeFile(SPIFFS, PATH_RTK_CASTER_HOST, p->value().c_str());
     } 
    }

    if (strcmp(p->name().c_str(), PARAM_RTK_CASTER_USER) == 0) {
      if (p->value().length() > 0) {
        writeFile(SPIFFS, PATH_RTK_CASTER_USER, p->value().c_str());
     } 
    }

    if (strcmp(p->name().c_str(), PARAM_RTK_MOINT_POINT) == 0) {
      if (p->value().length() > 0) {
        writeFile(SPIFFS, PATH_RTK_MOINT_POINT, p->value().c_str());
     } 
    }


  }
  Serial.println(F("Data saved to SPIFFS!"));
  request->send_P(200, "text/html", INDEX_HTML, RTKRoverManager::processor);
}

// Replaces placeholder with stored values
String RTKRoverManager::processor(const String& var) {
  if (var == PARAM_WIFI_SSID) {
    String savedSSID = readFile(SPIFFS, PATH_WIFI_SSID);
    return (savedSSID.isEmpty() ? String(PARAM_WIFI_SSID) : savedSSID);
  }
  else if (var == PARAM_WIFI_PASSWORD) {
    String savedPassword = readFile(SPIFFS, PATH_WIFI_PASSWORD);
    return (savedPassword.isEmpty() ? String(PARAM_WIFI_PASSWORD) : "*******");
  }

  else if (var == PARAM_RTK_CASTER_HOST) {
    String savedCaster = readFile(SPIFFS, PATH_RTK_CASTER_HOST);
    return (savedCaster.isEmpty() ? String(PARAM_RTK_CASTER_HOST) : savedCaster);
  }

  else if (var == PARAM_RTK_CASTER_USER) {
    String savedCaster = readFile(SPIFFS, PATH_RTK_CASTER_USER);
    return (savedCaster.isEmpty() ? String(PARAM_RTK_CASTER_USER) : savedCaster);
  }

  else if (var == PARAM_RTK_MOINT_POINT) {
    String savedCaster = readFile(SPIFFS, PATH_RTK_MOINT_POINT);
    return (savedCaster.isEmpty() ? String(PARAM_RTK_MOINT_POINT) : savedCaster);
  }
 
  else if (var == "next_addr") {
    String savedSSID = readFile(SPIFFS, PATH_WIFI_SSID);
    String savedPW = readFile(SPIFFS, PATH_WIFI_PASSWORD);
    if (savedSSID.isEmpty() || savedPW.isEmpty()) {
      return String(IP_AP);
    } else {
      String clientAddr = String(DEVICE_NAME);
      clientAddr += ".local";
      return clientAddr;
    }
  }
  else if (var == "next_ssid") {
    String savedSSID = readFile(SPIFFS, PATH_WIFI_SSID);
    return (savedSSID.isEmpty() ? String(AP_SSID) : savedSSID);
  }
  return String();
}

/********************************************************************************
*                             SPIFFS
* ******************************************************************************/

bool RTKRoverManager::setupSPIFFS(bool format) {
  bool success = true;

  #ifdef ESP32
    if (!SPIFFS.begin(true)) {
      DEBUG_SERIAL.println("An Error has occurred while mounting SPIFFS");
      success = false;
      return success;
    }
  #else
    if (!SPIFFS.begin()) {
      DEBUG_SERIAL.println("An Error has occurred while mounting SPIFFS");
      success = false;
      return success;
    }
  #endif
  
  if (format) {
    DEBUG_SERIAL.println(F("formatting SPIFFS, ..."));
    success &= SPIFFS.format();
  }

  return success;
}

String RTKRoverManager::readFile(fs::FS &fs, const char* path) {
  Serial.printf("Reading file: %s\r\n", path);
  File file = fs.open(path, "r");

  if (!file || file.isDirectory()) {
    Serial.println("- empty file or failed to open file");
    return String();
  }
  Serial.println("- read from file:");
  String fileContent;

  while (file.available()) {
    fileContent += String((char)file.read());
  }
  file.close();
  Serial.println(fileContent);

  return fileContent;
}

void RTKRoverManager::writeFile(fs::FS &fs, const char* path, const char* message) {
  Serial.printf("Writing file: %s\r\n", path);

  File file = fs.open(path, "w");
  if (!file) {
    Serial.println("- failed to open file for writing");
    return;
  }
  if (file.print(message)) {
    Serial.println("- file written");
  } else {
    Serial.println("- write failed");
  }
  file.close();
}
void RTKRoverManager::listFiles() {
  File root = SPIFFS.open("/");
  File file = root.openNextFile();
 
  while (file) {
      Serial.print("FILE: ");
      Serial.println(file.name());
      file = root.openNextFile();
  }
  file.close();
  root.close();
}

void RTKRoverManager::wipeSpiffsFiles() {
  File root = SPIFFS.open("/");
  File file = root.openNextFile();

  Serial.println(F("Wiping: "));

  while (file) {
    Serial.print("FILE: ");
    Serial.println(file.path());
    SPIFFS.remove(file.path());
    file = root.openNextFile();
  }
}