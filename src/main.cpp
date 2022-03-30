#include "EEPROM.h"
#include "HTML.h"
#include <WebServer.h>

#define WIFI_CRED_RESET_PIN 15
#define DELAY_MS 1000
WebServer server(80);

// #include "WiFiManager.h"
void handleNotFound() {
  String message = "File Not Found\n\n";
  message += "URI: ";
  message += server.uri();
  message += "\nMethod: ";
  message += (server.method() == HTTP_GET) ? "GET" : "POST";
  message += "\nArguments: ";
  message += server.args();
  message += "\n";
  for (uint8_t i = 0; i < server.args(); i++) {
    message += " " + server.argName(i) + ": " + server.arg(i) + "\n";
  }
  server.send(404, "text/plain", message);
}

/*
 * Function for writing WiFi creds to EEPROM
 * Returns: true if save successful, false if unsuccessful
 */
bool writeToMemory(String ssid, String pass) {
  char buff1[30];
  char buff2[30];
  ssid.toCharArray(buff1,30);
  pass.toCharArray(buff2,30); 
  EEPROM.writeString(100,buff1);
  EEPROM.writeString(200,buff2);
  delay(100);
  String s = EEPROM.readString(100);
  String p = EEPROM.readString(200);
  //#if DEBUG
  Serial.println("Stored SSID, password, are: ");
  Serial.print("SSID: ");
  Serial.println(s);
  Serial.print("PW: ");
  Serial.println(p);
  //#endif
  if (ssid == s && pass == p) {
    return true;  
  } else {
    return false;
  }
}


/*
 * Function for handling form
 */
void handleSubmit() {
  String response_success="<h1>Success</h1>";
  response_success +="<h2>Device will restart in 3 seconds</h2>";

  String response_error="<h1>Error</h1>";
  response_error +="<h2><a href='/'>Go back</a>to try again";
  
  if (writeToMemory(String(server.arg("ssid")),String(server.arg("password")))) {
     server.send(200, "text/html", response_success);
     EEPROM.commit();
     delay(3000);
     ESP.restart();
  } else {
     server.send(200, "text/html", response_error);
  }
}

/*
 * Function for home page
 */
void handleRoot() {
  if (server.hasArg("ssid")&& server.hasArg("password")) {
    handleSubmit();
  }
  else {
    server.send(200, "text/html", INDEX_HTML);
  }
}

/*
 * Function for loading form
 * Returns: false if no WiFi creds in EEPROM
 */
bool loadWIFICredsForm() {
  String s = EEPROM.readString(100);
  String p = EEPROM.readString(200);
  
  const char* ssid     = "RWAHT_WiFi_Manager";
  const char* password = "12345678";

  Serial.println("Setting Access Point...");
  
  WiFi.softAP(ssid, password);
  
  IPAddress IP = WiFi.softAPIP();
  
  Serial.print("AP IP address: ");
  Serial.println(IP);
  
  server.on("/", handleRoot);

  server.onNotFound(handleNotFound);

  server.begin();
  
  Serial.println("HTTP server started");
 
  while (s.length() <= 0 || p.length() <= 0) {
    server.handleClient();
    delay(1000);
    Serial.println("…");
  }
  
  return false;
}

/*
 * Function checking WiFi creds in memory 
 * Returns: true if not empty, false if empty
 */
bool CheckWIFICreds() {
  Serial.println("Checking WIFI credentials");
  String s = EEPROM.readString(100);
  String p = EEPROM.readString(200);
  //#if DEBUG
  Serial.println("Found credentials in EEPROM!");
  Serial.print("SSID: ");
  Serial.println(s);
  Serial.print("PW: ");
  Serial.println(p);
  delay(5000);
  //#endif
  if (s.length() > 0 && p.length() > 0) {
    return true;
  } else {
    return false;
  }
}

void wipeEEPROM() {
  for(int i=0;i<400;i++){
    EEPROM.writeByte(i,0);
  }
  EEPROM.commit();
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(WIFI_CRED_RESET_PIN, INPUT); //for resetting WiFi creds
  EEPROM.begin(400);
  Serial.begin(115200);
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