#ifndef CASTER_SECRETS_H
#define CASTER_SECRETS_H
// A place for your caster credentials

// RTK2Go http://www.rtk2go.com:2101/SNIP::STATUS#uptime

const char kCasterHost[] = "rtk2go.com"; 
const char kCasterPort[] = "2101";
const char kMountPoint[] = "soundwalk";
const char kCasterUser[] = "fhnwbasel@gmail.com";   // User must provide their own email address to use RTK2Go
const char kCasterUserPw[] = "";                    // This can be added to the web form if you want to use a PW here, it's not neccecary

// Device name 
const char kDeviceName[] = "roverli";           
// Wifi access
const char kWifiSsid[] = "YOUR_SSID_WITHOUT_SPACES"; // Wifi to connect the rover with
const char kWifiPw[] = "YOUR_WIFI_PASSWORD";

#endif /*** CASTER_SECRETS_H ***/