#ifndef CASTER_SECRETS_H
#define CASTER_SECRETS_H
// A place for your caster credentials, this will be not used if you use the web form of the RTKRoverManager

// RTK2Go http://www.rtk2go.com:2101/SNIP::STATUS#uptime

const char kCasterHost[] = "rtk2go.com"; 
const char kCasterPort[] = "2101";
const char kMountPoint[] = "YOUR_MOUNT_POINT";
const char kCasterUser[] = "YOUR_USER_EMAIL";           // User must provide their own email address to use RTK2Go
const char kCasterUserPw[] = "";                        // Not neccecary, more info: rtk2go.com

// Device name 
const char kDeviceName[] = "roverli";                   // E. g. 
// Wifi access
const char kWifiSsid[] = "YOUR_SSID_WITHOUT_SPACES";    // Wifi to connect the rover with
const char kWifiPw[] = "YOUR_WIFI_PASSWORD";

#endif /*** CASTER_SECRETS_H ***/
