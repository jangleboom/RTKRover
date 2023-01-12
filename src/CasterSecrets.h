#ifndef CASTER_SECRETS_H
#define CASTER_SECRETS_H
// A place for your caster credentials

// RTK2Go http://www.rtk2go.com:2101/SNIP::STATUS#uptime

const char kCasterHost[] = "rtk2go.com"; 
const char kCasterPort[] = "2101";
const char kMountPoint[] = "soundwalk";
const char kCasterUser[] = "fhnwbasel@gmail.com";   // User must provide their own email address to use RTK2Go
const char kCasterUserPw[] = "";                    // This can be added to the web form if you want to use a PW here, it's not neccecary

// Device name (this rover with headtracking)
const char kDeviceName[] = "rtkrover123";
// Wifi access
const char kWifiSsid[] = "ssid_without_spaces";
const char kWifiPw[] = "***";

#endif /*** CASTER_SECRETS_H ***/