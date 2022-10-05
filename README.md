<img align="right" src="./screenshots/rtkrover.jpeg" width="240"/> 

# RTKRover
## Headtracker + Real Time Kinematics (RTK Client)
### Hardware used:   
* Adafruit Feather ESP32 Huzzah 
* SparkFun GPS-RTK-SMA Breakout - ZED-F9P (Qwiic)
* SparkFun BNO080 Breakout
* ublox ANN-MB1 antenna
* LiPo battery
* Push button
* Resistor 10 k
* Switch

### Dependencies
* [ESPAsyncWebServer](https://github.com/me-no-dev/ESPAsyncWebServer)
* [RTKRoverManager](https://github.com/audio-communication-group/RTKRoverManager)

![plot](./fritzing/RTKRover-bb.png)

### Infrastructure:
* WiFi (e. g. a personal hotspot)
* free line of sight between antenna (horizontal placed) an sky

### Caster connection
To connect to a caster you need to create a CasterSecrets.h file with your credentials that looks like this:

````
#ifndef CASTER_SECRETS_H
#define CASTER_SECRETS_H
// A place for your caster credentials

// RTK2Go MountPoint http://www.rtk2go.com:2101/SNIP::STATUS#uptime
// Email: YOUR_ACCOUNT_EMAIL.COM
const char casterHost[] = "rtk2go.com";
const uint16_t casterPort = 2101;
const char mountPoint[] = "YOUR_RTK2GO_MOUNT_POINT";       //The mount point you want to push data to
const char casterUser[] = "YOUR_RTK2GO_ACCOUNT_EMAIL.COM";  //User must provide their own email address to use RTK2Go
const char casterUserPW[] = "";

// Or

// Another free NTRIP Caster is Emlid
// Emlid Caster MountPoint
// Email: YOUR_ACCOUNT_EMAIL.COM
const char casterHost[] = "caster.emlid.com";
const uint16_t casterPort = 2101;
const char casterUser[] = "YOUR_EMLID_USER_NAME";   //User name and pw must be obtained through their web portal
const char mountPoint[] = "YOUR_EMLID_MOUNT_POINT"; //The mount point you want to push data to
const char mountPointPW[] = "YOUR_EMLID_USER_PASSWORD";

// Use only one of this choices!

#endif /*** CASTER_SECRETS_H ***/

````
