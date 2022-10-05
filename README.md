<img align="right" src="./screenshots/rtkrover.jpeg" width="240"/> 

# Head Tracker + Real Time Kinematics (RTK Client)
## RTKRover

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
If you are not using the web form of the RTKRoverManager, to connect to a caster you need to create a CasterSecrets.h file with your credentials that looks like this:

````
#ifndef CASTER_SECRETS_H
#define CASTER_SECRETS_H
// A place for your caster credentials

// RTK2Go MountPoint http://www.rtk2go.com:2101/SNIP::STATUS#uptime
// Email: YOUR_ACCOUNT_EMAIL.COM
const char kCasterHost[] = "rtk2go.com";
const uint16_t kCasterPort = 2101;
const char kMountPoint[] = "YOUR_RTK2GO_MOUNT_POINT";       //The mount point you want to push data to
const char kCasterUser[] = "YOUR_RTK2GO_ACCOUNT_EMAIL.COM";  //User must provide their own email address to use RTK2Go
const char kCasterUserPW[] = "";

/*** Or another choice (you only need one): ***/

// Another free NTRIP Caster is Emlid
// Emlid Caster MountPoint
// Email: YOUR_ACCOUNT_EMAIL.COM
const char kCasterHost[] = "caster.emlid.com";
const uint16_t kCasterPort = 2101;
const char kCasterUser[] = "YOUR_EMLID_USER_NAME";   //User name and pw must be obtained through their web portal
const char kMountPoint[] = "YOUR_EMLID_MOUNT_POINT"; //The mount point you want to push data to
const char kMountPointPW[] = "YOUR_EMLID_USER_PASSWORD";


#endif /*** CASTER_SECRETS_H ***/

````

Then you have to replace the dynamically read credentials with the constant ones (add the k prefix e. g: mountPoint --> kMountPoint) in the main.cpp.
