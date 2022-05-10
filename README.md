### Headtracker + Real Time Kinematics (RTK Client)
Hardware used:   
* Adafruit Feather ESP32 Huzzah 
* SparkFun GPS-RTK-SMA Breakout - ZED-F9P (Qwiic)
* SparkFun BNO080 Breakout
* ublox ANN-MB1 antenna

To connect to a caster you need to create a secrets.h file with your credentials that looks like this:

````
#ifndef SECRETS_H
#define SECRETS_H
// A place for your caster credentials

//RTK2Go MountPoint 1 http://www.rtk2go.com:2101/SNIP::STATUS#uptime
// Email: YOUR_ACCOUNT_EMAIL.COM
const char casterHost[] = "rtk2go.com";
const uint16_t casterPort = 2101;
const char mountPoint[] = "YOUR_RTK2GO_MOUNT_POINT";       //The mount point you want to push data to
const char casterUser[] = "YOUR_RTK2GO_ACCOUNT_EMAIL.COM";  //User must provide their own email address to use RTK2Go
const char casterUserPW[] = "";

// Or

// Another free NTRIP Caster is Emlid
// Emlid Caster MountPoint 1
// Email: mr.markuese@gmail.com
const char casterHost[] = "caster.emlid.com";
const uint16_t casterPort = 2101;
const char casterUser[] = "YOUR_EMLID_USER_NAME";   //User name and pw must be obtained through their web portal
const char mountPoint[] = "YOUR_EMLID_MOUNT_POINT"; //The mount point you want to push data to
const char mountPointPW[] = "YOUR_EMLID_USER_PASSWORD";

// Use only one of this choices!

#endif /*** SECRETS_H ***/

````