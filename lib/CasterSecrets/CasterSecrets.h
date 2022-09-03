#ifndef CASTER_SECRETS_H
#define CASTER_SECRETS_H
// A place for your caster credentials

//RTK2Go MountPoint 1 http://www.rtk2go.com:2101/SNIP::STATUS#uptime
// Email: mr.markuese@gmail.com
const char casterHost[] = "rtk2go.com";
const uint16_t casterPort = 2101;
// const char mountPoint[] = "HAN_TST"; // Hannover test mountpoint
const char mountPoint[] = "headtracker2punkt0";     //The mount point you want to push data to
const char casterUser[] = "mr.markuese@gmail.com";  //User must provide their own email address to use RTK2Go
const char casterUserPW[] = "";
const int CONNECTION_TIMEOUT_MS = 10000;


// RTK2Go MountPoint 2 http://www.rtk2go.com:2101/SNIP::STATUS#uptime
// Email: christian-schreck@mail.de
// const char casterHost[] = "rtk2go.com";
// const uint16_t casterPort = 2101;
// const char mountPoint[] = "HEADTRACKER2PUNKT0"; //The mount point you want to push data to
// const char casterUser[] = "christian-schreck@mail.de"; //User must provide their own email address to use RTK2Go
// const char casterUserPW[] = "";

// Another free NTRIP Caster is Emlid
// Emlid Caster MountPoint 1
// Email: mr.markuese@gmail.com
// const char casterHost[] = "caster.emlid.com";
// const uint16_t casterPort = 2101;
//const char casterUser[] = "u44924"; //User name and pw must be obtained through their web portal
// const char mountPoint[] = "MP7156"; //The mount point you want to push data to
// const char mountPointPW[] = "545prm";

#endif /*** CASTER_SECRETS_H ***/