#ifndef CASTER_SECRETS_H
#define CASTER_SECRETS_H
// A place for your caster credentials

//RTK2Go MountPoint 1 http://www.rtk2go.com:2101/SNIP::STATUS#uptime
// Email: mr.markuese@gmail.com
// const char CASTER_HOST[] = "rtk2go.com"; 
const char CASTER_HOST[] = "3.23.52.207:2101";
const uint16_t CASTER_PORT = 2101;
const char MOUNT_POINT[] = "rtkberlin";
const char CASTER_USER[] = "mr.markuese@gmail.com";  //User must provide their own email address to use RTK2Go
const char casterUserPW[] = ""; // This can be added to the web form if you want to use a PW here, it's not neccecary
const int CONNECTION_TIMEOUT_MS = 10000;

// Another free NTRIP Caster is Emlid
// Emlid Caster MountPoint 1
// Email: mr.markuese@gmail.com
// const char casterHost[] = "caster.emlid.com";
// const uint16_t casterPort = 2101;
//const char casterUser[] = "u44924"; //User name and pw must be obtained through their web portal
// const char mountPoint[] = "MP7156"; //The mount point you want to push data to
// const char mountPointPW[] = "545prm";

#endif /*** CASTER_SECRETS_H ***/