/**********************************************************************
* Asyn device support using SSH communications library                *
**********************************************************************/
#ifndef DRVASYNPOWERPMACPORT_H
#define DRVASYNPOWERPMACPORT_H

#include <shareLib.h>  

#ifdef __cplusplus
extern "C" {
#endif  /* __cplusplus */

epicsShareFunc int drvAsynPowerPMACPortConfigure(const char *portName,
                                                 const char *hostaddress,
                                                 const char *username,
                                                 const char *password,
                                                 unsigned int priority,
                                                 int noAutoConnect,
                                                 int noProcessEos);

#ifdef __cplusplus
}
#endif  /* __cplusplus */
#endif  /* DRVASYNPOWERPMACPORT_H */
