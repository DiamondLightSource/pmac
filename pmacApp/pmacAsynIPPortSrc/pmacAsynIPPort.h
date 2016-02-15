#ifndef asynInterposePmac_H
#define asynInterposePmac_H

#include <shareLib.h>
#include <epicsExport.h>

#ifdef __cplusplus
extern "C" {
#endif  /* __cplusplus */

epicsShareFunc int pmacAsynIPPortConfigure(const char *portName,int addr);
epicsShareFunc int pmacAsynIPConfigure(const char *portName, const char *hostInfo);

#ifdef __cplusplus
}
#endif  /* __cplusplus */

#endif /* asynInterposePmac_H */
