/*
   NOTES
   
   This driver is an asyn interpose driver intended for use with pmacAsynMotor to allow communication over ethernet to a PMAC.

   *** Ensure I3=2 and I6=1 on the PMAC before using this driver. ***
   
   This driver supports sending ascii commands to the PMAC over asyn IP and obtaining the response. The driver uses the PMAC ethernet 
   packets VR_PMAC_GETRESPONSE, VR_PMAC_READREADY and VR_PMAC_GETBUFFER to send commands and get responses. The PMAC may send responses 
   in several different formats.  
   1) PMAC ascii command responses for single/multiple commands (e.g. I113 I114 I130 I131 I133) are in an ACK terminated 
      form as follows (CR seperates the cmd responses):
         data<CR(13)>data<CR(13)>data<CR(13)><ACK(6)>
   2) PMAC error responses to ascii commands ARE NOT ACK terminated as follows:
         <BELL(7)>ERRxxx<CR(13)>
   3) PMAC may also return the following:
         <STX(2)>data<CR(13)> 
   
   This driver can send ctrl commands (ctrl B/C/F/G/P/V) to the pmac (using VR_CTRL_REPONSE packet) however because the resulting  response 
   data is not terminated as above the driver does not know when all the response data has been received. The response data will therefore 
   only be returned after the asynUser.timeout has expired.
   
   This driver supports the octet flush method and issues a VR_PMAC_FLUSH to the PMAC.
   
   This driver does NOT support large buffer transfers (VR_PMAC_WRITEBUFFER, VR_FWDOWNLOAD) or set/get of DPRAM (VR_PMAC_SETMEM etc) or changing
   comms setup (VR_IPADDRESS, VR_PMAC_PORT)


   REVISION HISTORY
   
   10 Aug 07 - Pete Leicester - Diamond Light Source
   Modified to handle responses other than those ending in <ACK> (e.g. errors) - No longer used asyn EOS.
   
   9 Aug 07 - Pete Leicester - Diamond Light Source
   Initial version reliant on asyn EOS to return <ACK> terminated responses. 

   2 Feb 09 - Matthew Pearson - Diamond Light Source
   Ported to work with Asyn 4-10. Still compiles with pre Asyn4-10 versions but does not work.
   Also added new config function, pmacAsynIPPortConfigureEos(), to be used when disabling 
   low level EOS handling in the Asyn IP layer. This new function should be used with Asyn 4-10 and above 
   (it is not compatible with Asyn 4-9).
*/  


#include <stddef.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

#include <cantProceed.h>
#include <epicsAssert.h>
#include <epicsStdio.h>
#include <epicsString.h>
#include <osiSock.h>
#include <iocsh.h>

#include "asynDriver.h"
#include "asynOctet.h"
#include "pmacAsynIPPort.h"
#include "asynInterposeEos.h"
#include "drvAsynIPPort.h"
#include "epicsThread.h"
#include <epicsExport.h>


#define ETHERNET_DATA_SIZE 1492
#define MAX_BUFFER_SIZE 2097152
#define INPUT_SIZE        (ETHERNET_DATA_SIZE+1)  /* +1 to allow space to add terminating ACK */
#define STX   '\2'
#define CTRLB '\2'
#define CTRLC '\3'
#define ACK   '\6'
#define CTRLF '\6'
#define BELL  '\7'
#define CTRLG '\7'
#define CTRLP '\16'
#define CTRLV '\22'
#define CTRLX '\24'

/* PMAC ethernet command structure */
#pragma pack(1)
typedef struct tagEthernetCmd
{
    unsigned char RequestType;
    unsigned char Request;
    unsigned short wValue;
    unsigned short wIndex;
    unsigned short wLength; /* length of bData */
    unsigned char bData[ETHERNET_DATA_SIZE];
} ethernetCmd;
#pragma pack()

#define ETHERNET_CMD_HEADER ( sizeof(ethernetCmd) - ETHERNET_DATA_SIZE )

/* PMAC ethernet commands - RequestType field */
#define VR_UPLOAD   0xC0
#define VR_DOWNLOAD 0x40

/* PMAC ethernet commands - Request field */
#define VR_PMAC_SENDLINE    0xB0
#define VR_PMAC_GETLINE     0xB1
#define VR_PMAC_FLUSH       0xB3
#define VR_PMAC_GETMEM      0xB4
#define VR_PMAC_SETMEN      0xB5
#define VR_PMAC_SETBIT      0xBA
#define VR_PMAC_SETBITS     0xBB
#define VR_PMAC_PORT        0xBE
#define VR_PMAC_GETRESPONSE 0xBF
#define VR_PMAC_READREADY   0xC2
#define VR_CTRL_RESPONSE    0xC4
#define VR_PMAC_GETBUFFER   0xC5
#define VR_PMAC_WRITEBUFFER 0xC6
#define VR_PMAC_WRITEERROR  0xC7
#define VR_FWDOWNLOAD       0xCB
#define VR_IPADDRESS        0xE0

/* PMAC control character commands (VR_CTRL_RESPONSE cmd) */
static char ctrlCommands[] = { CTRLB,CTRLC,CTRLF,CTRLG,CTRLP,CTRLV };

typedef struct pmacPvt {
    char          *portName;
    int           addr;
    asynInterface pmacInterface;
    asynOctet     *poctet;  /* The methods we're overriding */
    void          *octetPvt;
    asynUser      *pasynUser;     /* For connect/disconnect reporting */
    ethernetCmd   *poutCmd;
    ethernetCmd   *pinCmd;
    char          *inBuf;
    unsigned int  inBufHead;
    unsigned int  inBufTail;
    } pmacPvt;

/*
   Notes on asyn
   use asynUser.timeout to specify I/O request timeouts in seconds
   asynStatus may return asynSuccess(0),asynTimeout(1),asynOverflow(2) or asynError(3)
   eomReason may return ASYN_EOM_CNT (1:Request count reached), ASYN_EOM_EOS (2:End of String detected), ASYN_EOM_END (4:End indicator detected)
   asynError indicates that asynUser.errorMessage has been populated by epicsSnprintf(). 
*/

/* Connect/disconnect handling */
static void pmacInExceptionHandler(asynUser *pasynUser,asynException exception);

/* asynOctet methods */
static asynStatus writeIt(void *ppvt,asynUser *pasynUser,
    const char *data,size_t numchars,size_t *nbytesTransfered);
static asynStatus readIt(void *ppvt,asynUser *pasynUser,
    char *data,size_t maxchars,size_t *nbytesTransfered,int *eomReason);
static asynStatus flushIt(void *ppvt,asynUser *pasynUser);
static asynStatus registerInterruptUser(void *ppvt,asynUser *pasynUser,
    interruptCallbackOctet callback, void *userPvt,void **registrarPvt);
static asynStatus cancelInterruptUser(void *drvPvt,asynUser *pasynUser,
     void *registrarPvt);
static asynStatus setInputEos(void *ppvt,asynUser *pasynUser,
    const char *eos,int eoslen);
static asynStatus getInputEos(void *ppvt,asynUser *pasynUser,
    char *eos,int eossize ,int *eoslen);
static asynStatus setOutputEos(void *ppvt,asynUser *pasynUser,
    const char *eos,int eoslen);
static asynStatus getOutputEos(void *ppvt,asynUser *pasynUser,
    char *eos,int eossize,int *eoslen);

/* Declare asynOctet here, and assign functions later on 
(compatible with both Asyn 4-10 and pre 4-10 versions).*/
static asynOctet octet;

static asynStatus readResponse(pmacPvt *pPmacPvt, asynUser *pasynUser, size_t maxchars, size_t *nbytesTransfered, int *eomReason );
static int pmacReadReady(pmacPvt *pPmacPvt, asynUser *pasynUser );
static int pmacFlush(pmacPvt *pPmacPvt, asynUser *pasynUser );
static int pmacAsynIPPortCommon(const char *portName, int addr, pmacPvt **pPmacPvt, asynInterface **plowerLevelInterface, asynUser **pasynUser);
epicsShareFunc int pmacAsynIPPortConfigureEos(const char *portName,int addr);
static asynStatus sendPmacGetBuffer(pmacPvt *pPmacPvt, asynUser *pasynUser, size_t maxchars,size_t *nbytesTransfered);

/**
 * Function that first initialises an Asyn IP port and then the PMAC Asyn IP interpose layer.
 * It is a wrapper for drvAsynIPPort::drvAsynIPPortConfigure() and 
 * pmacAsynIPPort::pmacAsynIPPortConfigureEos().
 * 
 * @param portName The Asyn Port name string.
 * @param hostInfo The hostname or IP address followed by IP port (eg. 172.23.243.156:1025)
 * @return status
 */
epicsShareFunc int pmacAsynIPConfigure(const char *portName, const char *hostInfo)
{
  asynStatus status = 0;

  if ((status = drvAsynIPPortConfigure(portName, hostInfo, 0, 0, 1)) != 0) {
    printf("pmacAsynIPConfigure: error from drvAsynIPPortConfigure. Port: %s\n", portName);
  }

  if ((status = pmacAsynIPPortConfigureEos(portName, 0)) != 0) {
    printf("pmacAsynIPConfigure: error from pmacAsynIPPortConfigureEos. Port: %s\n", portName);
  }
  
  return status;
}

/**
 * This reimplements pmacAsynIPPort::pmacAsynIPPortConfigure(), but introduces
 * EOS handling interpose layer above the PMAC IP layer.
 *
 * The setup of the Asyn IP port must set noProcessEos=1 to use this function. 
 *
 * NOTE: The use of this function is not compatible with versions of Asyn before 4-10.
 *
 * @param portName The Asyn port name
 * @param addr The Asyn address.
 * @return status
 */
epicsShareFunc int pmacAsynIPPortConfigureEos(const char *portName,int addr)
{
  pmacPvt       *pPmacPvt = NULL;
  asynInterface *plowerLevelInterface = NULL;
  asynStatus    status = 0;
  asynUser      *pasynUser = NULL;
  
  status = pmacAsynIPPortCommon(portName, addr, &pPmacPvt, &plowerLevelInterface, &pasynUser);
  if (status) {
    asynPrint(pasynUser, ASYN_TRACE_ERROR,
	      "pmacAsynIPPortConfigureEos: error from pmacAsynIPPortCommon %s: %s\n", 
	      portName, pasynUser->errorMessage);
  }
  
  /*Now interpose EOS handling layer, above the PMAC IP layer.*/
  asynInterposeEosConfig(portName,addr,1,1);
  
  asynPrint(pasynUser,ASYN_TRACE_FLOW, "Done pmacAsynIPPortConfigureEos OK\n");
  return(0);
}


epicsShareFunc int pmacAsynIPPortConfigure(const char *portName,int addr)
{
  pmacPvt       *pPmacPvt = NULL;
  asynInterface *plowerLevelInterface = NULL;
  asynUser      *pasynUser = NULL;
  asynStatus    status = 0;
 
  status = pmacAsynIPPortCommon(portName, addr, &pPmacPvt, &plowerLevelInterface, &pasynUser);

  status = pPmacPvt->poctet->setInputEos(pPmacPvt->octetPvt, pasynUser, "\6", 1);
  if (status) {
    asynPrint(pasynUser, ASYN_TRACE_ERROR,
	      "pmacAsynIPPortConfigure: unable to set input EOS on %s: %s\n", 
	      portName, pasynUser->errorMessage);
    return -1;
  }
  status = pPmacPvt->poctet->setOutputEos(pPmacPvt->octetPvt, pasynUser, "\r", 1);
  if (status) {
    asynPrint(pasynUser, ASYN_TRACE_ERROR,
	      "pmacAsynIPPortConfigure: unable to set output EOS on %s: %s\n", 
	      portName, pasynUser->errorMessage);
    return -1;
  }

  asynPrint(pasynUser,ASYN_TRACE_FLOW, "Done pmacAsynIPPortConfigure OK\n");
  return(0);
}

/**
 * Common code for both pmacAsynIPPortConfigure and pmacAsynIPPortConfigureEos.
 * @param portName
 * @param addr
 * @param pPmacPvt pointer
 * @param plowerLevelInterface pointer
 * @param pasynUser pointer
 * @return status
 */
static int pmacAsynIPPortCommon(const char *portName,
				int addr,
				pmacPvt **pPmacPvt,
				asynInterface **plowerLevelInterface,
				asynUser **pasynUser)
{
    asynStatus    status;
    size_t        len;

    /*Assign static asynOctet functions here.*/
    octet.write = writeIt;
    octet.read = readIt;
    octet.flush = flushIt;
    octet.setInputEos = setInputEos;
    octet.setOutputEos = setOutputEos;
    octet.getInputEos = getInputEos;
    octet.getOutputEos = getOutputEos;
    octet.registerInterruptUser = registerInterruptUser;
    octet.cancelInterruptUser = cancelInterruptUser;

    len = sizeof(pmacPvt) + strlen(portName) + 1;
    *pPmacPvt = callocMustSucceed(1,len,"calloc pPmacPvt error in pmacAsynIPPort::pmacAsynIPPortCommon().");
    (*pPmacPvt)->portName = (char *)(*pPmacPvt+1);
    strcpy((*pPmacPvt)->portName,portName);
    (*pPmacPvt)->addr = addr;
    (*pPmacPvt)->pmacInterface.interfaceType = asynOctetType;
    (*pPmacPvt)->pmacInterface.pinterface = &octet;
    (*pPmacPvt)->pmacInterface.drvPvt = *pPmacPvt;
    *pasynUser = pasynManager->createAsynUser(0,0);
    (*pPmacPvt)->pasynUser = *pasynUser;
    (*pPmacPvt)->pasynUser->userPvt = *pPmacPvt;
     
    status = pasynManager->connectDevice(*pasynUser,portName,addr);
    if(status!=asynSuccess) {
      printf("pmacAsynIPPortConfigure: %s connectDevice failed\n",portName);
      pasynManager->freeAsynUser(*pasynUser);
      free(*pPmacPvt);
      return -1;
    }
    
    status = pasynManager->exceptionCallbackAdd(*pasynUser,pmacInExceptionHandler);
    if(status!=asynSuccess) {
      printf("pmacAsynIPPortConfigure: %s exceptionCallbackAdd failed\n",portName);
      pasynManager->freeAsynUser(*pasynUser);
      free(*pPmacPvt);
      return -1;
    }

    status = pasynManager->interposeInterface(portName,addr,
      &(*pPmacPvt)->pmacInterface,&(*plowerLevelInterface));
    if(status!=asynSuccess) {
        printf("pmacAsynIPPortConfigure: %s interposeInterface failed\n",portName);
        pasynManager->exceptionCallbackRemove(*pasynUser);
        pasynManager->freeAsynUser(*pasynUser);
        free(*pPmacPvt);
        return -1;
	}

    (*pPmacPvt)->poctet = (asynOctet *)(*plowerLevelInterface)->pinterface;
    (*pPmacPvt)->octetPvt = (*plowerLevelInterface)->drvPvt;

    (*pPmacPvt)->poutCmd = callocMustSucceed(1,sizeof(ethernetCmd),"calloc poutCmd error in pmacAsynIPPort::pmacAsynIPPortCommon().");
    (*pPmacPvt)->pinCmd = callocMustSucceed(1,sizeof(ethernetCmd),"calloc pinCmd error in pmacAsynIPPort::pmacAsynIPPortCommon().");
    
    (*pPmacPvt)->inBuf = callocMustSucceed(1,MAX_BUFFER_SIZE,"calloc inBuf error in pmacAsynIPPort::pmacAsynIPPortCommon().");
    
    (*pPmacPvt)->inBufHead = 0;
    (*pPmacPvt)->inBufTail = 0;
    
    return status;
}


static void pmacInExceptionHandler(asynUser *pasynUser,asynException exception)
{
    pmacPvt *pPmacPvt = (pmacPvt *)pasynUser->userPvt;
    asynPrint(pasynUser,ASYN_TRACE_FLOW, "pmacInExceptionHandler\n");

    if (exception == asynExceptionConnect) {
        pPmacPvt->inBufHead = 0;
        pPmacPvt->inBufTail = 0;
    }
}

/*
    Read reponse data from PMAC into buffer pmacPvt.inBuf. If there is no data in the asyn buffer then issue
    pmac GETBUFFER command to get any outstanding data still on the PMAC.
*/
static asynStatus readResponse(pmacPvt *pPmacPvt, asynUser *pasynUser, size_t maxchars, size_t *nbytesTransfered, int *eomReason )
{
    asynStatus status = asynSuccess;
    size_t thisRead = 0;
    *nbytesTransfered = 0;
    if (maxchars>INPUT_SIZE) maxchars = INPUT_SIZE;

    asynPrint(pasynUser,ASYN_TRACE_FLOW, "pmacAsynIPPort::readResponse. Performing pPmacPvt->poctet->read().\n");

    status = pPmacPvt->poctet->read(pPmacPvt->octetPvt,
      pasynUser,pPmacPvt->inBuf,maxchars,&thisRead,eomReason);

    asynPrint(pasynUser,ASYN_TRACE_FLOW, "%s readResponse1 maxchars=%zd, thisRead=%zd, eomReason=%d, status=%u\n",pPmacPvt->portName, maxchars, thisRead, *eomReason, status);     
    if (status == asynTimeout && thisRead == 0 && pasynUser->timeout>0) {
         /* failed to read as many characters as required into the input buffer, 
            check for more response data on the PMAC */
         if ( pmacReadReady(pPmacPvt,pasynUser )) { 

	    status = sendPmacGetBuffer(pPmacPvt, pasynUser, maxchars, nbytesTransfered);
            asynPrintIO(pasynUser,ASYN_TRACE_FLOW,(char*)pPmacPvt->pinCmd,ETHERNET_CMD_HEADER,
                "%s write GETBUFFER\n",pPmacPvt->portName);
                
            /* We have nothing to return at the moment so read again */
	    status = pPmacPvt->poctet->read(pPmacPvt->octetPvt,
	      pasynUser,pPmacPvt->inBuf,maxchars,&thisRead,eomReason);
               
            asynPrint(pasynUser,ASYN_TRACE_FLOW, "%s readResponse2 maxchars=%zd, thisRead=%zd, eomReason=%d, status=%u\n",pPmacPvt->portName, maxchars, thisRead, *eomReason, status);                    
        } 
    } 
          
    if (thisRead>0) {
        if (status == asynTimeout) status = asynSuccess;
        *nbytesTransfered = thisRead;   
        pPmacPvt->inBufTail = 0;
        pPmacPvt->inBufHead = thisRead;
    }

    asynPrint( pasynUser, ASYN_TRACE_FLOW, "pmacAsynIPPort::readResponse. END\n" );

    return status; 
}


/*
   Send ReadReady command to PMAC to discover if there is any data to read from it.
   Returns: 0 - no data available
            1 - data available
*/            
static int pmacReadReady(pmacPvt *pPmacPvt, asynUser *pasynUser )
{
    ethernetCmd cmd;
    char data[2] = {0};
    asynStatus status;
    size_t thisRead = 0;
    size_t nbytesTransfered = 0;
    int eomReason = 0;
    int retval = 0;

    cmd.RequestType = VR_UPLOAD;
    cmd.Request = VR_PMAC_READREADY;
    cmd.wValue = 0;
    cmd.wIndex = 0;
    cmd.wLength = htons(2);

    status = pPmacPvt->poctet->write(pPmacPvt->octetPvt,
      pasynUser,(char*)&cmd,ETHERNET_CMD_HEADER,&nbytesTransfered);
    
    if(status!=asynSuccess) { 
        asynPrintIO(pasynUser,ASYN_TRACE_ERROR,(char*)&cmd,ETHERNET_CMD_HEADER,
            "%s write pmacReadReady fail\n",pPmacPvt->portName);
    }
     
    status = pPmacPvt->poctet->read(pPmacPvt->octetPvt,
      pasynUser,data,2,&thisRead,&eomReason);

    if(status==asynSuccess) {
         if (thisRead==2 && data[0] != 0) {
             retval = 1;
         }    
         asynPrintIO(pasynUser,ASYN_TRACE_FLOW,data,thisRead,
             "%s read pmacReadReady OK thisRead=%zd\n",pPmacPvt->portName,thisRead);
    } else {
        asynPrint(pasynUser,ASYN_TRACE_ERROR, "%s read pmacReadReady failed status=%d,retval=%d",pPmacPvt->portName,status,retval);
    }
    return retval;            
}

/*
   Send Flush command to PMAC and wait for confirmation ctrlX to be returned. 
   Returns: 0 - failed
            1 - success
*/            
static int pmacFlush(pmacPvt *pPmacPvt, asynUser *pasynUser )
{
    ethernetCmd cmd;
    char data[2];
    asynStatus status = asynSuccess;
    size_t thisRead;
    size_t nbytesTransfered = 0;
    int eomReason = 0;
    int retval = 0;

    cmd.RequestType = VR_DOWNLOAD;
    cmd.Request = VR_PMAC_FLUSH;
    cmd.wValue = 0;
    cmd.wIndex = 0;
    cmd.wLength = 0;

    status = pPmacPvt->poctet->write(pPmacPvt->octetPvt,
      pasynUser,(char*)&cmd,ETHERNET_CMD_HEADER,&nbytesTransfered);
    
    if(status!=asynSuccess) { 
        asynPrintIO(pasynUser,ASYN_TRACE_ERROR,(char*)&cmd,ETHERNET_CMD_HEADER,
            "%s write pmacFlush fail\n",pPmacPvt->portName);
    }
        
    /* read flush acknowledgement character */
    /* NB we don't check what the character is (manual sais ctrlX, we get 0x40 i.e.VR_DOWNLOAD) */
    status = pPmacPvt->poctet->read(pPmacPvt->octetPvt,
      pasynUser,data,1,&thisRead,&eomReason);

    if(status==asynSuccess) {
         asynPrint(pasynUser,ASYN_TRACE_FLOW, "%s read pmacFlush OK\n",pPmacPvt->portName);
         retval = 1;    
    } else {
        asynPrint(pasynUser,ASYN_TRACE_ERROR, "%s read pmacFlush failed - thisRead=%zd, eomReason=%d, status=%d\n",pPmacPvt->portName, thisRead, eomReason, status);
    }

    pPmacPvt->inBufTail = 0;
    pPmacPvt->inBufHead = 0;
    
    return retval;            
}


/* asynOctet methods */


/* This function sends either a ascii string command to the PMAC using VR_PMAC_GETRESPONSE or a single control 
   character (ctrl B/C/F/G/P/V) using VR_CTRL_RESPONSE
*/
static asynStatus writeIt(void *ppvt,asynUser *pasynUser,
    const char *data,size_t numchars,size_t *nbytesTransfered)
{
    asynStatus status;
    ethernetCmd* outCmd;
    pmacPvt *pPmacPvt = (pmacPvt *)ppvt;
    size_t nbytesActual = 0;
    asynPrint( pasynUser, ASYN_TRACE_FLOW, "pmacAsynIPPort::writeIt\n" );
    assert(pPmacPvt);
    
    /* NB currently we assume control characters arrive as individual characters/calls to this routine. Idealy we should probably
       scan the data buffer for control commands and do PMAC_GETRESPONSE and CTRL_RESPONSE commands as necessary,  */
    outCmd = pPmacPvt->poutCmd;
    if (numchars==1 && strchr(ctrlCommands,data[0])){
        outCmd->RequestType = VR_UPLOAD;
        outCmd->Request = VR_CTRL_RESPONSE;
        outCmd->wValue = data[0];
        outCmd->wIndex = 0;
        outCmd->wLength = htons(0);
	status = pPmacPvt->poctet->write(pPmacPvt->octetPvt,
	  pasynUser,(char*)pPmacPvt->poutCmd,ETHERNET_CMD_HEADER,&nbytesActual);
        *nbytesTransfered = (nbytesActual==ETHERNET_CMD_HEADER) ? numchars : 0;
    } else {
        if (numchars > ETHERNET_DATA_SIZE) {
            /* NB large data should probably be sent using PMAC_WRITEBUFFER which isnt implemented yet - for the moment just truncate */
            numchars = ETHERNET_DATA_SIZE;
            asynPrint( pasynUser, ASYN_TRACE_ERROR, "writeIt - ERROR TRUNCATED\n" );
        }
        outCmd->RequestType = VR_DOWNLOAD;
        outCmd->Request = VR_PMAC_GETRESPONSE;
        outCmd->wValue = 0;
        outCmd->wIndex = 0;
        outCmd->wLength = htons(numchars);
        memcpy(outCmd->bData,data,numchars);
	status = pPmacPvt->poctet->write(pPmacPvt->octetPvt,
	  pasynUser,(char*)pPmacPvt->poutCmd,numchars+ETHERNET_CMD_HEADER,&nbytesActual);
        *nbytesTransfered = (nbytesActual>ETHERNET_CMD_HEADER) ? (nbytesActual - ETHERNET_CMD_HEADER) : 0;
    }
        
    asynPrintIO(pasynUser,ASYN_TRACE_FLOW,(char*)pPmacPvt->poutCmd,numchars+ETHERNET_CMD_HEADER,
            "%s writeIt\n",pPmacPvt->portName);
                    
    return status;
}


/* This function reads data using read() into a local buffer and then look for message terminating characters and returns a complete 
   response (or times out), adding on ACK if neccessary.
   The PMAC command response may be any of the following:-
       data<CR>data<CR>....data<CR><ACK>
       <BELL>data<CR> e.g. an error <BELL>ERRxxx<CR>
       <STX>data<CR>
   (NB asyn EOS only allows one message terminator to be specified. We add on ACK for the EOS layer above.)    
*/             
static asynStatus readIt(void *ppvt,asynUser *pasynUser,
    char *data,size_t maxchars,size_t *nbytesTransfered,int *eomReason)
{
    pmacPvt *pPmacPvt = (pmacPvt *)ppvt;
    asynStatus status = asynSuccess;
    size_t thisRead = 0;
    size_t nRead = 0;
    int bell = 0;
    int initialRead = 1;
 
    asynPrint( pasynUser, ASYN_TRACE_FLOW, "pmacAsynIPPort::readIt. START\n" );
    assert(pPmacPvt);

    if (maxchars > 0) {
      for (;;) {
	if ((pPmacPvt->inBufTail != pPmacPvt->inBufHead)) {
	  *data = pPmacPvt->inBuf[pPmacPvt->inBufTail++];
	  if (*data == BELL || *data == STX) bell = 1;
	  if (*data == '\r' && bell) { 
	    /* <BELL>xxxxxx<CR> or <STX>xxxxx<CR> received - its probably an error response (<BELL>ERRxxx<CR>) - assume there is no more response data to come */
	    nRead++; /* make sure the <CR> is passed to the client app */
	    /*Add on ACK, because that's what we expect to be EOS in EOS interpose layer.*/
	    if ((nRead+1)>maxchars) {
	      /*If maxchars is reached overwrite <CR> with ACK, so that no more reads will be done from EOS layer.*/
	      *data = ACK;
	    } else {
	      data++;
	      nRead++;
	      *data = ACK;
	    }
	    break;
	  }
	  if (*data == ACK || *data == '\n') {
	    /* <ACK> or <LF> received - assume there is no more response data to come */
	    /* If <LF>, replace with an ACK.*/
	    if (*data == '\n') {
	      *data = ACK;
	    }
	    asynPrint( pasynUser, ASYN_TRACE_FLOW, "Message was terminated with ACK in pmacAsynIPPort::readIt.\n" );
	    /*Pass ACK up to Asyn EOS handling layer.*/
	    data++;
	    nRead++;
	    break;
	  }
	  data++;
	  nRead++;
	  if (nRead >= maxchars) break;
	  continue;
	}
	
	asynPrint( pasynUser, ASYN_TRACE_FLOW, "pmacAsynIPPort::readIt. Calling readResponse().\n" );
	if (!initialRead) {
	  if (pmacReadReady(pPmacPvt, pasynUser)) { 
	    status = sendPmacGetBuffer(pPmacPvt, pasynUser, maxchars, nbytesTransfered);
	  }
	}
	status = readResponse(pPmacPvt, pasynUser, maxchars-nRead, &thisRead, eomReason);
	initialRead = 0;
	if(status!=asynSuccess || thisRead==0) break;       
      }
    }
    *nbytesTransfered = nRead;
    
    asynPrintIO(pasynUser,ASYN_TRACE_FLOW, data, *nbytesTransfered, "%s pmacAsynIPPort readIt nbytesTransfered=%zd, eomReason=%d, status=%d\n",pPmacPvt->portName,*nbytesTransfered, *eomReason, status);
             
	asynPrint( pasynUser, ASYN_TRACE_FLOW, "pmacAsynIPPort::readIt. END\n" );

    return status;
}

static asynStatus sendPmacGetBuffer(pmacPvt *pPmacPvt, asynUser *pasynUser, size_t maxchars,size_t *nbytesTransfered)
{
  asynStatus status = 0;
  ethernetCmd* inCmd = NULL;

  inCmd = pPmacPvt->pinCmd;
  inCmd->RequestType = VR_UPLOAD;
  inCmd->Request = VR_PMAC_GETBUFFER;
  inCmd->wValue = 0;
  inCmd->wIndex = 0;
  inCmd->wLength = htons(maxchars);
  status = pPmacPvt->poctet->write(pPmacPvt->octetPvt,
				   pasynUser,(char*)pPmacPvt->pinCmd,ETHERNET_CMD_HEADER,nbytesTransfered);
  
  return status;

}


static asynStatus flushIt(void *ppvt,asynUser *pasynUser)
{
    pmacPvt *pPmacPvt = (pmacPvt *)ppvt;
    asynStatus status;
    asynPrint( pasynUser, ASYN_TRACE_FLOW, "pmacAsynIPPort::flushIt\n" );
    assert(pPmacPvt);

    pmacFlush (pPmacPvt, pasynUser);
    status = pPmacPvt->poctet->flush(pPmacPvt->octetPvt,pasynUser);
    return status;
}

static asynStatus registerInterruptUser(void *ppvt,asynUser *pasynUser,
    interruptCallbackOctet callback, void *userPvt,void **registrarPvt)
{
    pmacPvt *pPmacPvt = (pmacPvt *)ppvt;
    asynPrint( pasynUser, ASYN_TRACE_FLOW, "pmacAsynIPPort::registerInterruptUser\n" );
    assert(pPmacPvt);

    return pPmacPvt->poctet->registerInterruptUser(pPmacPvt->octetPvt,
        pasynUser,callback,userPvt,registrarPvt);
} 

static asynStatus cancelInterruptUser(void *drvPvt,asynUser *pasynUser,
     void *registrarPvt)
{
    pmacPvt *pPmacPvt = (pmacPvt *)drvPvt;
    asynPrint( pasynUser, ASYN_TRACE_FLOW, "pmacAsynIPPort::cancelInterruptUser\n" );
    assert(pPmacPvt);

    return pPmacPvt->poctet->cancelInterruptUser(pPmacPvt->octetPvt,
        pasynUser,registrarPvt);
} 

static asynStatus setInputEos(void *ppvt,asynUser *pasynUser,
    const char *eos,int eoslen)
{
    pmacPvt *pPmacPvt = (pmacPvt *)ppvt;
    asynPrint( pasynUser, ASYN_TRACE_FLOW, "pmacAsynIPPort::setInputEos\n" );
    assert(pPmacPvt);

    return pPmacPvt->poctet->setInputEos(pPmacPvt->octetPvt,pasynUser,
      eos,eoslen); 
}

static asynStatus getInputEos(void *ppvt,asynUser *pasynUser,
    char *eos,int eossize,int *eoslen)
{
    pmacPvt *pPmacPvt = (pmacPvt *)ppvt;
    asynPrint( pasynUser, ASYN_TRACE_FLOW, "pmacAsynIPPort::getInputEos\n" );
    assert(pPmacPvt);

    return pPmacPvt->poctet->getInputEos(pPmacPvt->octetPvt,pasynUser,
       eos,eossize,eoslen);
}

static asynStatus setOutputEos(void *ppvt,asynUser *pasynUser,
    const char *eos, int eoslen)
{
    pmacPvt *pPmacPvt = (pmacPvt *)ppvt;
    asynPrint( pasynUser, ASYN_TRACE_FLOW, "pmacAsynIPPort::setOutputEos\n" );
    assert(pPmacPvt);

    return pPmacPvt->poctet->setOutputEos(pPmacPvt->octetPvt,
        pasynUser,eos,eoslen); 
}

static asynStatus getOutputEos(void *ppvt,asynUser *pasynUser,
    char *eos,int eossize,int *eoslen)
{
    pmacPvt *pPmacPvt = (pmacPvt *)ppvt;
    asynPrint( pasynUser, ASYN_TRACE_FLOW, "pmacAsynIPPort::getOutputEos\n" );
    assert(pPmacPvt);

    return pPmacPvt->poctet->getOutputEos(pPmacPvt->octetPvt,
        pasynUser,eos,eossize,eoslen);
}


/* register pmacAsynIPPortConfigure*/
static const iocshArg pmacAsynIPPortConfigArg0 =
    { "portName", iocshArgString };
static const iocshArg pmacAsynIPPortConfigArg1 =
    { "addr", iocshArgInt };
static const iocshArg *pmacAsynIPPortConfigArgs[] = 
    {&pmacAsynIPPortConfigArg0,&pmacAsynIPPortConfigArg1};
static const iocshFuncDef pmacAsynIPPortConfigFuncDef =
    {"pmacAsynIPPortConfigure", 2, pmacAsynIPPortConfigArgs};
static void pmacAsynIPPortConfigCallFunc(const iocshArgBuf *args)
{
    pmacAsynIPPortConfigure(args[0].sval,args[1].ival);
}

/* Register pmacAsynIPPortConfigureEos.*/
static const iocshArg pmacAsynIPPortConfigEosArg0 =
    { "portName", iocshArgString };
static const iocshArg pmacAsynIPPortConfigEosArg1 =
    { "addr", iocshArgInt };
static const iocshArg *pmacAsynIPPortConfigEosArgs[] = 
    {&pmacAsynIPPortConfigEosArg0,&pmacAsynIPPortConfigEosArg1};
static const iocshFuncDef pmacAsynIPPortConfigEosFuncDef =
    {"pmacAsynIPPortConfigureEos", 2, pmacAsynIPPortConfigEosArgs};
static void pmacAsynIPPortConfigEosCallFunc(const iocshArgBuf *args)
{
    pmacAsynIPPortConfigureEos(args[0].sval,args[1].ival);
}


/* Register pmacAsynIPConfigure.*/
static const iocshArg pmacAsynIPConfigureArg0 =
    { "portName", iocshArgString };
static const iocshArg pmacAsynIPConfigureArg1 =
    { "hostInfo", iocshArgString };
static const iocshArg *pmacAsynIPConfigureArgs[] = 
    {&pmacAsynIPConfigureArg0,&pmacAsynIPConfigureArg1};
static const iocshFuncDef pmacAsynIPConfigureFuncDef =
    {"pmacAsynIPConfigure", 2, pmacAsynIPConfigureArgs};
static void pmacAsynIPConfigureCallFunc(const iocshArgBuf *args)
{
    pmacAsynIPConfigure(args[0].sval,args[1].sval);
}

static void pmacAsynIPPortRegister(void)
{
    static int firstTime = 1;
    if (firstTime) {
        firstTime = 0;
        iocshRegister(&pmacAsynIPPortConfigFuncDef, pmacAsynIPPortConfigCallFunc);
	iocshRegister(&pmacAsynIPPortConfigEosFuncDef, pmacAsynIPPortConfigEosCallFunc);
	iocshRegister(&pmacAsynIPConfigureFuncDef, pmacAsynIPConfigureCallFunc);
    }
}
epicsExportRegistrar(pmacAsynIPPortRegister);
