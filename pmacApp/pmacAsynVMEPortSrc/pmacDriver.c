/*
 *  Implementation of the Open/Close/Read/Write/Ioctl interface to
 *  PMAC DPRAM ASCII and PMAC Mailbox ASCII.
 *
 *  Author: Andy Foster (for Diamond)
 *  Date:   26th May 2006
 *
*/

/* ANSI C headers */
#include <stdio.h>

/* vxWorks headers */
#include <iosLib.h>    /* For DEV_HDR, iosDrvCreate and iosDevAdd */
#include <logLib.h>    /* For logMsg */
#include <taskLib.h>   /* For taskDelay */

/* EPICS headers */
#include <epicsRingBytes.h>
#include <epicsTypes.h>
#include <epicsThread.h>
#include <epicsEvent.h>
#include <cantProceed.h>
#include <devLib.h>

/* PMAC headers */
#include <pmacVme.h>
#include <pmacDriver.h>

/** Makes writing to a register on the VME board a bit more explicit */
#define getReg(location)          (location)
#define setReg(location, value)   (location) = (value)

#define PMAC_DRIVER_DEBUG        0
#define PMAC_BASE_MBX_REGS_IN   16
#define PMAC_BASE_MBX_REGS_OUT  15
#define PMAC_BASE_ASC_REGS_OUT 160

int    pmacDrvNumAsc  = 0;     /* DPRAM ASCII driver number   */
int    pmacDrvNumMbx  = 0;     /* Mailbox ASCII driver number */
int    replyQueueSize = 40960; /* Size of ring buffer - large enough for a list gather response of 1024 samples of 3 variables */

typedef struct
{
    DEV_HDR          devHdr;
    int              ctlr;
    int              openFlag;
    int              cancelFlag;
    int              polling;
    epicsRingBytesId replyQ;
    epicsEventId     ioReadmeId;
    epicsEventId     ioReceivedId;
    void             (*readMeISR)( void * );
} PMAC_DEV;

/* Function prototypes */

static int  pmacOpen(           PMAC_DEV *, char *, int );
static int  pmacClose(          PMAC_DEV *              );
static int  pmacRead(           PMAC_DEV *, char *, int );
static int  pmacIoctl(          PMAC_DEV *, int, int *  );
static int  pmacWriteAsc(       PMAC_DEV *, char *, int );
static int  pmacWriteMbx(       PMAC_DEV *, char *, int );
static void pmacAscReadMeISR(   PMAC_DEV *              );
static void pmacMbxReadMeISR(   PMAC_DEV *              );
static void pmacMbxReceivedISR( PMAC_DEV *              );

PMAC_DEV pmacAscDev[PMAC_MAX_CTLRS];
PMAC_DEV pmacMbxDev[PMAC_MAX_CTLRS];

IMPORT int       pmacVmeConfigLock;
IMPORT PMAC_CTLR pmacVmeCtlr[PMAC_MAX_CTLRS];

/* TRANSACTION_LOCK introduces a binary semaphore that is taken by the DPRAM
   write routine and given by the read ISR when it sees the end of transaction.
   terminator (ACK). This ensures that there is only one transaction going to
   one PMAC at a time. This makes communication more reliable if there is more
   than one PMAC in the VME chassis */

#define TRANSACTION_LOCK
#ifdef TRANSACTION_LOCK
static epicsEventId transactionLock = NULL;
#endif

/* This is the polling task for DPRAM ASCII communications.  This task is only
   created if the driver is requested to use polling and the default behaviour
   is to use the interrupt service routine.  This task is created in the pmacDrv
   function below.

   Added by Alan Greer on 12th August 2014. */

static void tpmac_poll_task_c(void *ptr)
{
  PMAC_DEV *pDev = (PMAC_DEV *)ptr;

  /* Run this loop forever, checking status as fast as possible */
  while(1)
  {
    /* Call the same routine that would be interrupted */
    pmacAscReadMeISR(pDev);
    /* Make sure other scheduled tasks are allowed to execute */
    taskDelay(0);
  }
}

/* This routine installs the DPRAM ASCII driver and the Mailbox ASCII driver.
   It adds a DRPAM ASCII device and a Mailbox ASCII device for every PMAC
   card that has been configured.
   This routine is called from "drvPmac_init" in "drvPmac.c"
   which in turn is called from EPICS "iocInit()" */

STATUS pmacDrv(int polling)
{
  STATUS ret;
  int    installedAsc;
  int    installedMbx;
  int    i;
  char   devNameAsc[32];
  char   devNameMbx[32];
  char   errorString[64];
  long   status;

  ret          = OK;
  installedAsc = FALSE;
  installedMbx = FALSE;

  /* For the DPRAM ASCII driver */

  /* check if driver already installed */

  if (pmacDrvNumAsc > 0)
  {
    installedAsc = TRUE;
    ret          = OK;
  }

  if( !installedAsc )
  {
#ifdef TRANSACTION_LOCK
    if (transactionLock == NULL) transactionLock = epicsEventMustCreate (epicsEventFull);
#endif
    pmacDrvNumAsc = iosDrvInstall( 0, 0, pmacOpen,  pmacClose, pmacRead, pmacWriteAsc, pmacIoctl );

    /* Add a DPRAM ASCII device for every configured card */
    for( i=0; i < PMAC_MAX_CTLRS; i++ )
    {
      pmacAscDev[i].ctlr       = pmacVmeCtlr[i].ctlr;
      pmacAscDev[i].openFlag   = 0;
      pmacAscDev[i].cancelFlag = 0;
      pmacAscDev[i].polling    = polling;

      if( pmacVmeCtlr[i].configured )
      {
        sprintf( devNameAsc, "/dev/pmac/%d/asc", pmacVmeCtlr[i].ctlr );
        ret = iosDevAdd( &pmacAscDev[i].devHdr, devNameAsc, pmacDrvNumAsc);
        if( ret == ERROR )
        {
          sprintf( errorString, "pmacDrv: Error adding: /dev/pmac/%d/asc device", pmacVmeCtlr[i].ctlr );
          cantProceed( errorString );
        }

        pmacAscDev[i].replyQ = epicsRingBytesCreate(replyQueueSize);
        if( !pmacAscDev[i].replyQ )
          cantProceed("pmacDrv: Failed to create ring buffer");

        pmacAscDev[i].ioReadmeId   = epicsEventMustCreate( epicsEventEmpty ); 
        pmacAscDev[i].ioReceivedId = 0;
        pmacAscDev[i].readMeISR    = (void *)pmacAscReadMeISR;

        if (polling == 0){
          status = devConnectInterruptVME( pmacVmeCtlr[i].irqVector + 1,
                                        (void *)pmacAscReadMeISR, (void *) &(pmacAscDev[i]) );
          if(!RTN_SUCCESS(status))
            cantProceed("pmacDrv: Failed to connect to DPRAM ASCII readme interrupt");
        } else {
          /* Create the thread that calls the interrupt service routine when interrupts are not available */
          status = (epicsThreadCreate("tpmac_poll_task",
                                      epicsThreadPriorityLow,
                                      epicsThreadGetStackSize(epicsThreadStackMedium),
                                      (EPICSTHREADFUNC)tpmac_poll_task_c,
                                      &(pmacAscDev[i])) == NULL);
          if (status){
            cantProceed("pmacDrv: Failed to create DPRAM ASCII polling task");
          }
        }
      }
    }
  }

  /* For the Mailbox ASCII driver */

  /* check if driver already installed */

  if(pmacDrvNumMbx > 0)
  {
    installedMbx = TRUE;
    ret          = OK;
  }

  if( !installedMbx )
  {
    pmacDrvNumMbx = iosDrvInstall( 0, 0, pmacOpen,  pmacClose, pmacRead, pmacWriteMbx, pmacIoctl );

    /* Add a Mailbox ASCII device for every configured card */
    for( i=0; i < PMAC_MAX_CTLRS; i++ )
    {
      pmacMbxDev[i].ctlr       = pmacVmeCtlr[i].ctlr;
      pmacMbxDev[i].openFlag   = 0;
      pmacMbxDev[i].cancelFlag = 0;
      pmacAscDev[i].polling    = polling;

      if ( pmacVmeCtlr[i].configured )
      {
        sprintf( devNameMbx, "/dev/pmac/%d/mbx", pmacVmeCtlr[i].ctlr );           
        ret = iosDevAdd( &pmacMbxDev[i].devHdr, devNameMbx, pmacDrvNumMbx );
        if( ret == ERROR ) 
        {
          sprintf( errorString, "pmacDrv: Error adding: /dev/pmac/%d/mbx device", pmacVmeCtlr[i].ctlr );
          cantProceed( errorString );
        }

        pmacMbxDev[i].replyQ = epicsRingBytesCreate(replyQueueSize);
        if( !pmacMbxDev[i].replyQ )
          cantProceed("pmacDrv: Failed to create ring buffer");

        pmacMbxDev[i].ioReadmeId   = epicsEventMustCreate( epicsEventEmpty ); 
        pmacMbxDev[i].ioReceivedId = epicsEventMustCreate( epicsEventEmpty );
        pmacMbxDev[i].readMeISR    = (void *)pmacMbxReadMeISR;

        status = devConnectInterruptVME( pmacVmeCtlr[i].irqVector,
                                      (void *)pmacMbxReadMeISR, (void *) &(pmacMbxDev[i]) );

        if( !RTN_SUCCESS(status) )
          cantProceed("pmacDrv: Failed to connect to Mailbox ASCII readme interrupt");

        status = devConnectInterruptVME( pmacVmeCtlr[i].irqVector - 1,
                                      (void *)pmacMbxReceivedISR, (void *) &(pmacMbxDev[i]) );

        /* Pre-enable responses to commands */
        /* pmacVmeCtlr[i].pBase->mailbox.MB[1].data = 0; */

       if( !RTN_SUCCESS(status) )
          cantProceed("pmacDrv: Failed to connect to Mailbox ASCII received interrupt");
      }
    }
  }
  return( ret );
}

/* The routines:                                                      */
/*   pmacOpen                                                         */
/*   pmacClose                                                        */
/*   pmacRead                                                         */
/*   pmacIoctl                                                        */
/*   are generic between the DPRAM ASCII and Mailbox ASCII interfaces */

static int pmacOpen( PMAC_DEV *pPmacDev, char * remainder, int mode )
{
  if( remainder[0] != 0 || pPmacDev->openFlag )
    return ERROR;
  else
    pPmacDev->openFlag = TRUE;

  return( (int)pPmacDev );
}


static int pmacClose( PMAC_DEV *pPmacDev )
{
  if( pPmacDev->openFlag )
    pPmacDev->openFlag = FALSE;

  return( OK );
}

int pmacNoInterruptCount=0;
int pmacDriverDebug=0;

static int pmacRead( PMAC_DEV *pPmacDev, char *buffer, int nBytes )
{
  int numRead;

  numRead = epicsRingBytesGet(pPmacDev->replyQ, buffer, nBytes);

  if( numRead == 0 )  /* The buffer was empty */
  {
      epicsEventWaitStatus status;

      /* Check to see if the semaphore was given */
      status = epicsEventWaitWithTimeout( pPmacDev->ioReadmeId, 0.2 );

      if (status == epicsEventWaitTimeout)
      {
          int irqLevel = pmacVmeCtlr[pPmacDev->ctlr].irqLevel;

	  pmacNoInterruptCount++;

          if ( pPmacDev->readMeISR ) {
              if (pmacDriverDebug)
                  logMsg( "Manually calling ISR for PMAC card %d, vector %d\n",
                          pPmacDev->ctlr, pmacVmeCtlr[pPmacDev->ctlr].irqVector,0,0,0,0 );

              devDisableInterruptLevel (intVME, irqLevel);
              pPmacDev->readMeISR( pPmacDev );
              devEnableInterruptLevel (intVME, irqLevel);
          }
          epicsEventWait( pPmacDev->ioReadmeId );
      }

      if( pPmacDev->cancelFlag )
      {
        pPmacDev->cancelFlag = FALSE;
      }
      else
      {
        numRead    = epicsRingBytesGet(pPmacDev->replyQ, buffer, nBytes);
      }
  }

  return( numRead );
}


static int pmacIoctl( PMAC_DEV *pPmacDev, int request, int *arg )
{
  int ret = 0;

  switch( request )
  {
    case FIONREAD:
      *arg = epicsRingBytesUsedBytes( pPmacDev->replyQ );
      break;

    case FIORFLUSH:
      epicsRingBytesFlush( pPmacDev->replyQ );
      break;

    case FIOCANCEL:
      pPmacDev->cancelFlag = TRUE;
      epicsEventSignal( pPmacDev->ioReadmeId );
      break;

    default:
      break;
  }
  return( ret );
}


/* The write and ISR routines are specific to the */
/* DPRAM ASCII and Mailbox ASCII interfaces       */

static int pmacWriteAsc( PMAC_DEV *pDev, char *buffer, int nBytes )
{
  int        i;
  int        ctlr;
  int        numWritten;
  static int totalWritten = 0;
  PMAC_DPRAM *dpramAsciiOut;
  PMAC_DPRAM *dpramAsciiOutControl;

  ctlr                 = pDev->ctlr;
  dpramAsciiOut        = pmacRamAddr(ctlr,0x0EA0);
  dpramAsciiOutControl = pmacRamAddr(ctlr,0x0E9C);
  i                    = 0;
  numWritten           = 0;

#ifdef TRANSACTION_LOCK
  epicsEventWaitWithTimeout( transactionLock, 0.3 );
#endif

  for( i=0; (i < nBytes) && (i < PMAC_BASE_ASC_REGS_OUT); i++ )
  {
    if( buffer[i] == '\r' )
    {
      /* Send command to PMAC */
      setReg( dpramAsciiOut[totalWritten], (char) 0 );
      setReg( *dpramAsciiOutControl, (char) 1 );
      totalWritten = 0;
    }
    else
    {
      if( totalWritten == 0 )
      {
        /* Line termination just sent - ensure PMAC is ready */
        int count = 0;
        const double delay = epicsThreadSleepQuantum();

        while( getReg( *dpramAsciiOutControl ) != 0x0 )
        {
          epicsThreadSleep(delay);
          count++;
          if( count > 10 )
            printf( "pmacWriteAsc: Stuck in while loop\n" );
        }
      }
      setReg( dpramAsciiOut[totalWritten], buffer[i] );
      totalWritten++;
    }
    numWritten++;
  }
  return(numWritten);
}


/* pmacAscReadMeISR - Interrupt Service Routine which is called when
                      PMAC issues an interrupt to tell us the DPRAM ASCII buffer
                      can be read.
                      Note: There is 1 interrupt per line of the response and
                      1 interrupt for the ACK at the end. */

static void pmacAscReadMeISR( PMAC_DEV *pDev )
{
  int         i;
  int         ctlr;
  int         pushOK;
  int         length;
  volatile epicsUInt16 *dpramAsciiInControl;
  PMAC_DPRAM  *dpramAsciiIn;
  union {epicsUInt16 S; char C[2];} control;
  ctlr                = pDev->ctlr;
  dpramAsciiInControl = (volatile epicsUInt16 *) pmacRamAddr(ctlr, 0x0F40);
  dpramAsciiIn        = pmacRamAddr(ctlr, 0x0F44);
  control.S           = getReg (*dpramAsciiInControl);

  if (control.S == 0)
  {
      if (pDev->polling == 0){
        logMsg( "No response from PMAC in pmacAscInISR\n", 0,0,0,0,0,0 );
      }
      return;
  }
  if (control.C[1] == 0 )
  {
    length = getReg( *pmacRamAddr(ctlr, 0x0F42) ) - 1;
    for( i=0; i<length; i++ )
    {
        char c = getReg(*dpramAsciiIn);
        pushOK = epicsRingBytesPut( pDev->replyQ, &c, 1);
        if( !pushOK ) logMsg("PMAC reply ring buffer full\n", 0,0,0,0,0,0);
        dpramAsciiIn++;
    }
    pushOK = epicsRingBytesPut( pDev->replyQ, &(control.C[0]), 1);
    if( !pushOK ) logMsg("PMAC reply ring buffer full\n", 0,0,0,0,0,0);
  }
  else
  {
    /* Build a "ERRnnn" string from the BCD error code in dpramAsciiInControl */
    char response[]={PMAC_TERM_BELL,'E','R','R','0','0','0',PMAC_TERM_CR,PMAC_TERM_ACK};

    /* Convert the BCD encoded error number to its ASCII equivalent */
    response[4] += ((control.C[1])       & 0xF );
    response[5] += ((control.C[0] >> 4 ) & 0xF );
    response[6] += ((control.C[0] )      & 0xF );

    /* Push the data the onto the ring buffer */
    pushOK = epicsRingBytesPut( pDev->replyQ, response, sizeof(response) );
    if( !pushOK )
      logMsg("PMAC reply ring buffer full\n", 0, 0, 0, 0, 0, 0);
  }

  setReg( *dpramAsciiInControl, (epicsUInt16) 0 );
  epicsEventSignal( pDev->ioReadmeId );

#ifdef TRANSACTION_LOCK
  if ( control.C[0] == PMAC_TERM_ACK ) epicsEventSignal( transactionLock );
#endif

  return;
}


static int pmacWriteMbx( PMAC_DEV *pPmacDev, char *buffer, int nBytes )
{
  int       i;
  int       j;
  int       numWritten;
  int       ctlr;
  char      firstChar, c;
  PMAC_CTLR *pPmacCtlr;

  j          = 0;
  numWritten = 0;
  ctlr       = pPmacDev->ctlr;
  pPmacCtlr  = &pmacVmeCtlr[ctlr];

  while( numWritten < nBytes )
  {
    firstChar = buffer[j];
    for( i = 1; i < PMAC_BASE_MBX_REGS_OUT; i++ )
    {
      if( PMAC_DRIVER_DEBUG )
        printf("pmacWriteMbx: 0x%x (%d)\n", buffer[j+i], i);
      pPmacCtlr->pBase->mailbox.MB[i+1].data = buffer[j+i];
      numWritten++;
      if( buffer[j+i] == PMAC_TERM_CR )
        break;
    }

    if( PMAC_DRIVER_DEBUG )
      printf("pmacWriteMbx: 0x%x (0)\n", firstChar);
    pPmacCtlr->pBase->mailbox.MB[0].data = firstChar;
    numWritten++;

    /* Now, enable the response, and read the data back to ensure it is written */
    pPmacCtlr->pBase->mailbox.MB[1].data = 0;
    c = pPmacCtlr->pBase->mailbox.MB[1].data;

    epicsEventWait( pPmacDev->ioReceivedId );
    j += PMAC_BASE_MBX_REGS_OUT;
  }

  return( numWritten );
}


/* We get this interrupt when PMAC has filled the Mailbox registers */
/* and we can then read the data                                    */

static void pmacMbxReadMeISR( PMAC_DEV *pPmacDev )
{
  int       i;
  int       ctlr;
  int       pushOK;
  int       sendMore;
  char	    c;
  PMAC_CTLR *pPmacCtlr;
  int        terminator=0;

  /* logMsg("Inside pmacMbxReadMeISR...\n", 0, 0, 0, 0, 0, 0); */

  ctlr      = pPmacDev->ctlr;
  pPmacCtlr = &pmacVmeCtlr[ctlr];
  sendMore  = 1;

  for( i = 0; i < PMAC_BASE_MBX_REGS_IN && !terminator; i++ )
  {
    c      = pPmacCtlr->pBase->mailbox.MB[i].data;

    pushOK = epicsRingBytesPut( pPmacDev->replyQ, &c, 1);
    if( !pushOK )
      logMsg("PMAC reply ring buffer full\n", 0, 0, 0, 0, 0, 0);

    terminator = ( (c == PMAC_TERM_CR) || (c == PMAC_TERM_ACK) || (c == PMAC_TERM_BELL) );
    if (terminator)
    {
        static int hadBell = 0;
        int bell = (c == PMAC_TERM_BELL);

        /* Add an ACK to the first terminator after a BELL to make parsing easier */
        if (hadBell) 
        {
            c = PMAC_TERM_ACK;
            pushOK = epicsRingBytesPut( pPmacDev->replyQ, &c, 1);
            if( !pushOK )
                logMsg("PMAC reply ring buffer full\n", 0, 0, 0, 0, 0, 0);
        }
        hadBell = bell;
    }
  }

  /* Write to mailbox register number 1 if there is more data in the response
     to this command. According to the manual we should be able to do this
     after receiving any response to pre-enable the next response, but in
     reality, this doesn't always work */

  if (c != PMAC_TERM_ACK) 
  {
      pPmacCtlr->pBase->mailbox.MB[1].data = 0;
      c = pPmacCtlr->pBase->mailbox.MB[1].data;
  }
  epicsEventSignal( pPmacDev->ioReadmeId );

  return;
}


/* We get this interrupt when PMAC has successfully received the data */
/* we have placed in the Mailbox registers                            */

static void pmacMbxReceivedISR( PMAC_DEV *pPmacDev )
{
  /* logMsg("Inside pmacMbxReceivedISR...\n", 0, 0, 0, 0, 0, 0); */

  epicsEventSignal( pPmacDev->ioReceivedId );
  return;
}


/* This is a test routine for testing Open/Close of multiple PMAC devices
   without having PMAC cards in the crate */

long pmacVmeConfigSim( int ctlrNumber, unsigned long addrBase, unsigned long addrDpram,
                       unsigned int irqVector, unsigned int irqLevel )
{
  PMAC_CTLR *pPmacCtlr;
	
  if( pmacVmeConfigLock != 0 )
  {
    printf( "pmacVmeConfigSim: Cannot change configuration after initialization\n" );
    return(ERROR);
  }
  	
  if( (ctlrNumber < 0) | (ctlrNumber >= PMAC_MAX_CTLRS) )
  {
    printf( "pmacVmeConfigSim: Controller number %d invalid -- must be 0 to %d.\n",
             ctlrNumber, PMAC_MAX_CTLRS-1 );
    return(ERROR);
  }
  
  if( pmacVmeCtlr[ctlrNumber].configured )
  {
    printf( "pmacVmeConfigSim: Controller %d already configured -- request ignored.\n",
            ctlrNumber );
    return(ERROR);
  }
	
  pPmacCtlr                = &pmacVmeCtlr[ctlrNumber];
  pPmacCtlr->ctlr          = ctlrNumber;
  pPmacCtlr->vmebusBase    = addrBase;
  pPmacCtlr->irqVector     = irqVector;
  pPmacCtlr->irqLevel      = irqLevel;
  pPmacCtlr->enabled       = FALSE;
  pPmacCtlr->present       = FALSE;
  pPmacCtlr->active        = FALSE;
  pPmacCtlr->enabledBase   = TRUE;
  pPmacCtlr->presentBase   = TRUE;
  pPmacCtlr->activeBase    = FALSE;
  pPmacCtlr->enabledDpram  = TRUE;
  pPmacCtlr->presentDpram  = TRUE;
  pPmacCtlr->activeDpram   = FALSE;
  pPmacCtlr->enabledGather = TRUE;
  pPmacCtlr->activeGather  = FALSE;
  pPmacCtlr->vmebusDpram   = addrDpram;
  if( addrDpram == 0 )
    pPmacCtlr->enabledDpram = FALSE;

  pPmacCtlr->present    = pPmacCtlr->presentBase | pPmacCtlr->presentDpram;
  pPmacCtlr->enabled    = pPmacCtlr->enabledBase | pPmacCtlr->enabledDpram;
  pPmacCtlr->configured = TRUE;
	
  return(0);
}


#define PMAC_ASYN
#ifdef PMAC_ASYN
#include "asynDriver.h"
#include "drvAsynSerialPort.h"
#include <epicsExport.h>
#include <iocsh.h>

int pmacAsynConfig( char * mbx_prefix, char * asc_prefix, unsigned int priority, int polling)
{
    int i;
    char devName[32];
    char asynName[32];
    static int installedAsynAsc = 0;
    static int installedAsynMbx = 0;

    pmacDrv(polling);

    if( !installedAsynMbx && mbx_prefix )
    {
        /* Add a DPRAM ASCII device for every configured card */
        for( i=0; i < PMAC_MAX_CTLRS; i++ )
        {
            if( pmacVmeCtlr[i].configured )
            {
                sprintf( devName,  "/dev/pmac/%d/mbx", pmacVmeCtlr[i].ctlr );
                sprintf( asynName, "%s%d", mbx_prefix, pmacVmeCtlr[i].ctlr );
                drvAsynSerialPortConfigure( asynName, devName, priority, 0, 0 );
            }
        }
        installedAsynMbx = 1;
    }

    if( !installedAsynAsc && asc_prefix )
    {
        /* Add a DPRAM ASCII device for every configured card */
        for( i=0; i < PMAC_MAX_CTLRS; i++ )
        {
            if( pmacVmeCtlr[i].configured )
            {
                sprintf( devName,  "/dev/pmac/%d/asc", pmacVmeCtlr[i].ctlr );
                sprintf( asynName, "%s%d", asc_prefix, pmacVmeCtlr[i].ctlr );
                drvAsynSerialPortConfigure( asynName, devName, priority, 0, 0 );
            }
        }
        installedAsynAsc = 1;
    }

    return 0;
}

static const iocshArg pmacAsynConfigArg0 = {"PMAC Mailbox Asyn port prefix",     iocshArgString};
static const iocshArg pmacAsynConfigArg1 = {"PMAC DPRAM ASCII Asyn port prefix", iocshArgString};
static const iocshArg pmacAsynConfigArg2 = {"Asyn port priority (0 for default)", iocshArgInt};
static const iocshArg pmacAsynConfigArg3 = {"Should the driver poll (0=no, 1=yes)", iocshArgInt};
static const iocshArg * const pmacAsynConfigArgs[] = {&pmacAsynConfigArg0, &pmacAsynConfigArg1, &pmacAsynConfigArg2, &pmacAsynConfigArg3};
 
static const iocshFuncDef pmacAsynConfigDef = {"pmacAsynConfig", 4, pmacAsynConfigArgs};

static void pmacAsynConfigCallFunc(const iocshArgBuf *args)
{
    pmacAsynConfig(args[0].sval, args[1].sval, args[2].ival, args[3].ival);
}


static void pmacAsynConfigRegister(void)
{
    iocshRegister(&pmacAsynConfigDef,  pmacAsynConfigCallFunc);
}

epicsExportRegistrar(pmacAsynConfigRegister);

#endif
