/* drvPmacEth.c -  EPICS Device Driver Support for UMAC Turbo
 * Author       Oleg A. Makarov
 * Date:        2012/04/13
 *
 *      Experimental Physics and Industrial Control System (EPICS)
 */

/*
*****************************************************************
                          COPYRIGHT NOTIFICATION
*****************************************************************

THE FOLLOWING IS A NOTICE OF COPYRIGHT, AVAILABILITY OF THE CODE,
AND DISCLAIMER WHICH MUST BE INCLUDED IN THE PROLOGUE OF THE CODE
AND IN ALL SOURCE LISTINGS OF THE CODE.

(C)  COPYRIGHT 2012 UNIVERSITY OF CHICAGO

Argonne National Laboratory (ANL), with facilities in the States of
Illinois and Idaho, is owned by the United States Government, and
operated by the University of Chicago under provision of a contract
with the Department of Energy.

Portions of this material resulted from work developed under a U.S.
Government contract and are subject to the following license:  For
a period of five years from March 30, 1993, the Government is
granted for itself and others acting on its behalf a paid-up,
nonexclusive, irrevocable worldwide license in this computer
software to reproduce, prepare derivative works, and perform
publicly and display publicly.  With the approval of DOE, this
period may be renewed for two additional five year periods.
Following the expiration of this period or periods, the Government
is granted for itself and others acting on its behalf, a paid-up,
nonexclusive, irrevocable worldwide license in this computer
software to reproduce, prepare derivative works, distribute copies
to the public, perform publicly and display publicly, and to permit
others to do so.

*****************************************************************
                                DISCLAIMER
*****************************************************************

NEITHER THE UNITED STATES GOVERNMENT NOR ANY AGENCY THEREOF, NOR
THE UNIVERSITY OF CHICAGO, NOR ANY OF THEIR EMPLOYEES OR OFFICERS,
MAKES ANY WARRANTY, EXPRESS OR IMPLIED, OR ASSUMES ANY LEGAL
LIABILITY OR RESPONSIBILITY FOR THE ACCURACY, COMPLETENESS, OR
USEFULNESS OF ANY INFORMATION, APPARATUS, PRODUCT, OR PROCESS
DISCLOSED, OR REPRESENTS THAT ITS USE WOULD NOT INFRINGE PRIVATELY
OWNED RIGHTS.

*****************************************************************
LICENSING INQUIRIES MAY BE DIRECTED TO THE INDUSTRIAL TECHNOLOGY
DEVELOPMENT CENTER AT ARGONNE NATIONAL LABORATORY (630-252-2000).
*/

/*
 * Modification History:
 * ---------------------
 * .01  04-13-2012        oam     initial
 */

/*
 * DESCRIPTION:
 * ------------
 * This module implements EPICS Device Driver Support for UMAC Turbo.
 *
 */

/*
 * INCLUDES
 */

#include <stdlib.h>
#include <stdio.h>
#include <string.h>   /* memcpy */
#include <pthread.h>
/*#include <signal.h>*/
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <sys/types.h>
#include <time.h>
#include <netdb.h>
#include <errno.h>

/* EPICS Includes */

#include <recSup.h>
#include <devLib.h>
#include <epicsExport.h>
#include <epicsTypes.h>
#include <epicsTime.h>
#include <epicsExit.h>
#include <epicsRingBytes.h>
#include <cantProceed.h>
#include <iocsh.h>
#include <dbLock.h>
#include <dbCommon.h>
#include <errlog.h>   /* errlogPrintf, errPrintf */
#include <epicsTime.h>
#include <taskwd.h>


/* local includes */
#include "drvPmacEth.h"
#include "pmacEthernet.h"

/*
 * DEFINES
 */
#define FOREVER	for (;;)
#define PMAC_DIAGNOSTICS TRUE
#if PMAC_DIAGNOSTICS
#define PMAC_MESSAGE	errlogPrintf
#define PMAC_DEBUG(level,code)	{ if (drvPmacDebug >= (level)) { code } }
#else
#define PMAC_DEBUG(level,code)      ;
#endif

#define PMAC_MEM_SIZE 0x7B31F
#define PMAC_DPRAM    0x60000

#define MAX_PMAC_CARDS 5
#define PMAC_MAX_MTR (256) /* 32 motors x 8 values = 256 */
#define	PMAC_MAX_BKG (314) /* 16 C.S. x 19 values + 10 global = 314 */
#define	PMAC_MAX_VAR (128) /* max allowed */
#define	PMAC_MAX_OPN (480) /* */
#define	PMAC_MAX_TIM (1)   /* */

#define DEVICE_COMMAND_QUEUE_SIZE 1000

#define PMAC_MTR_RATE (.05)
#define PMAC_BKG_RATE (.15)
#define PMAC_VAR_RATE (.15)
/*
#define TRUE 1
#define FALSE -1
*/

#define PMAC_DPRAM_MTR	 1
#define PMAC_DPRAM_BKG	 2
#define	PMAC_DPRAM_VAR	 3
#define PMAC_DPRAM_OPN	 4
#define PMAC_DPRAM_NONE	 (-1)

#define PMAC_MEMTYP_Y	 1
#define PMAC_MEMTYP_X	 2
#define PMAC_MEMTYP_SY	 3
#define PMAC_MEMTYP_SX	 4
#define PMAC_MEMTYP_DP	 5
#define PMAC_MEMTYP_D	 6
#define PMAC_MEMTYP_F	 7
#define PMAC_MEMTYP_L	 8
#define PMAC_MEMTYP_HY	 9
#define PMAC_MEMTYP_HX	 10
#define PMAC_MEMTYP_NONE (-1)

/* PMAC Hardware Constants */

#define PMAC_VARTYP_Y	 0x00
#define PMAC_VARTYP_L	 0x10
#define PMAC_VARTYP_X	 0x20
#define PMAC_VARTYP_NONE (-1)

/* #define DEBUG_PMAC_ETH 1 */
            

typedef struct { /* PMAC_CARD */
  int              card;
  int              configured;
  
  char             ipaddr [50];
  char             hostname [50];
  int              sock;
  fd_set           readfds;  
  fd_set           writefds;  
  
  int		   enabledMtr;
  int		   enabledBkg;
  int		   enabledVar;

  epicsRingBytesId MbxBuf;

  volatile double  scanMtrRate;
  volatile double  scanBkgRate;
  volatile double  scanVarRate;

  pthread_attr_t   procMbxAttr;

  pthread_t        procMbx;
  pthread_t        procMtr;
  pthread_t        procBkg;
  pthread_t        procVar;

  pthread_cond_t   procMbxState;
  pthread_mutex_t  mbxMutex;

  int	   	   numMtrIo;
  int	   	   numBkgIo;
  int	   	   numVarIo;
  int	   	   numOpnIo;
  int		   numtimVB;

  PMAC_RAM_IO	   MtrIo[PMAC_MAX_MTR];
  PMAC_RAM_IO	   BkgIo[PMAC_MAX_BKG];
  PMAC_RAM_IO	   VarIo[PMAC_MAX_VAR];
  PMAC_RAM_IO	   OpnIo[PMAC_MAX_OPN];
  PMAC_RAM_IO	   timVB[PMAC_MAX_TIM];

  struct waveformRecord *StrErr;		/* PV to report PMAC errors */

} PMAC_CARD;

/*
 * FORWARD DECLARATIONS
 */

PMAC_LOCAL long drvPmacRamGetData (PMAC_RAM_IO *pRamIo);
PMAC_LOCAL long drvPmacRamPutData (int card, PMAC_RAM_IO *pRamIo);

void            *mbxProcessTask (void *pcard);
void            *mtrProcessTask (void *pcard);
void            *bkgProcessTask (void *pcard);
void            *varProcessTask (void *pcard);

           int  drvPmacEthInit     ();
PMAC_LOCAL void drvPmacMtrScanInit (int	card);
PMAC_LOCAL void drvPmacBkgScanInit (int card);
PMAC_LOCAL void drvPmacVarScanInit (int card);

int             drvPmacGetResponse (int pmacCard, char *outstr, char *response);

void            drvPmacStrErr      (struct waveformRecord *pRec); 

/*
 * GLOBALS
 */

#if PMAC_DIAGNOSTICS
volatile int	drvPmacDebug = 0;		/* must be > 0 to see messages */
#endif

int num_pmac_cards = 0;
static int epics_terminate = 0;

PMAC_CARD pmacCards [MAX_PMAC_CARDS];

static void drvPmacCloseSockets (void *);


void *mbxProcessTask (void *pcard);

/*******************************************************************************
 *
 *drvPmacEthConfigure -	Configures the IP address of the PMAC-ETH
 *
 * This routine is to be called in the startup script in order to init the
 * driver options.
 *
 * By default there are no cards configured.
 *
 *	Returns: -1 on failure
 *			  1 on success
 */
int drvPmacEthConfig (int card, char *pmacAddress) {
  int i;
  PMAC_CARD *pCard;
  struct hostent *hp;
  
  if (num_pmac_cards == 0) {
    for (i = 0; i < MAX_PMAC_CARDS; i++) {
      pmacCards [i].configured = FALSE;
      pmacCards [i].card = i;
    }
  }

  if (card < 0 || card > MAX_PMAC_CARDS) {
    printf ("Card number out of range. Please specify a number between 0 and %d!\n",MAX_PMAC_CARDS);
    return -1;
  }
  
  if (pmacCards [card].configured == TRUE) {
    printf ("Card already configured!\n");
    return -1;
  }
  
  if ((hp = gethostbyname (pmacAddress)) == NULL) {
    printf ("The PMAC address you specified (%s) is invalid!\n", pmacAddress); 
    return -1;
  } else { /*deep copy hostent structure */
    strcpy (pmacCards [card].ipaddr, hp->h_addr);
    strcpy (pmacCards [card].hostname, hp->h_name);
  }
  
  pCard = &pmacCards [card];
  pCard->card = card;
  pCard->numMtrIo = 0;
  pCard->numBkgIo = 0;
  pCard->numVarIo = 0;
  pCard->numOpnIo = 0;
  pCard->numtimVB = 0;

  pCard->scanMtrRate = PMAC_MTR_RATE;
  pCard->scanBkgRate = PMAC_BKG_RATE;
  pCard->scanVarRate = PMAC_VAR_RATE;

  pCard->enabledMtr = TRUE;
  pCard->enabledBkg = TRUE;
  pCard->enabledVar = TRUE;

  pCard->StrErr = NULL;
  pCard->configured = TRUE;
  num_pmac_cards++;

  drvPmacEthInit ();
  pmacRamConfig (card);
  
  return 1;                    
}

       #define handle_error_en(en, msg) \
               do { errno = en; perror(msg); exit(EXIT_FAILURE); } while (0)

       static void display_sched_attr(int policy, struct sched_param *param) {
           printf("    policy=%s, priority=%d\n",
                   (policy == SCHED_FIFO)  ? "SCHED_FIFO" :
                   (policy == SCHED_RR)    ? "SCHED_RR" :
                   (policy == SCHED_OTHER) ? "SCHED_OTHER" :
                   "???",
                   param->sched_priority);
       }

       static void display_thread_sched_attr(char *msg) {
           int policy, s;
           struct sched_param param;

           s = pthread_getschedparam(pthread_self(), &policy, &param);
           if (s != 0) handle_error_en(s, "pthread_getschedparam");

           printf("%s\n", msg);
           display_sched_attr(policy, &param);
       }

/*
	Tries to reconnect the PMAC card. This function will block until it succeeds reconnecting
*/
void reconnectPmac (PMAC_CARD *pCard) {
  pCard->sock = -1;
  while (pCard->sock == -1) {
    pCard->sock = pmacSockOpen (pCard->ipaddr, pCard->hostname, &pCard->readfds, &pCard->writefds);
    if (pCard->sock == -1) sleep (PMAC_RECONNECT_TIME);
  }
  pmacSockFlush (pCard->sock, &pCard->readfds, &pCard->writefds);
}

/*
Sends a buffer containing multiple lines of program code to PMAC. Program lines should be seperated with
\0 (also the last line). Maximum buffer size is 1024 bytes. If there are more program lines, multiple calls 
to this function should be made. 
The function returns:
	 -1 on communication error
	 -2 on PMAC download error
	  1 on no error
	  
	  Also check comments for pmacSockWriteBuffer function in pmacEthernet.c
*/
int drvPmacSendBuffer (int pmacCard, char *buffer, int bufferLength) {
  PMAC_CARD *pCard = &pmacCards [pmacCard];
  int ethRsp;
  char response [4];

  if (pCard->configured == TRUE) {
    if (pCard->sock == -1) { /* reconnect if no socket*/
      pCard->sock = pmacSockOpen (pCard->ipaddr, pCard->hostname, &pCard->readfds, &pCard->writefds);
    }
  
    if (pCard->sock != -1) { /* communicate with pmac only if we've got an open socket*/
      ethRsp = pmacSockWriteBuffer (pCard->sock, &pCard->readfds, &pCard->writefds, buffer, bufferLength, response);
      if (ethRsp == -1) {
  	pmacSockClose (pCard->sock);
  	pCard->sock = -1;
  	printf ("Communication error \n");
  	return -1;
      } else {
  	if (response [3] == 0) { /* no error */
  	  return 1;
  	} else { /* error downloading */
  	  printf ("Error #%d during downloading line %d (consult PMAC software reference under variable I6). \n",
	    response [2], response [1]*256+ *response);
  	  return -2;
  	}
      }
    }	

  } 
  return -1;
}

/*******************************************************************************
 *
 * drvPmacStartup - startup PMAC driver
 *
 */
PMAC_LOCAL int drvPmacStartup (void) {
  char 	      *MyName = "drvPmacStartup";
  int	      i;

  static int status = 0;
  static int oneTimeOnly = 0;

  if (oneTimeOnly != 0) return status;

  oneTimeOnly = 1;

  for (i = 0; i < MAX_PMAC_CARDS; i++) {
    if (pmacCards [i].configured) {

      PMAC_DEBUG (1, PMAC_MESSAGE ("%s: Starting tasks for card %d.\n", MyName, i);)

      if (pmacCards [i].enabledMtr) drvPmacMtrScanInit (i);

      if (pmacCards [i].enabledBkg) drvPmacBkgScanInit (i);

      if (pmacCards [i].enabledVar) drvPmacVarScanInit (i);

    }
  }

  return status;
}

/*******************************************************************************
 *
 * drvPmacMemSpecParse - parse a PMAC address specification into values
 *
 * specification is of the form <memType>:$<hexAddr> where valid <memType>
 * strings are Y, X, SY, SX, DP, D, F, L, HY and HX and <hexAddr> must be
 * in the range 0000 -> PMAC_MEM_SIZE.
 *
 * For example, "SX:$8000" is memory type "SX" and Hex address 0x8000.
 *
 */
long drvPmacMemSpecParse (char *pmacAdrSpec, int *memType, int *pmacAdr) {
  char *MyName = "drvPmacMemSpecParse";
  int  parmCount;

  long status;
  char memTypeStr[11];

  parmCount = 0;
  *memTypeStr = '\0';
  *memType = PMAC_MEMTYP_NONE;
  *pmacAdr = 0;

  parmCount = sscanf (pmacAdrSpec, "%[^:]:$%x", memTypeStr, pmacAdr);

  PMAC_DEBUG (1,
    PMAC_MESSAGE ("%s: parse '%s' results parmCount %d\n", MyName, pmacAdrSpec, parmCount);
    PMAC_MESSAGE ("%s: memTypeStr '%c' pmacAdr %x\n", MyName, *memTypeStr, *pmacAdr);
  )

  if (parmCount != 2) {
    status = S_dev_badInit;
    errPrintf (status, __FILE__, __LINE__, "%s: Improper address specification '%s'.", MyName, pmacAdrSpec);
    return status;
  }

  if (strcmp (memTypeStr,"Y") == 0) {
    *memType = PMAC_MEMTYP_Y;
  } else if (strcmp (memTypeStr,"X") == 0) {
    *memType = PMAC_MEMTYP_X;
  } else if (strcmp (memTypeStr,"SY") == 0) {
    *memType = PMAC_MEMTYP_SY;
  } else if (strcmp (memTypeStr,"SX") == 0) {
    *memType = PMAC_MEMTYP_SX;
  } else if (strcmp (memTypeStr,"HY") == 0) {
    *memType = PMAC_MEMTYP_HY;
  } else if (strcmp (memTypeStr,"HX") == 0) {
    *memType = PMAC_MEMTYP_HX;
  } else if (strcmp (memTypeStr,"DP") == 0) {
    *memType = PMAC_MEMTYP_DP;
  } else if (strcmp (memTypeStr,"F") == 0) {
    *memType = PMAC_MEMTYP_F;
  } else if (strcmp (memTypeStr,"D") == 0) {
    *memType = PMAC_MEMTYP_D;
  } else if (strcmp (memTypeStr,"L") == 0) {
    *memType = PMAC_MEMTYP_L;
  } else {
    status = S_dev_badInit;
    errPrintf (status, __FILE__, __LINE__, "%s: Illegal address type '%s' for '%s'.", MyName, memTypeStr, pmacAdrSpec);
    return status;
  }

  PMAC_DEBUG (1, PMAC_MESSAGE ("%s: memType %d\n", MyName, *memType);)

  if ((*pmacAdr < 0) || (*pmacAdr > PMAC_MEM_SIZE)) {
    status = S_dev_badInit;
    errPrintf (status, __FILE__, __LINE__, "%s: Address %x out of range for '%s'.", MyName, (unsigned int)pmacAdr, pmacAdrSpec);
    return status;
  }

  return 0;

}

/*******************************************************************************
 *
 * drvPmacDpramRequest - add PMAC DPRAM address to scan list
 *
 */
long drvPmacDpramRequest (short card, short pmacAdrOfs, char *pmacAdrSpec, void	(*pFunc)(void *), void *pParm, PMAC_RAM_IO **ppRamIo) {
  char        *MyName = "drvPmacDpramRequest";
  int	      i;
  int         status;
  int	      hostOfs;

  PMAC_CARD   *pCard;
  PMAC_RAM_IO *pMtrIo;
  PMAC_RAM_IO *pBkgIo;
  PMAC_RAM_IO *pVarIo;
  PMAC_RAM_IO *pOpnIo;
  PMAC_RAM_IO *ptimVB;

  int	      memType;
  int	      pmacAdr;
  int         num, mask;

  pCard = &pmacCards [card];

  /* Parse PMAC Address Specification */
  status = drvPmacMemSpecParse (pmacAdrSpec, &memType, &pmacAdr);
  if (!RTN_SUCCESS(status)) {
    return status;
  }

  /* Add PMAC Address Offset */
  pmacAdr += pmacAdrOfs;
  hostOfs = 4 * (pmacAdr - PMAC_DPRAM);
  if (memType == PMAC_MEMTYP_HX) {
    hostOfs += 2;
  }


  /* If Motor Data Reporting Buffer */
  if ((pmacAdr >= PMAC_DPRAM + 0x1A) && (pmacAdr < PMAC_DPRAM + 0x19D)) {
    /* set motor mask registers */
    if (pmacAdr >= PMAC_DPRAM + 0x1D) {
      num = (pmacAdr-PMAC_DPRAM-0x1D)/0xC;
      pmacSockGetMem (pCard->sock, &pCard->readfds, &pCard->writefds, pmacRamAddr (card,0x70), 0x70, 4);
      if (num < 16) { 
	mask = 1 << num;
	pmacRamGet16 (pmacRamAddr (card,0x70), &status);
	status |= mask;
	pmacRamPut16 (pmacRamAddr (card,0x70), status);
      }
      else {
	mask = 1 << (num - 16);
	pmacRamGet16 (pmacRamAddr (card,0x72), &status);
	status |= mask;
	pmacRamPut16 (pmacRamAddr (card,0x72), status);
      }
      pmacSockSetMem (pCard->sock, &pCard->readfds, &pCard->writefds, pmacRamAddr (card,0x70), 0x70, 4);
    }
  
    i = pCard->numMtrIo;
    pMtrIo = &pCard->MtrIo [i];
    *ppRamIo = pMtrIo;

    pMtrIo->memType = memType;
    pMtrIo->pmacAdr = pmacAdr;
    pMtrIo->hostOfs = hostOfs;
    pMtrIo->pAddress = pmacRamAddr (card, hostOfs);
    pMtrIo->pFunc = pFunc;
    pMtrIo->pParm = pParm;

    PMAC_DEBUG (1, PMAC_MESSAGE ("%s: Mtr -- index %d memType %d hostOfs %x pAddress %#010x\n",
      MyName, i, memType, hostOfs, (unsigned int) pMtrIo->pAddress);)

    pCard->numMtrIo++;
    if (pCard->numMtrIo > PMAC_MAX_MTR) {
      printf("%s: too many records refer to MtrIo %d \n",MyName,pCard->numMtrIo);
      return -1;
    }
  }

  /* If Background Fixed Data Buffer */
  else if ((pmacAdr >= PMAC_DPRAM + 0x19E) && (pmacAdr < PMAC_DPRAM + 0x3A7)) {

    /* update max CS number */
    num = (pmacAdr-PMAC_DPRAM-0x1A7)/0x20;
    pmacSockGetMem (pCard->sock, &pCard->readfds, &pCard->writefds, pmacRamAddr (card,0x674), 0x674, 2);
    pmacRamGet16 (pmacRamAddr (card,0x674), &i);
    if (num >= (i & 0x1f)) {
      i = num + 1;
      pmacRamPut16 (pmacRamAddr (card,0x674), i);
      pmacSockSetMem (pCard->sock, &pCard->readfds, &pCard->writefds, pmacRamAddr (card,0x674), 0x674, 2);
    }
  
    i = pCard->numBkgIo;
    pBkgIo = &pCard->BkgIo [i];
    *ppRamIo = pBkgIo;

    pBkgIo->memType = memType;
    pBkgIo->pmacAdr = pmacAdr;
    pBkgIo->hostOfs = hostOfs;
    pBkgIo->pAddress = pmacRamAddr (card, hostOfs);
    pBkgIo->pFunc = pFunc;
    pBkgIo->pParm = pParm;

    PMAC_DEBUG (1, PMAC_MESSAGE ("%s: Bkg -- index %d memType %d hostOfs %x pAddress %#010lx\n",
      MyName, i, memType, hostOfs, (long unsigned int) pBkgIo->pAddress);)

    /* clear host bit */
    pmacSockSetBit (pCard->sock, &pCard->readfds, &pCard->writefds, 0x676, 0, 15);
    
    /* clear PMAC bit */
    pmacSockSetBit (pCard->sock, &pCard->readfds, &pCard->writefds, 0x67A, 0, 15);

    pCard->numBkgIo++;
    if (pCard->numBkgIo > PMAC_MAX_BKG) {
      printf("%s: too many records refer to BkgIo %d \n", MyName, pCard->numBkgIo);
      return -1;
    }
  }

  /* Background Variable Data Buffer -- Outside of DPRAM */
  else if ((pmacAdr < PMAC_DPRAM) || (pmacAdr > PMAC_DPRAM + 0x3FFF)) {
    i = pCard->numVarIo;
    pVarIo = &pCard->VarIo [i];
    *ppRamIo = pVarIo;
    pVarIo->memType = memType;
    pVarIo->pmacAdr = pmacAdr;
    pVarIo->hostOfs = 0;	    /* Unknown at this time */
    pVarIo->pAddress = pmacRamAddr (card, 0);
    pVarIo->pFunc = pFunc;
    pVarIo->pParm = pParm;

    PMAC_DEBUG (1, PMAC_MESSAGE ("%s: Var -- index %d memType %d\n", MyName, i, memType);)

    pCard->numVarIo++;
    if (pCard->numVarIo > PMAC_MAX_VAR) {
      printf("%s: too many records refer to VarIo %d \n",MyName,pCard->numVarIo);
      return -1;
    }
  }
  /* If Variable Buffer reporting time stamp */
  else if (pmacAdr == PMAC_DPRAM + 0x411) {
    i = pCard->numtimVB;
    ptimVB = &pCard->timVB [i];
    *ppRamIo = ptimVB;

    ptimVB->memType = memType;
    ptimVB->pmacAdr = pmacAdr;
    ptimVB->hostOfs = hostOfs;
    ptimVB->pAddress = pmacRamAddr (card, hostOfs);
    ptimVB->pFunc = pFunc;
    ptimVB->pParm = pParm;
  
    (*pFunc)(ptimVB->pParm); /* read VB time register once */
  

    PMAC_DEBUG (1,
      PMAC_MESSAGE ("%s: timVB -- index %d memType %d hostOfs %x pAddress %#010lx\n",
  	MyName, i, memType, hostOfs, (long unsigned int) ptimVB->pAddress);
    )

    pCard->numtimVB++;
    if (pCard->numtimVB > PMAC_MAX_TIM) {
      printf ("%s: too many records refer to timVB %d \n",MyName,pCard->numtimVB);
      return -1;
    }
  }

  /* If   Control Panel Function (does not yet existing) Or 
  	  Background Data reporting Buffer Control Or
  	  DPRAM ASCII Buffers Or
  	  Background Variable Read & Write Buffer Control Or
  	  Binary Rotary Program Buffer Control Or
  	  Data Gathering Control Or
  	  Variable-Sized Buffers / Open Use DPRAM */
  else if (((pmacAdr >= PMAC_DPRAM) && (pmacAdr < PMAC_DPRAM + 0x1A)) ||
  	   ((pmacAdr >= PMAC_DPRAM + 0x19D) && (pmacAdr < PMAC_DPRAM + 0x19F)) ||
  	   ((pmacAdr >= PMAC_DPRAM + 0x3A7) && (pmacAdr < PMAC_DPRAM + 0x1000))) {
    i = pCard->numOpnIo;
    pOpnIo = &pCard->OpnIo [i];
    *ppRamIo = pOpnIo;
    pOpnIo->memType = memType;
    pOpnIo->pmacAdr = pmacAdr;
    pOpnIo->hostOfs = hostOfs;
    pOpnIo->pAddress = pmacRamAddr (card, hostOfs);
    pOpnIo->pFunc = pFunc;
    pOpnIo->pParm = pParm;

    PMAC_DEBUG (1,
      PMAC_MESSAGE ("%s: Opn -- index %d memType %d hostOfs %x pAddress %#010lx\n",
  	MyName, i, memType, hostOfs, (long unsigned int) pOpnIo->pAddress);
    )

    pCard->numOpnIo++;
    if (pCard->numOpnIo > PMAC_MAX_OPN) {
      printf("%s: too many records refer to OpnIo %d \n",MyName,pCard->numOpnIo);
      return -1;
    }
  }
  else {
    status = S_dev_badRequest;
    errPrintf (status, __FILE__, __LINE__, "%s: Unsupported DPRAM address range %#06x.", MyName, pmacAdr);
    return status;
  }

  return 0;
}

/*******************************************************************************
 *
 * drvPmacVarSetup - enable operation of background variable data buffer
 *
 */
long drvPmacVarSetup (int card) {
  char        *MyName = "drvPmacVarSetup";
  int	      i;
  long        status;
  PMAC_CARD   *pCard = &pmacCards [card];
  PMAC_RAM_IO *pVarIo;
  int	      hostOfs;
  short	      configOfs;
  int         configFormat = PMAC_VARTYP_NONE;

  /* Start Config Table At PMAC_DPRAM + 0x540 */
  /* Maximum 128 addresses + 128 Data Words */
  /* End of data at PMAC_DPRAM + 0xFFF */

  /* Determine First Data Location */
  configOfs = 4 * 0x540;
  hostOfs = 4 * (0x540 + pCard->numVarIo);

  /* Set Size Of Buffer To Zero Before Changing Buffer Configuration */
  status = pmacRamPut16 (pmacRamAddr (card, 0x1048), 0);

  /* Configure Starting Address Offset Of Buffer 0x540 = 0x450 + 0xF0*/
  status = pmacRamPut16 (pmacRamAddr (card, 0x104A), 0xF0);

  pmacSockSetMem (pCard->sock, &pCard->readfds, &pCard->writefds, pmacRamAddr (card, 0x1048), 0x1048, 4);

  /* For Each Background Variable */
  for (i=0; i < pCard->numVarIo; i++) {
  /* Determine Location */
    pVarIo = &pCard->VarIo [i];
    pVarIo->hostOfs = hostOfs;
    pVarIo->pAddress = pmacRamAddr (card, hostOfs);

    /* Determine Memory Format */
    switch (pVarIo->memType) {
      case (PMAC_MEMTYP_Y) :
      case (PMAC_MEMTYP_SY) :
    	configFormat = PMAC_VARTYP_Y;
    	break;
      case (PMAC_MEMTYP_X) :
      case (PMAC_MEMTYP_SX) :
    	configFormat = PMAC_VARTYP_X;
    	break;
      case (PMAC_MEMTYP_D) :
      case (PMAC_MEMTYP_L) :
    	configFormat = PMAC_VARTYP_L;
    	break;
      default :
    	status = ERROR;
    	printf ("%s: Illegal address type '%d' at address 0x%x (index %d of %d).",
  	  MyName, pVarIo->memType,*pVarIo->pAddress,i,pCard->numVarIo);
    	return status;
    }

    /* Configure PMAC Address To Be Copied Into Buffer */
    status = pmacRamPut16 (pmacRamAddr (card, configOfs), pVarIo->pmacAdr);
    status = pmacRamPut16 (pmacRamAddr (card, configOfs + 2), configFormat);
    pmacSockSetMem (pCard->sock, &pCard->readfds, &pCard->writefds, pmacRamAddr (card, configOfs), configOfs, 4);

    PMAC_DEBUG (1,
      PMAC_MESSAGE ("%s: configFormat %d memType %d hostOfs %x pAddress %#010x\n",
    	MyName, configFormat, pVarIo->memType, pVarIo->hostOfs, (int) pVarIo->pAddress);
    )

    /* Determine Location of Next Variable */
    configOfs += 4; hostOfs += 4;
    if (configFormat == PMAC_VARTYP_L) hostOfs += 4;

  }

  /* Configure Size Of Buffer */
  status = pmacRamPut16 (pmacRamAddr (card,0x1048), pCard->numVarIo);
  pmacSockSetMem (pCard->sock, &pCard->readfds, &pCard->writefds, pmacRamAddr (card,0x1048), 0x1048, 2);

  /* Clear Data Ready Bit For Next Data Fill */
  pmacSockSetBit (pCard->sock, &pCard->readfds, &pCard->writefds, 0x1044, 0, 0);

  return 0;
}

/*******************************************************************************
 *
 * drvPmacMtrRead - read fixed motor data buffer
 *
 */
long drvPmacMtrRead (int card) {
  char        *MyName = "drvPmacMtrRead";
  int	      i;
  long        status;
  PMAC_CARD   *pCard = &pmacCards [card];

  /* 
  X:$06001A (0x006A) Motor Data Reporting Buffer (778 + 768 = 1546 bytes)
            (0x0374) 
  Y:$06019D (0x0674) Background Data Reporting Buffer
  */

  /* Read PMAC Motor Fixed Data Buffer */
  pmacSockGetMem (pCard->sock, &pCard->readfds, &pCard->writefds, pmacRamAddr (card,0x006A), 0x006A, 778);

  /* Check PMAC Busy Bit */
  status = pmacRamGet16 (pmacRamAddr (card, 0x06E), &i);

  PMAC_DEBUG (5, PMAC_MESSAGE ("%s: PMAC status 0x%x\n", MyName, i);)

  if ((i & 0x8000) == 0) return 0;  /* PMAC writing to DPRAM */

  /* Read PMAC Motor Fixed Data Buffer */
  pmacSockGetMem (pCard->sock, &pCard->readfds, &pCard->writefds, pmacRamAddr (card,0x0374), 0x0374, 768);

  /* Clear PMAC Ready Bit */
  pmacSockSetBit (pCard->sock, &pCard->readfds, &pCard->writefds, 0x006E, 0, 15);

  /* Read host copy of PMAC Motor Fixed Data Buffer */
  for (i=0; i < pCard->numMtrIo; i++) {
    status = drvPmacRamGetData (&pCard->MtrIo [i]);

    /* Notify Requester Of New Data */
    if (pCard->MtrIo [i].pFunc != NULL) (*pCard->MtrIo [i].pFunc) (pCard->MtrIo [i].pParm);
  }

  return 0;
}

/*******************************************************************************
 *
 * drvPmacBkgRead - read PMAC card background fixed data buffer
 *
 */
long drvPmacBkgRead (int card) {
  char        *MyName = "drvPmacBkgRead";
  int	      i;
  long        status;
  PMAC_CARD   *pCard = &pmacCards [card];

  /* 
  Y:$06019D (0x0674) Background Data Reporting Buffer (1064+1024 = 2088 bytes)
            (0x0A9C) 
  Y:$0603A7 (0x0E9C) DPRAM ASCII Command Buffer Control
  */

  /* Read PMAC Background Fixed Data Buffer 1-st portion of 1064 bytes */
  pmacSockGetMem (pCard->sock, &pCard->readfds, &pCard->writefds, pmacRamAddr (card,0x0674), 0x0674, 1064);

  /* Check for PMAC Data Ready */
  status = pmacRamGet16 (pmacRamAddr (card, 0x067A), &i);

  PMAC_DEBUG (5, PMAC_MESSAGE ("%s: PMAC status 0x%x\n", MyName, i);)

  /* If No Data Ready Then Return Without Reading */
  if ((i & 0x8000) == 0) return 0;

  /* Read PMAC Background Fixed Data Buffer 2-nd portion of 1024 bytes */
  pmacSockGetMem (pCard->sock, &pCard->readfds, &pCard->writefds, pmacRamAddr (card,0x0A9C), 0x0A9C, 1024);

  /* Clear Data Ready Bit For Next Data */
  pmacSockSetBit (pCard->sock, &pCard->readfds, &pCard->writefds, 0x067A, 0, 15);

  /* Read host copy of PMAC Background Fixed Data Buffer */
  for (i=0; i < pCard->numBkgIo; i++) {
    status = drvPmacRamGetData (&pCard->BkgIo [i]);
    /* Notify Requester Of New Data */
    if (pCard->BkgIo [i].pFunc != NULL) (*pCard->BkgIo [i].pFunc) (pCard->BkgIo [i].pParm);
  }

  return 0;
}

/*******************************************************************************
 *
 * drvPmacVarRead - read PMAC card background var data buffer
 */
long drvPmacVarRead (int card) {
  char        *MyName = "drvPmacVarRead";
  int	      i;
  long        status;
  PMAC_CARD   *pCard = &pmacCards [card];

/*
  Y:$060411 (0x1044) Background Variable-Read Control  
  Y:$060450 (0x1140) first bvr-buffer address (we'll use 1400 bytes only)
  X:$0605AE (0x16B8) first bvr-buffer address we are not reading 
  X:$060FFF (0x3FFE) last DPRAM address (Option 2:   8K x 16)
  X:$063FFF (0xFFFE) last DPRAM address (Option 2B: 32K x 16)
*/

  /* Check For PMAC Data Ready */
  pmacSockGetMem (pCard->sock, &pCard->readfds, &pCard->writefds, pmacRamAddr (card, 0x1044), 0x1044, 4);
  status = pmacRamGet16 (pmacRamAddr (card, 0x1044), &i);

  PMAC_DEBUG (5, PMAC_MESSAGE ("%s: PMAC status 0x%x\n", MyName, i);)

  /* If No Data Ready Then Return Without Reading */
  if ((i & 0x0001) != 1) return 0;

  /* Read PMAC Background Variable Data Buffer */
  pmacSockGetMem (pCard->sock, &pCard->readfds, &pCard->writefds, pmacRamAddr (card, 0x1140), 0x1140, 1400);

  /* Clear PMAC Data Ready Bit For Next Data */
  pmacSockSetBit (pCard->sock, &pCard->readfds, &pCard->writefds, 0x1044, 0, 0);

  for (i=0; i < pCard->numVarIo; i++) {
    /* Read host copy of PMAC Background Variable Data Buffer */
    status = drvPmacRamGetData (&pCard->VarIo [i]);
    /* Notify Requester Of New Data */
    if (pCard->VarIo [i].pFunc != NULL) (*pCard->VarIo [i].pFunc) (pCard->VarIo [i].pParm);
  }

  /* Read PMAC Background Variable Data Buffer time stamp*/
  for (i=0; i < pCard->numtimVB; i++) {
    status = drvPmacRamGetData (&pCard->timVB [i]);
    if (pCard->timVB [i].pFunc != NULL) (*pCard->timVB [i].pFunc) (pCard->timVB [i].pParm);
  }

  return 0;
}

/*******************************************************************************
 * drvPmacRamGetData - read data from PMAC DPRAM
 */
PMAC_LOCAL long drvPmacRamGetData (PMAC_RAM_IO *pRamIo) {
  /* char * MyName = "drvPmacRamGetData"; */
  long    status;
/*
  printf ("memType %d, pAddress 0x%x, hostOfs 0x%x, pmacAdr 0x%x, valInt %d, valDouble %f\n",
    pRamIo->memType, (unsigned int) pRamIo->pAddress, pRamIo->hostOfs, pRamIo->pmacAdr, pRamIo->valInt, pRamIo->valDouble);
*/
  switch (pRamIo->memType) {
    case (PMAC_MEMTYP_HY) :
    case (PMAC_MEMTYP_HX) :
      status = pmacRamGet16 (pRamIo->pAddress, &pRamIo->valInt);
      pRamIo->valDouble = (double) pRamIo->valInt;
      break;
    case (PMAC_MEMTYP_Y) :
    case (PMAC_MEMTYP_X) :
      status = pmacRamGet24U (pRamIo->pAddress, &pRamIo->valInt);
      pRamIo->valDouble = (double) pRamIo->valInt;
      break;
    case (PMAC_MEMTYP_SY) :
    case (PMAC_MEMTYP_SX) :
    case (PMAC_MEMTYP_DP) :
      status = pmacRamGet24 (pRamIo->pAddress, &pRamIo->valInt);
      pRamIo->valDouble = (double) pRamIo->valInt;
      break;
    case (PMAC_MEMTYP_F) :
      status = pmacRamGetF (pRamIo->pAddress, &pRamIo->valDouble);
      pRamIo->valInt = 0;
      break;
    case (PMAC_MEMTYP_D) :
      status = pmacRamGetD (pRamIo->pAddress, &pRamIo->valDouble);
      pRamIo->valInt = 0;
      break;
    case (PMAC_MEMTYP_L) :
      status = pmacRamGetL (pRamIo->pAddress, &pRamIo->valDouble);
      pRamIo->valInt = 0;
      break;
  }

  return 0;
}

/*******************************************************************************
 * drvPmacRamPutData - write data to PMAC DPRAM
 */
PMAC_LOCAL long drvPmacRamPutData (int card, PMAC_RAM_IO *pRamIo) {
  char *MyName = "drvPmacRamPutData";
  long status;
  PMAC_CARD *pCard = &pmacCards [card];

  switch (pRamIo->memType) {
    case (PMAC_MEMTYP_HY) :
    case (PMAC_MEMTYP_HX) :
      status = pmacRamPut16 (pRamIo->pAddress, pRamIo->valInt);
      pmacSockSetMem (pCard->sock, &pCard->readfds, &pCard->writefds, (char *) pRamIo->pAddress, pRamIo->hostOfs, 2);
      break;
    case (PMAC_MEMTYP_Y) :
    case (PMAC_MEMTYP_X) :
    case (PMAC_MEMTYP_SY) :
    case (PMAC_MEMTYP_SX) :
    case (PMAC_MEMTYP_DP) :
      status = pmacRamPut32 (pRamIo->pAddress, pRamIo->valInt);
      pmacSockSetMem (pCard->sock, &pCard->readfds, &pCard->writefds, (char *) pRamIo->pAddress, pRamIo->hostOfs, 4);
      break;
    case (PMAC_MEMTYP_F) :
      status = pmacRamPutF (pRamIo->pAddress, pRamIo->valDouble);
      pmacSockSetMem (pCard->sock, &pCard->readfds, &pCard->writefds, (char *) pRamIo->pAddress, pRamIo->hostOfs, 4);
      break;
    default:
      printf ("%s: ERR! pmacAdr 0x%x, hostOfs 0x%x, memType %d, valInt %d, valDouble %f \n",
        MyName, pRamIo->pmacAdr, pRamIo->hostOfs, pRamIo->memType, pRamIo->valInt, pRamIo->valDouble);
  }
  return 0;
}

/*******************************************************************************
 *
 * drvPmacGetResponse - write command and read response in PMAC mailbox
 * The command should end with \0.
 * The function returns:
 * 	 -1 on communication errorr
 * 	  number of recieved characters on no error
 * 
 * 	  Also check comments for pmacSockGetResponse function in pmacEthernet.c
 */
int drvPmacGetResponse (int pmacCard, char *outstr, char *response) {
  int numchar;
  PMAC_CARD *pCard = &pmacCards [pmacCard];

  if (pCard->configured == TRUE) {
    if (pCard->sock == -1) { /* reconnect if no socket */
      pCard->sock = pmacSockOpen (pCard->ipaddr, pCard->hostname, &pCard->readfds, &pCard->writefds);
    }
    if (pCard->sock != -1) { /* communicate with pmac only if we've got an open socket */
      if ((numchar = pmacSockGetResponse (pCard->sock, &pCard->readfds, &pCard->writefds, outstr, response)) == -1) {
  	pmacSockClose (pCard->sock);
  	printf ("pmacSockGetResponse error!\n");
      }
      return numchar;
    }
  }
  return -1;
}

/*******************************************************************************
 *
 * drvPmacMbxScan - put PMAC MBX request on queue
 *
 */
int drvPmacMbxScan (PMAC_MBX_IO *pMbxIo) {
  char      *MyName = "drvPmacMbxScan";
  PMAC_CARD *pCard = &pmacCards [pMbxIo->card];
  if (pCard->configured == TRUE) { 
    pthread_mutex_lock (&pCard->mbxMutex);
    if (epicsRingBytesPut (pCard->MbxBuf, (void *) &pMbxIo, sizeof (pMbxIo)) != 0) {
      pthread_mutex_unlock (&pCard->mbxMutex); 
      pthread_cond_signal (&pCard->procMbxState);
    } else {
      pthread_mutex_unlock (&pCard->mbxMutex); 
      printf ("%s: PMAC card %d, request queue full!\n", MyName, pMbxIo->card);
    }
    return 0;
  } else {
    /*printf ("PMAC card number %d is not configured!\n", pMbxIo->card);*/
    pMbxIo->status = -1;
    return -1;
  }
}

/*******************************************************************************
 *
 * drvPmacMbxScanInit - initialize PMAC MBX scan task
 *
	Initializes the PMAC ethernet driver. This function should be called only once, after all the 
	PMAC cards have been configured with drvPmacEthConfigure
*/
int drvPmacEthInit () {
  int s, card;
  char responseBuffer [1400];
  PMAC_CARD *pCard;
  static int oneTimeOnly = 0;
  struct sched_param param;
  pthread_attr_t attr;


  epicsAtExit (drvPmacCloseSockets, NULL);

  if (oneTimeOnly != 0) return 1;
  oneTimeOnly = -1;  
  

  for (card = 0; card < MAX_PMAC_CARDS; card++) {
    pCard = &pmacCards [card];
    if (pCard->configured == TRUE) {
      #ifdef DEBUG_PMAC_ETH
        printf ("Initializing PMAC card %d\n", card);
      #endif        
      s = pthread_attr_init(&attr);
      if (s != 0) handle_error_en(s, "pthread_attr_init");
      s = pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);
      if (s != 0) handle_error_en(s, "pthread_attr_setinheritsched");
      s = pthread_attr_setschedpolicy(&attr, SCHED_FIFO);
      if (s != 0) handle_error_en(s, "pthread_attr_setschedpolicy");
      param.sched_priority = sched_get_priority_max(SCHED_FIFO);
      s = pthread_attr_setschedparam(&attr, &param);
      if (s != 0) handle_error_en(s, "pthread_attr_setschedparam");

      pCard->sock = pmacSockOpen (pCard->ipaddr, pCard->hostname, &pCard->readfds, &pCard->writefds);
      if (pCard->sock == -1) return -1;
      pmacSockFlush (pCard->sock, &pCard->readfds, &pCard->writefds);
      if (pmacSockGetResponse (pCard->sock, &pCard->readfds, &pCard->writefds, "I6=1\0", responseBuffer) == -1) {
        printf ("Error while communicating (I6=1) with PMAC!\n");
        pCard->configured = FALSE;
        break;
      }
      if (pmacSockGetResponse (pCard->sock, &pCard->readfds, &pCard->writefds, "I3=2\0", responseBuffer) == -1) {
        printf ("Error while communicating (I3=2) with PMAC!\n");
        pCard->configured = FALSE;
        break;
      }
      pCard->MbxBuf = epicsRingBytesCreate (sizeof (void *) * DEVICE_COMMAND_QUEUE_SIZE);
      pthread_cond_init (&pCard->procMbxState, NULL);
      pthread_mutex_init (&pCard->mbxMutex, NULL);
      pthread_create (&pCard->procMbx, &attr, mbxProcessTask, &pCard->card);
      s = pthread_attr_destroy(&attr);
      if (s != 0) handle_error_en(s, "pthread_attr_destroy");
    }
  }
  return 1;
}

/*******************************************************************************
 *
 * mbxProcessTask - task for PMAC MBX input/output
 * Thread for mailbox requests for one PMAC card.
 */
void *mbxProcessTask (void *pcard) {
  char            *MyName = "mbxProcessTask";
  PMAC_CARD       *pCard = pcard;
  PMAC_MBX_IO     *pMbxIo = NULL;
  struct dbCommon *pRec = NULL;
  int             ethRsp;
  char            responseBuffer [1400];
  int             i;
  epicsTimeStamp  err_time;
  char  	  time_str [40];

  display_thread_sched_attr("Scheduler attributes of mbxProcessTask");

  FOREVER {
    if (epics_terminate == 1) break;
    pthread_mutex_lock (&pCard->mbxMutex);
    if (epicsRingBytesIsEmpty (pCard->MbxBuf)) pthread_cond_wait (&pCard->procMbxState, &pCard->mbxMutex);
    /* Get an I/O request from the queue */
    if (epicsRingBytesGet (pCard->MbxBuf, (void *)&pMbxIo, sizeof (pMbxIo)) != -1) {
      pthread_mutex_unlock (&pCard->mbxMutex);

      /* reconnect if no socket*/
      if (pCard->sock == -1) pCard->sock = pmacSockOpen (pCard->ipaddr, pCard->hostname, &pCard->readfds, &pCard->writefds);

      /* communicate with pmac only if we've got an open socket*/
      ethRsp = -1;
      if (pCard->sock != -1) ethRsp = pmacSockGetResponse (pCard->sock, &pCard->readfds, &pCard->writefds, pMbxIo->command, responseBuffer);
        	      
      if (ethRsp > 0) { /* valid response from PMAC */

        for (i = 0; i < strlen (pMbxIo->command); i++) {
          if (pMbxIo->command [i] == 'k' || pMbxIo->command [i] == 'K') { /* Print kill commands sent */
            printf ("Kill Command \"%s\" sent to \"%s\"\n", pMbxIo->command, pCard->hostname);
            break;
          }
        }

    	if (*(responseBuffer + ethRsp - 1) == PMAC_TERM_ACK) {
    	  pMbxIo->status = 1; /* ACK */
    	  if (ethRsp > 1 && *(responseBuffer + ethRsp - 2) == PMAC_TERM_CR) {
    	    memcpy (pMbxIo->response, responseBuffer, ethRsp - 1);
    	    *(pMbxIo->response + ethRsp - 1) = 0; /*  replace ACK with \0 */
    	    *(pMbxIo->response + ethRsp - 2) = 0; /* remove last \n */
    	  }

    	  /* DEBUG ONLY*/
    	  #ifdef DEBUG_PMAC_ETH
    	    if (ethRsp > 1) *(pMbxIo->response + ethRsp - 2) = 0;
    	    else *(pMbxIo->response) = 0;
    	    printf ("Executed '%s'. Response was '%s'. %d\n", pMbxIo->command, pMbxIo->response, ethRsp);
    	  #endif
    	} else if (*responseBuffer == PMAC_TERM_BELL) {
	  epicsTimeGetCurrent (&err_time);
	  callbackGetUser (pRec, &pMbxIo->callback);
    	  pMbxIo->status = -2; /* PMAC REPORTED AN ERROR*/
    	  memcpy (pMbxIo->errmsg, responseBuffer + 1, 6);
    	  *(pMbxIo->errmsg + 6) = 0;

	  if (pCard->StrErr) {    /* report error through the StrErr waveform record */
	    epicsTimeToStrftime(time_str,28,"%m/%d/%y %H:%M:%S.%09f",&err_time);
	    dbScanLock ((struct dbCommon *)pCard->StrErr);
	    sprintf (pCard->StrErr->bptr, "%17.17s %s CMD=%s, %s \"%s\"",
	      time_str, pRec->name, pMbxIo->command, pMbxIo->errmsg, pmacError (pMbxIo->errmsg));
            printf ("%s: %s:.\n", MyName, (char *) pCard->StrErr->bptr);
	    pCard->StrErr->nord = strlen (pCard->StrErr->bptr);
	    pCard->StrErr->pact = TRUE;
	    pCard->StrErr->putf = TRUE;
	    (*(pCard->StrErr->rset->process)) ((struct rset*) pCard->StrErr);
	    dbScanUnlock ((struct dbCommon *)pCard->StrErr);
	  } else printf ("PMAC reported an Error (%s) while executing '%s'!\n", pMbxIo->errmsg, pMbxIo->command);
    	}

	/*Print kill commands sent*/
      } else if (ethRsp == 0) { /* NOTHING RETURNED AND NO COMM. ERROR  (can this happen??) */
    	pMbxIo->status = -1; 
      } else { /* some error in communication */
        pmacSockClose (pCard->sock); pCard->sock = -1; pMbxIo->status = -1; 
	printf ("Communication error while executing command: %s!\n", pMbxIo->command);
      }   
      callbackRequest (&pMbxIo->callback);
    } else  pthread_mutex_unlock (&pCard->mbxMutex);
  }
  return pcard;
}

/*******************************************************************************
 *
 * mtrProcessTask - perform motor fixed buffer scanning
 *
 */
void *mtrProcessTask (void *pcard) {
  /*char      *MyName = "mtrProcessTask";*/
  PMAC_CARD *pCard = pcard;
  double    scanMtrRate = pCard->scanMtrRate;

  display_thread_sched_attr("Scheduler attributes of mtrProcessTask");

  FOREVER {
    if (pCard->enabledMtr) drvPmacMtrRead (pCard->card);
    if (epics_terminate == 1) break;
    epicsThreadSleep (scanMtrRate);
  }
  return NULL;
}

/*******************************************************************************
 *
 * drvPmacMtrScanInit - spawn task to perform scanning of the motor fixed buffer
 *
 */
PMAC_LOCAL void drvPmacMtrScanInit (int	card) {
  /* char   *MyName = "drvPmacMtrScanInit"; */
  /* long status; */
  int s;
  struct sched_param param;
  pthread_attr_t attr;

  s = pthread_attr_init(&attr);
  if (s != 0) handle_error_en(s, "pthread_attr_init");
  s = pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);
  if (s != 0) handle_error_en(s, "pthread_attr_setinheritsched");
  s = pthread_attr_setschedpolicy(&attr, SCHED_RR);
  if (s != 0) handle_error_en(s, "pthread_attr_setschedpolicy");
  param.sched_priority = 20;
  s = pthread_attr_setschedparam(&attr, &param);
  if (s != 0) handle_error_en(s, "pthread_attr_setschedparam");

  PMAC_CARD *pCard = &pmacCards [card];

  pthread_create (&pCard->procMtr, &attr, mtrProcessTask, &pCard->card);
  s = pthread_attr_destroy(&attr);
  if (s != 0) handle_error_en(s, "pthread_attr_destroy");

  return;
}

/*******************************************************************************
 *
 * bkgProcessTask - perform background scanning
 *
 */
void *bkgProcessTask (void *pcard) {
  /* char   *MyName = "bkgProcessTask"; */
  PMAC_CARD *pCard = pcard;
  double scanBkgRate = pCard->scanBkgRate;
  
  display_thread_sched_attr("Scheduler attributes of bkgProcessTask");

  FOREVER {
    epicsThreadSleep (scanBkgRate);
    if (pCard->enabledBkg) drvPmacBkgRead (pCard->card);
    if (epics_terminate == 1) break;
  }
  return NULL;
}

/*******************************************************************************
 *
 * drvPmacBkgScanInit - spawn task to perform background scanning
 *
 */
PMAC_LOCAL void drvPmacBkgScanInit (int card) {
  /* char   *MyName = "drvPmacBkgScanInit"; */
  /* long   status; */
  int s;
  struct sched_param param;
  pthread_attr_t attr;

  s = pthread_attr_init(&attr);
  if (s != 0) handle_error_en(s, "pthread_attr_init");
  s = pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);
  if (s != 0) handle_error_en(s, "pthread_attr_setinheritsched");
  s = pthread_attr_setschedpolicy(&attr, SCHED_RR);
  if (s != 0) handle_error_en(s, "pthread_attr_setschedpolicy");
  param.sched_priority = 19;
  s = pthread_attr_setschedparam(&attr, &param);
  if (s != 0) handle_error_en(s, "pthread_attr_setschedparam");

  PMAC_CARD *pCard = &pmacCards [card];

  pthread_create (&pCard->procBkg, &attr, bkgProcessTask, &pCard->card); 
  s = pthread_attr_destroy(&attr);
  if (s != 0) handle_error_en(s, "pthread_attr_destroy");

  return;
}

/*******************************************************************************
 *
 * varProcessTask - perform background variable scanning
 *
 */
void *varProcessTask (void *pcard) {
  /* char   *MyName = "varProcessTask"; */
  PMAC_CARD *pCard = pcard;
  double scanVarRate = pCard->scanVarRate;

  display_thread_sched_attr("Scheduler attributes of varProcessTask");

  FOREVER {
    epicsThreadSleep (scanVarRate);
    if (pCard->enabledVar) drvPmacVarRead (pCard->card);
    if (epics_terminate == 1) break;
  }
  return NULL;
}

/*******************************************************************************
 *
 * drvPmacVarScanInit - spawn task to perform background scanning
 *
 */
PMAC_LOCAL void drvPmacVarScanInit (int card) {
  /*char   *MyName = "drvPmacVarScanInit";*/
  long      status;
  int s;
  struct sched_param param;
  pthread_attr_t attr;

  s = pthread_attr_init(&attr);
  if (s != 0) handle_error_en(s, "pthread_attr_init");
  s = pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);
  if (s != 0) handle_error_en(s, "pthread_attr_setinheritsched");
  s = pthread_attr_setschedpolicy(&attr, SCHED_RR);
  if (s != 0) handle_error_en(s, "pthread_attr_setschedpolicy");
  param.sched_priority = 18;
  s = pthread_attr_setschedparam(&attr, &param);
  if (s != 0) handle_error_en(s, "pthread_attr_setschedparam");

  PMAC_CARD *pCard = &pmacCards [card];

  status = drvPmacVarSetup (card);

  pthread_create (&pCard->procVar, &attr, varProcessTask, &pCard->card); 
  s = pthread_attr_destroy(&attr);
  if (s != 0) handle_error_en(s, "pthread_attr_destroy");

  return;
}

/*
	drvPmacStrErr - set StrErr pointer in the pmacCards [card] structure,
	                called from devPmacMbx.c
*/
void drvPmacStrErr (struct waveformRecord *pRec) {
  PMAC_CARD *pCard = &pmacCards [pRec->inp.value.vmeio.card];
  pCard->StrErr = pRec;
  return;
}

/* Closes sockets to all Pmac cards on exit */
static void drvPmacCloseSockets (void *arg) {
  int card, status;
  PMAC_CARD *pCard;

  epics_terminate = 1;
  epicsThreadSleep (1.0);
  for (card = 0; card < MAX_PMAC_CARDS; card++) {
    pCard = &pmacCards [card];
    if (pCard->configured == TRUE) {
      status = pthread_cancel(pCard->procMbx);
      if (pCard->sock != -1) {
    	pmacSockClose (pCard->sock);
    	printf ("Socket Closed to Pmac %d\n", card);
      }
      epicsRingBytesDelete (pCard->MbxBuf);
      printf ("Terminating EPICS driver for Pmac %d \n", card);
    }
  }

}


/*register drvPmacEthConfigure */
static const iocshArg drvPmacEthConfigureArg0 = {"card", iocshArgInt};
static const iocshArg drvPmacEthConfigureArg1 = {"pmacAddress", iocshArgString};
static const iocshArg *drvPmacEthConfigureArgs [] = {&drvPmacEthConfigureArg0, &drvPmacEthConfigureArg1};
static const iocshFuncDef drvPmacEthConfigureFuncDef = {"drvPmacEthConfig", 2, drvPmacEthConfigureArgs};
static void drvPmacEthConfigureCallFunc (const iocshArgBuf *args) {
  drvPmacEthConfig (args [0].ival, args [1].sval);
}

static void drvPmacEthConfigureRegister (void) {
  static int firstTime = 1;
  if (firstTime) {
    firstTime = 0;
    iocshRegister (&drvPmacEthConfigureFuncDef, drvPmacEthConfigureCallFunc);
  }
}
epicsExportRegistrar (drvPmacEthConfigureRegister);
