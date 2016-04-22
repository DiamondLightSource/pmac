/* pmacRam.c -  EPICS Device Driver Library for Turbo PMAC2-VME Ultralite
 *
 * Author       Oleg A. Makarov
 *              Thomas A. Coleman's PMAC-VME driver library was used as a prototype 
 * Date:        2003/08/19
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
 
(C)  COPYRIGHT 1995 UNIVERSITY OF CHICAGO
 
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
 * .01  6-7-95        tac     initial
 * .02  8-19-03       oam     Turbo PMAC2-VME Ultralite initial
 * .03  26th May 2006 ajf     Add DPRAM ASCII semaphore and ISR.
 *                            Comment out vxMemProbe of mailbox registers
 *                            as this causes an MVME-5500 to hang.
 * .04  28-Feb-2012   oam     Turbo PMAC2-ETH
 */

/*
 * DESCRIPTION:
 * ------------
 * This module drives PMAC-VME.
 *
 */

/*
 * INCLUDES
 */

/* VxWorks Includes */
#ifdef vxWorks
 #include <vxWorks.h>
 #include <vxLib.h>
 #include <sysLib.h>
 #include <taskLib.h>
 #include <iv.h>
 #include <logLib.h>      /* Sergey */
#endif
 
#include <math.h>
#include <stdio.h>	 /* Sergey */
#include <string.h>	 /* Sergey */
#include <pthread.h>
#define __PROTOTYPE_5_0         /* Sergey */

/* EPICS Includes */

#include <devLib.h>
#include <errMdef.h>
#include <errlog.h>   /* errlogPrintf, errPrintf */

#include <pmacRam.h>

/*
 * DEFINES
 */
#ifndef vxWorks
#define STATUS int
#define S_objLib_OBJ_TIMEOUT -1
#endif

#define PMAC_DIAGNOSTICS TRUE
#define PMAC_PRIVATE FALSE

#if PMAC_PRIVATE
#define PMAC_LOCAL LOCAL
#else
#define PMAC_LOCAL
#endif

#if PMAC_DIAGNOSTICS
#define PMAC_MESSAGE	errlogPrintf
#define PMAC_DEBUG(level,code)       { if (pmacRamDebug >= (level)) { code } }
#else
#define PMAC_DEBUG(level,code)      ;
#endif

#define NO_ERR_STATUS	(-1)

#define PMAC_BUFSIZE	(80)

#define PMAC_MEM_SIZE_DPRAM	(0x1000)	/* Size of DPRAM */

#define BYTESWAP(x) (MSB(x) | (LSB(x) << 8))

/*
 * GLOBALS
 */

char * pmacRamVersion = "@(#) pmacRam.c 1.7 2012/02/28";

#if PMAC_DIAGNOSTICS
volatile int	pmacRamDebug = 0;		/* must be > 0 to see messages */
#endif

/*
 * LOCALS
 */

           int       pmacRamConfigLock = 0;
PMAC_LOCAL int       pmacRamNumCtlrs = 0;
PMAC_LOCAL PMAC_CTLR pmacRamCtlr[PMAC_MAX_CTLRS];

void pmacRamReport (int card, int level) {
  PMAC_CTLR *pCtlr = &pmacRamCtlr[card];

  printf ("    hostDpram = %p\n", pCtlr->pDpramBase);
}

/*******************************************************************************
 *
 * pmacRamConfig - Configure PMAC-VME Controller Addresses and Interrupts
 *
 * This routine is to be called in the startup script in order to init the
 * controller addresses and the associated IRQ vectors and levels.
 *
 * By default there are no controllers configured.
 *
 */
long pmacRamConfig ( int ctlrNumber) {
  char          *MyName = "pmacRamConfig";
  int		i;
  PMAC_CTLR 	*pPmacCtlr;

  if (pmacRamConfigLock != 0) {
    printf ( "%s: Cannot change configuration after initialization -- request ignored.\n", MyName);
    return ERROR;
  }
  
  if (pmacRamNumCtlrs == 0) {
    for (i=0; i < PMAC_MAX_CTLRS; i++ ) {pmacRamCtlr[i].configured = FALSE;}
  }

  if ((ctlrNumber < 0) | (ctlrNumber >= PMAC_MAX_CTLRS)) {
    printf ("%s: Controller number %d invalid -- must be 0 to %d.\n",
      MyName, ctlrNumber, PMAC_MAX_CTLRS - 1);
    return ERROR;
  }
  
  if (pmacRamCtlr[ctlrNumber].configured) {
    printf ("%s: Controller %d already configured -- request ignored.\n", MyName, ctlrNumber);
    return ERROR;
  }

  PMAC_DEBUG (1, printf ("%s: Initializing controller %d.\n", MyName, ctlrNumber);)

  pPmacCtlr = &pmacRamCtlr[ctlrNumber];
  pPmacCtlr->ctlr = ctlrNumber;

  pPmacCtlr->enabledDpram = TRUE;
  pPmacCtlr->presentDpram = TRUE;
  pPmacCtlr->activeDpram = FALSE;
  pPmacCtlr->enabledGather = TRUE;
  pPmacCtlr->activeGather = FALSE;


  if (pPmacCtlr->enabledDpram) {
    pPmacCtlr->pDpramBase = (PMAC_DPRAM *) malloc (4 * PMAC_MEM_SIZE_DPRAM);
    pPmacCtlr->presentDpram = TRUE;
  }

  pPmacCtlr->configured = TRUE;
  pmacRamNumCtlrs++;

  PMAC_DEBUG (1,
    printf ("%s: presentDpram=%d \n", MyName, pPmacCtlr->presentDpram);
  )

  PMAC_DEBUG (1,
    printf ("%s: enabledDpram=%d \n", MyName, pPmacCtlr->enabledDpram);
  )

  PMAC_DEBUG (1,
    printf ("%s: pmacRamNumCtlrs =  %d and  PMAC_MAX_CTLRS = %d\n",
      MyName, pmacRamNumCtlrs, PMAC_MAX_CTLRS);
  )

  return 0;
}

/*******************************************************************************
 *
 * pmacRamInit - Initialize PMAC-VME Hardware Configuration
 *
 */
PMAC_LOCAL long pmacRamInit (void) {
  /* char   *MyName = "pmacRamInit"; */
  int	    i;
  PMAC_CTLR *pPmacCtlr;

  pmacRamConfigLock = 1;

  if (pmacRamNumCtlrs == 0) {
    return 0;
  }

  for (i=0; i < PMAC_MAX_CTLRS; i++) {
    pPmacCtlr = &pmacRamCtlr[i];

    if (pPmacCtlr->configured) {

      if (pPmacCtlr->presentDpram & pPmacCtlr->enabledDpram) {
  	pPmacCtlr->activeDpram = TRUE;
      }

      if (pPmacCtlr->activeDpram & pPmacCtlr->enabledGather) {
  	pPmacCtlr->activeGather = TRUE;
      }

    }
  }	  

  return 0;
}


/*******************************************************************************
 *
 * pmacRamAddr - get DPRAM address
 *
 */
char *pmacRamAddr (int ctlr, int offset) {
  PMAC_CTLR *pCtlr = &pmacRamCtlr[ctlr];
  char      *pDpram = (char *) (pCtlr->pDpramBase + offset);

  PMAC_DEBUG (2,
    PMAC_MESSAGE ("pmacRamAddr:  Controller #%d, at base %#X with offset %#X = address a24 %#010X \n",
      ctlr, (unsigned int) pCtlr->pDpramBase, offset, (unsigned int) pDpram);
  )
/* This is a workaround to restore PMAC clock synchronization in case of 2 PMACS -- Sergey, Oleg 2006.01.30 */
/* (At the end of IOC startup script set pmacRamDebug=1 for about 2s and 6s after booting IOC) */
  PMAC_DEBUG (1,
    PMAC_MESSAGE ("pmacRamAddr:  Controller #%d\n",ctlr);
    /* PMAC_MESSAGE ("\n"); */
    /* printf ("pmacRamAddr:  Controller #%d\n",ctlr); */
    /* printf ("pmacRamAddr:  Controller #%d, at base %#X with offset %#X = address a24 %#010X \n",
         ctlr, (int)pCtlr->pDpramBase, offset, (int)pDpram); */
  )


  return pDpram;
}
	
/*******************************************************************************
 *
 * pmacRamOffs - get DPRAM host offset
 *
 */
short pmacRamOffs (int ctlr, PMAC_DPRAM *paddr) {
  PMAC_CTLR  *pCtlr = &pmacRamCtlr[ctlr];
  unsigned short offset = (unsigned short) (paddr - pCtlr->pDpramBase);

  PMAC_DEBUG (2,
    PMAC_MESSAGE ("pmacRamOffs:  Controller #%d, at base %#X with addr %#X, offset %#010X \n",
      ctlr, (unsigned int) pCtlr->pDpramBase, (unsigned int) paddr, (unsigned short) offset);
  )
/* This is a workaround to restore PMAC clock synchronization in case of 2 PMACS -- Sergey, Oleg 2006.01.30 */
/* (At the end of IOC startup script set pmacRamDebug=1 for about 2s and 6s after booting IOC) */
  PMAC_DEBUG (1,
    PMAC_MESSAGE ("pmacRamOffs:  Controller #%d\n",ctlr);
    /* PMAC_MESSAGE ("\n"); */
    /* printf ("pmacRamOffs:  Controller #%d\n",ctlr); */
    /* printf ("pmacRamOffs:  Controller #%d, at base %#X with addr %#X = address a24 %#010X \n",
         ctlr, (int)pCtlr->pDpramBase, paddr, (unsigned short) offset); */
  )


  return offset;
}
	
/*******************************************************************************
 *
 * pmacRamGet16 - get DPRAM 16 bits
 *
 */
PMAC_LOCAL long pmacRamGet16 (PMAC_DPRAM *pDpram, int *pVal) {
  volatile short int *pRamVal = (volatile short int *) pDpram;

  /* Return value */
  *pVal = (int) (*pRamVal);

  return 0;
}

/*******************************************************************************
 *
 * pmacRamPut16 - put DPRAM 16 bits
 *
 */
PMAC_LOCAL long pmacRamPut16 (PMAC_DPRAM *pDpram, int val) {
  short int *pval = (short int *) pDpram;

/*
  printf ("pmacRamPut16: addr:0x%x, val=%d \n", (unsigned int) pDpram, val);
*/

  *pval = (short int) val;

  return 0;
}

/*******************************************************************************
 *
 * pmacRamGet24U - get DPRAM 24 bits unsigned
 *
 */
PMAC_LOCAL long pmacRamGet24U (PMAC_DPRAM *pDpram, int *pVal) {
  volatile int *pRamVal = (volatile int *) pDpram;

  /* Return value */
  *pVal = *pRamVal & 0xffffff;

  return 0;
}

/*******************************************************************************
 *
 * pmacRamGet24 - get DPRAM 24 bits
 *
 */
PMAC_LOCAL long pmacRamGet24 (PMAC_DPRAM *pDpram, int *pVal) {
  int val0;

  /* Read PMAC DPRAM */
  pmacRamGet24U (pDpram, &val0);

  if (val0 & 0x800000) val0 |= 0xff000000;
  	  
  /* Return value */
  *pVal = val0;

  return 0;
}

/*******************************************************************************
 *
 * pmacRamPut32 - put DPRAM 32 bits
 *
 */
PMAC_LOCAL long pmacRamPut32 (PMAC_DPRAM *pDpram, int val) {
  int *pval = (int *) pDpram;
/*
  printf ("pmacRamPut32: addr:0x%x, val=%d \n", (unsigned int) pDpram, val);
*/

  *pval = val;

  return 0;
}

/*******************************************************************************
 *
 * pmacRamGetF - get DPRAM F word
 *
 */
PMAC_LOCAL long pmacRamGetF (PMAC_DPRAM *pDpram, double *pVal) {
  volatile float *pvalF = (volatile float *) pDpram;

  /* Return value */
  *pVal = (double) (*pvalF);

  return 0;
}

/*******************************************************************************
 *
 * pmacRamPutF - put DPRAM F word
 *
 */
PMAC_LOCAL long pmacRamPutF (PMAC_DPRAM *pDpram, double val) {
  float *valF = (float *) pDpram;
/*
  printf ("pmacRamPutF: addr:0x%x, val=%f \n", (unsigned int) pDpram, val);
*/
  *valF = (float) val;
  return 0;
}

/*******************************************************************************
 *
 * pmacRamGetD - get DPRAM D word
 *
 */
PMAC_LOCAL long pmacRamGetD (PMAC_DPRAM *pDpram, double *pVal) {
  int val0;
  int val1;

  pmacRamGet24U (pDpram, &val0);
  pmacRamGet24 (&(pDpram[4]), &val1);

  /* Convert 48 bit fixed-point format */
  /* Return value */
  *pVal = ((double)val1) * 0x1000000 + (double) val0;

  return 0;
}

/*******************************************************************************
 *
 * pmacRamGetL - get DPRAM L word
 *
 * mantissa/2^35 * 2^(exp-$7ff)
 */
PMAC_LOCAL long pmacRamGetL (PMAC_DPRAM *pDpram, double *pVal) {
  int	 val0;
  int	 val1;
  double mantissa = 0.0;
  int	 exponent = 0;
  double valD;

  /* Read PMAC DPRAM */
  pmacRamGet24U (pDpram, &val0);
  pmacRamGet24U (&(pDpram[4]), &val1);

  /* Convert 48 bit floating point format */
  mantissa = ((double)(val1 & 0x00000fff)) * 0x1000000 + (double) val0;
  exponent = ((val1 >> 12) & 0x00000fff) - 2082;  /* 0x7ff + 35 */

  if (mantissa == 0.0) {
    valD = 0.0;
  } else {
    valD = mantissa * pow (2.0, (double) exponent);
  }

  /* Return value */
  *pVal = valD;

  return 0;
}
