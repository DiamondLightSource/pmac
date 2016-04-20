	/* @(#) devPmacEth.c 1.0 2005/06/09 */

/* devPmacEth.c -  EPICS Device Support for PMAC-ETH ASCII 
                   for communication with PMAC via ethernet*/

/*
 * Author:      Thomas A. Coleman
 * Date:        2005/06/09
 * 
 * Modified by: Gasper Jansa 
 * Modified by: Rok Gajsek
 * Modified by: Rok Vrabic
 * Modified by: Oleg Makarov 
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
*/

/*
 * Modification History:
 * ---------------------
 * .01  2005/06/09        tac     created
 * .02  2005/11/28        gjansa (gasper.jansa@cosylab.com)     modified to be included in M-comm driver
 * .03  2005/11/29	      rgajsek (rok.gajsek@cosylab.com)  adding record support
 * .04  2006/03/16        gjansa (gasper.jansa@cosylab.com) error logging
 * .05  2006/08/17        rvrabic(rok.vrabic@cosylab.com)       added setpoint initialization and watchdog
 */

/*
 * DESCRIPTION:
 * ------------
 * This module implements EPICS Device Support for M-comm driver.
 *
 */

/*
 * INCLUDES
 */

/* EPICS Includes */




#include <alarm.h>
#include <dbAccess.h>
#include <recSup.h>
#include <devSup.h>
#include <epicsExport.h>
#include <callback.h>
#include <recGbl.h>
#include <cantProceed.h>
#include <errlog.h>
#include <aiRecord.h>
#include <biRecord.h>
#include <longinRecord.h>
#include <mbbiRecord.h>
#include <aoRecord.h>
#include <boRecord.h>
#include <longoutRecord.h>
#include <mbboRecord.h>
#include <stringinRecord.h>
#include <stringoutRecord.h>
#include <waveformRecord.h>

#define STATUS_RECORD
#ifdef STATUS_RECORD
  #include	<statusRecord.h>
#endif

/* Standard Includes */
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

/* local includes */
#include	"drvPmacEth.h"

/*
 * DEFINES
 */

#define PMAC_DIAGNOSTICS TRUE
#if PMAC_DIAGNOSTICS
  #define PMAC_MESSAGE	        errlogPrintf
  #define PMAC_DEBUG(level,code)  { if (PMAC_DEBUG >= (level)) { code } }
  #define PMAC_TRACE(level,code)  { if ( (pRec->tpro >= (level)) || (PMAC_DEBUG == (level)) ) { code } }
#else
  #define PMAC_DEBUG(level,code)  ;
  #define PMAC_TRACE(level,code)  ;
#endif

#define PMAC_PRIVATE FALSE
#if PMAC_PRIVATE
  #define PMAC_LOCAL LOCAL
#else
  #define PMAC_LOCAL
#endif

#define NO_ERR_STATUS   (-1)


/*
 * TYPEDEFS
 */

typedef struct {  /* PMAC_DSET_AI */
  long      number;
  DEVSUPFUN report;
  DEVSUPFUN init;
  DEVSUPFUN init_record;
  DEVSUPFUN get_ioint_info;
  DEVSUPFUN read;
  DEVSUPFUN special_linconv;
} PMAC_DSET_AI;

typedef struct {  /* PMAC_DSET_AO */
  long      number;
  DEVSUPFUN report;
  DEVSUPFUN init;
  DEVSUPFUN init_record;
  DEVSUPFUN get_ioint_info;
  DEVSUPFUN write;
  DEVSUPFUN special_linconv;
} PMAC_DSET_AO;

typedef struct {  /* PMAC_DSET_BI */
  long      number;
  DEVSUPFUN report;
  DEVSUPFUN init;
  DEVSUPFUN init_record;
  DEVSUPFUN get_ioint_info;
  DEVSUPFUN read;
} PMAC_DSET_BI;

typedef struct {  /* PMAC_DSET_BO */
  long      number;
  DEVSUPFUN report;
  DEVSUPFUN init;
  DEVSUPFUN init_record;
  DEVSUPFUN get_ioint_info;
  DEVSUPFUN write;
} PMAC_DSET_BO;

typedef struct {  /* PMAC_DSET_LI */
  long      number;
  DEVSUPFUN report;
  DEVSUPFUN init;
  DEVSUPFUN init_record;
  DEVSUPFUN get_ioint_info;
  DEVSUPFUN read;
} PMAC_DSET_LI;

typedef struct {  /* PMAC_DSET_LO */
  long      number;
  DEVSUPFUN report;
  DEVSUPFUN init;
  DEVSUPFUN init_record;
  DEVSUPFUN get_ioint_info;
  DEVSUPFUN write;
} PMAC_DSET_LO;

typedef struct {  /* PMAC_DSET_MBBI */
  long      number;
  DEVSUPFUN report;
  DEVSUPFUN init;
  DEVSUPFUN init_record;
  DEVSUPFUN get_ioint_info;
  DEVSUPFUN read;
} PMAC_DSET_MBBI;

typedef struct {  /* PMAC_DSET_MBBO */
  long      number;
  DEVSUPFUN report;
  DEVSUPFUN init;
  DEVSUPFUN init_record;
  DEVSUPFUN get_ioint_info;
  DEVSUPFUN write;
} PMAC_DSET_MBBO;

typedef struct {  /* PMAC_DSET_SI */
  long      number;
  DEVSUPFUN report;
  DEVSUPFUN init;
  DEVSUPFUN init_record;
  DEVSUPFUN get_ioint_info;
  DEVSUPFUN read;
} PMAC_DSET_SI;

typedef struct {  /* PMAC_DSET_SO */
  long      number;
  DEVSUPFUN report;
  DEVSUPFUN init;
  DEVSUPFUN init_record;
  DEVSUPFUN get_ioint_info;
  DEVSUPFUN write;
} PMAC_DSET_SO;

typedef struct {  /* PMAC_DSET_WF */
  long      number;
  DEVSUPFUN report;
  DEVSUPFUN init;
  DEVSUPFUN init_record;
  DEVSUPFUN get_ioint_info;
  DEVSUPFUN read;
} PMAC_DSET_WF;

#ifdef STATUS_RECORD
typedef struct {  /* PMAC_DSET_STATUS */
  long      number;
  DEVSUPFUN report;
  DEVSUPFUN init;
  DEVSUPFUN init_record;
  DEVSUPFUN get_ioint_info;
  DEVSUPFUN read;
} PMAC_DSET_STATUS;
#endif

typedef struct {  /* PMAC_MBX_DPVT */
  PMAC_MBX_IO MbxIo;
} PMAC_MBX_DPVT;


/*
 * FORWARD DECLARATIONS
 */

static long devPmacMbx_init();

static long devPmacMbxAi_init();
static long devPmacMbxAi_read();

static long devPmacMbxAo_init();
static long devPmacMbxAo_write();

static long devPmacMbxBi_init();
static long devPmacMbxBi_read();

static long devPmacMbxBo_init();
static long devPmacMbxBo_write();

static long devPmacMbxLi_init();
static long devPmacMbxLi_read();

static long devPmacMbxLo_init();
static long devPmacMbxLo_write();

static long devPmacMbxMbbi_init();
static long devPmacMbxMbbi_read();

static long devPmacMbxMbbo_init();
static long devPmacMbxMbbo_write();

static long devPmacMbxSi_init();
static long devPmacMbxSi_read();

static long devPmacMbxSo_init();
static long devPmacMbxSo_write();

static long devPmacMbxWf_init();
static long devPmacMbxWf_write();

#ifdef STATUS_RECORD
  static long devPmacMbxStatus_init();
  static long devPmacMbxStatus_read();
#endif

static long devPmacMbxBoWatchdog_init();
static long devPmacMbxBoWatchdog_write();

int drvPmacMbxScan (PMAC_MBX_IO *pMbxIo);

static void devPmacMbxCallback (CALLBACK *pCallback);

/*
 * GLOBALS
 */
PMAC_DSET_AI devPmacMbxAi = {
  6,
  NULL,
  devPmacMbx_init,
  devPmacMbxAi_init,
  NULL,
  devPmacMbxAi_read,
  NULL
}; 
epicsExportAddress (dset, devPmacMbxAi);

PMAC_DSET_AO devPmacMbxAo = {
  6,
  NULL,
  devPmacMbx_init,
  devPmacMbxAo_init,
  NULL,
  devPmacMbxAo_write,
  NULL
};
epicsExportAddress (dset, devPmacMbxAo);

PMAC_DSET_BI devPmacMbxBi = {
  5,
  NULL,
  devPmacMbx_init,
  devPmacMbxBi_init,
  NULL,
  devPmacMbxBi_read
};

epicsExportAddress (dset, devPmacMbxBi);

PMAC_DSET_BO devPmacMbxBo = {
  5,
  NULL,
  devPmacMbx_init,
  devPmacMbxBo_init,
  NULL,
  devPmacMbxBo_write
};
epicsExportAddress (dset, devPmacMbxBo);

PMAC_DSET_LI devPmacMbxLi = {
  5,
  NULL,
  devPmacMbx_init,
  devPmacMbxLi_init,
  NULL,
  devPmacMbxLi_read
};
epicsExportAddress (dset, devPmacMbxLi);

PMAC_DSET_LO devPmacMbxLo = {
  5,
  NULL,
  devPmacMbx_init,
  devPmacMbxLo_init,
  NULL,
  devPmacMbxLo_write
};
epicsExportAddress (dset, devPmacMbxLo);

PMAC_DSET_MBBI devPmacMbxMbbi = {
  5,
  NULL,
  devPmacMbx_init,
  devPmacMbxMbbi_init,
  NULL,
  devPmacMbxMbbi_read
};
epicsExportAddress (dset, devPmacMbxMbbi);

PMAC_DSET_MBBO devPmacMbxMbbo = {
  5,
  NULL,
  devPmacMbx_init,
  devPmacMbxMbbo_init,
  NULL,
  devPmacMbxMbbo_write
};
epicsExportAddress (dset, devPmacMbxMbbo);

PMAC_DSET_SI devPmacMbxSi = {
  5,
  NULL,
  devPmacMbx_init,
  devPmacMbxSi_init,
  NULL,
  devPmacMbxSi_read
};
epicsExportAddress (dset, devPmacMbxSi);

PMAC_DSET_WF devPmacMbxWf = {
  5,
  NULL,
  devPmacMbx_init,
  devPmacMbxWf_init,
  NULL,
  devPmacMbxWf_write
};
epicsExportAddress (dset, devPmacMbxWf);

PMAC_DSET_SO devPmacMbxSo = {
  5,
  NULL,
  devPmacMbx_init,
  devPmacMbxSo_init,
  NULL,
  devPmacMbxSo_write
};
epicsExportAddress (dset, devPmacMbxSo);

#ifdef STATUS_RECORD
  PMAC_DSET_STATUS devPmacMbxStatus = {
    5,
    NULL,
    devPmacMbx_init,
    devPmacMbxStatus_init,
    NULL,
    devPmacMbxStatus_read
  };
  epicsExportAddress (dset, devPmacMbxStatus);
#endif

PMAC_DSET_LO devPmacMbxBoWatchdog = {
  5,
  NULL,
  devPmacMbx_init,
  devPmacMbxBoWatchdog_init,
  NULL,
  devPmacMbxBoWatchdog_write
};
epicsExportAddress (dset, devPmacMbxBoWatchdog);

char *devPmacMbxVersion = "@(#) devPmacMbx.c 1.0 2012/02/17";

#if PMAC_DIAGNOSTICS
  volatile int PMAC_DEBUG = 0;
  epicsExportAddress (int, PMAC_DEBUG);
#endif

/*watchdog array: each array field is associated with a device card number*/
volatile long watchdog[100];

/*
 * LOCALS
 */

/*******************************************************************************
 *
 * devPmacMbx_init - EPICS device support init function
 *
 */
static long devPmacMbx_init (int after) {
/*	char *MyName = "devPmacMbx_init"; */
  int i;
  long status = 0;

  /*watchdog initialization*/
  for (i=0; i<100; i++) {
    watchdog[i] = 0;
  }

  if (after == 1) {
    status = drvPmacEthInit(); 
  }

  return status;
}


/*******************************************************************************
 *
 * devPmacMbxDpvtInit - EPICS PMAC_MBX_DPVT init
 *
 */
PMAC_MBX_DPVT *devPmacMbxDpvtInit (struct dbCommon *pRec, int card) {
/*char *MyName = "devPmacMbxDpvtInit"; */
  PMAC_MBX_DPVT *pDpvt;

  pDpvt = (PMAC_MBX_DPVT *) malloc (sizeof(PMAC_MBX_DPVT));

  pDpvt->MbxIo.pRec = pRec;
  pDpvt->MbxIo.card = card;
  pDpvt->MbxIo.terminator = 0;
  pDpvt->MbxIo.status = 0;
  pDpvt->MbxIo.init = 0;
  pDpvt->MbxIo.command[0] = '\0';
  pDpvt->MbxIo.response[0] = '\0';

  callbackSetCallback (devPmacMbxCallback, &pDpvt->MbxIo.callback);
  callbackSetPriority (pRec->prio, &pDpvt->MbxIo.callback);
  callbackSetUser ( (void *) pRec, &pDpvt->MbxIo.callback);

  return pDpvt;
}

/*******************************************************************************
 *
 * devPmacMbxAi_init - EPICS PMAC device support ai init
 *
 */
static long devPmacMbxAi_init (struct aiRecord *pRec) {
  char *MyName = "devPmacMbxAi_init";
  /* long status; */

  switch (pRec->inp.type) {
    case (VME_IO) :

      PMAC_DEBUG (1, 
        PMAC_MESSAGE ("%s: %s: \"card %d parm %s.\"\n", MyName, pRec->name,
          pRec->inp.value.vmeio.card, pRec->inp.value.vmeio.parm);
      )

      pRec->dpvt = (void *) devPmacMbxDpvtInit ((struct dbCommon *) pRec, (int) pRec->inp.value.vmeio.card);
      return 0;

    default :
      recGblRecordError (S_db_badField, (void *) pRec, "devPmacMbxAi_init: Illegal INP field");
      return S_db_badField;
  }
}
/*******************************************************************************
 *
 * devPmacMbxAi_read - EPICS PMAC device support ai read
 *
 */
static long devPmacMbxAi_read (struct aiRecord *pRec) {
  char *MyName = "devPmacMbxAi_read";
  /* long status; */

  double  valD;

  PMAC_MBX_DPVT * pDpvt = (PMAC_MBX_DPVT *) pRec->dpvt;
  PMAC_MBX_IO *   pMbxIo = &pDpvt->MbxIo;

  if (pRec->pact) {

      PMAC_TRACE (2,
        PMAC_MESSAGE ("%s: %s \"Command  [%s] response [%s].\"\n", MyName, pRec->name, pMbxIo->command, pMbxIo->response);)

    if(pMbxIo->status == -1){ /*error in communication*/
      PMAC_MESSAGE ("%s : %s \"Communication problems.\"\n", MyName, pRec->name);      
      recGblSetSevr (pRec, COMM_ALARM, INVALID_ALARM);

      return 2;
    }
    if(pMbxIo->status == -2){ /*PMAC reported an error*/
      PMAC_MESSAGE ("%s : %s \"PMAC reported an error code: %s.\"\n", MyName, pRec->name, pMbxIo->errmsg);
      recGblSetSevr (pRec, COMM_ALARM, INVALID_ALARM);
      return 2;
    }



    sscanf (pMbxIo->response, "%lf", &valD);

    /* Adjust Slope And Offset */
    if (pRec->aslo != 0.0) {
      valD *= pRec->aslo;
    }
    if (pRec->aoff != 0.0) {
      valD += pRec->aoff;
    }

    /* pRec->linr Conversion Ignored */

    /* Apply Smoothing Algorithm */
    if (pRec->smoo != 0.0) {
      if (pRec->init == TRUE) pRec->val = valD;	    /* initial condition */
      pRec->val = valD * (1.00 - pRec->smoo) + (pRec->val * pRec->smoo);
    } else {
      pRec->val = valD;
    }

    pRec->udf = FALSE;
    return 2;
  } else {

    sprintf (pMbxIo->command, "%s", pRec->inp.value.vmeio.parm);

    PMAC_TRACE (2, PMAC_MESSAGE ("%s: %s \"Command to send [%s].\"\n", MyName, pRec->name, pMbxIo->command);)
  
    if(drvPmacMbxScan (pMbxIo) < 0) {
      PMAC_MESSAGE ("%s: %s  \"PMAC device number %d is not configured!\"\n",MyName,pRec->name,pMbxIo->card);
      recGblSetSevr (pRec, COMM_ALARM, INVALID_ALARM);
    }
  
    pRec->pact = TRUE;
  	    
    return 0;
  }
}

/*******************************************************************************
 *
 * devPmacMbxAo_init - EPICS PMAC device support ao init
 *
 */
static long devPmacMbxAo_init (struct aoRecord *pRec) {
  char          *MyName = "devPmacMbxAo_init";
  PMAC_MBX_DPVT *pDpvt;

  switch (pRec->out.type) {
    case (VME_IO) :

      PMAC_DEBUG (1,
        PMAC_MESSAGE ("%s: %s: \"card %d parm %s.\"\n", MyName,pRec->name,
          pRec->out.value.vmeio.card, pRec->out.value.vmeio.parm);
      )

      pRec->dpvt = (void *) devPmacMbxDpvtInit ((struct dbCommon *)pRec, (int)pRec->out.value.vmeio.card);

      if(pRec->out.value.vmeio.signal == 2) {
        pDpvt = (PMAC_MBX_DPVT *)pRec->dpvt;
        pDpvt->MbxIo.init=1;
      }     

      return 2;

    default :
      recGblRecordError (S_db_badField, (void *) pRec, "devPmacMbxAo_init: Illegal OUT field");
      return S_db_badField;
  }
}

/*******************************************************************************
 *
 * devPmacMbxAo_write - EPICS PMAC device support ao write
 *
 */
static long devPmacMbxAo_write (struct aoRecord *pRec) {
  char *MyName = "devPmacMbxAo_write";
  /* long status; */

  double  valD;

  /*temporary string: 31 = max length of out.value.vmeio.parm*/
  char temp[31];

  PMAC_MBX_DPVT *pDpvt = (PMAC_MBX_DPVT *) pRec->dpvt;
  PMAC_MBX_IO	*pMbxIo = &pDpvt->MbxIo;

  if (pRec->pact) {
    PMAC_TRACE (2, PMAC_MESSAGE ("%s: %s \"Command  [%s] response [%s].\"\n", MyName, pRec->name, pMbxIo->command, pMbxIo->response);)
    if(pMbxIo->status == -1) { /*error in communication*/
       PMAC_MESSAGE ("%s : %s \"Communication problems.\"\n",MyName, pRec->name);
       recGblSetSevr (pRec, COMM_ALARM, INVALID_ALARM);
       pMbxIo->init = 0;
       return 0;
    }
    if(pMbxIo->status == -2) { /*PMAC reported an error*/
      PMAC_MESSAGE ("%s : %s \"PMAC reported an error code: %s.\"\n", MyName, pRec->name, pMbxIo->errmsg);
      recGblSetSevr (pRec, COMM_ALARM, INVALID_ALARM);
      pMbxIo->init = 0;
      return 0;
    }
    if(pMbxIo->init == 1) {
      sscanf (pMbxIo->response, "%lf", &valD);
      pRec->val = valD;
      pMbxIo->init = 0;
      return 0;
    }
    return 0;
  } else {
    if(pMbxIo->init == 1) {
      strcpy(temp, pRec->out.value.vmeio.parm);
      temp[strlen(temp)-1] = 0;
      sprintf (pMbxIo->command, "%s", temp);

      PMAC_TRACE (2, PMAC_MESSAGE ("%s: %s \"Command to send [%s].\"\n", MyName, pRec->name, pMbxIo->command);)
  
      if(drvPmacMbxScan (pMbxIo) < 0) {
  	PMAC_MESSAGE ("%s: %s  \"PMAC device number %d is not configured!\"\n",MyName,pRec->name,pMbxIo->card);
  	recGblSetSevr (pRec, COMM_ALARM, INVALID_ALARM);
      }
  
      pRec->pact = TRUE;

      return 0;
    } else {

      /* Output Value */
      valD = (double) pRec->oval;

      /* Adjust Slope And Offset */
      if (pRec->aoff != 0.0) {
  	valD -= (double) pRec->aoff;
      }
      if (pRec->aslo != 0.0) {
  	valD /= (double) pRec->aslo;
      }

      /* pRec->linr Conversion Ignored */

      sprintf (pMbxIo->command, "%s%f", pRec->out.value.vmeio.parm, valD);

      PMAC_TRACE (2, PMAC_MESSAGE ("%s: %s \"Command to send [%s].\"\n", MyName, pRec->name, pMbxIo->command);)
  
      if(drvPmacMbxScan (pMbxIo) < 0) {
  	PMAC_MESSAGE ("%s: %s  \"PMAC device number %d is not configured!\"\n",MyName,pRec->name,pMbxIo->card);
  	recGblSetSevr (pRec, COMM_ALARM, INVALID_ALARM);
      }
  
      pRec->pact = TRUE;
  
      return 0;
    }
  }
}

/*******************************************************************************
 *
 * devPmacMbxBi_init - EPICS PMAC device support bi init
 *
 */
static long devPmacMbxBi_init (struct biRecord *pRec) {
  char *MyName = "devPmacMbxBi_init";
  /* long status; */

  switch (pRec->inp.type) {
    case (VME_IO) :
      PMAC_DEBUG (1, PMAC_MESSAGE ("%s: %s: \"card %d parm %s.\"\n", MyName,pRec->name,
  	pRec->inp.value.vmeio.card, pRec->inp.value.vmeio.parm);)
      pRec->dpvt = (void *) devPmacMbxDpvtInit ((struct dbCommon *) pRec, (int) pRec->inp.value.vmeio.card);
      return 0;

    default :
      recGblRecordError (S_db_badField, (void *) pRec, "devPmacMbxBi_init: Illegal INP field");
      return S_db_badField;
  }
}
/*******************************************************************************
 *
 * devPmacMbxBi_read - EPICS PMAC device support bi read
 *
 */
static long devPmacMbxBi_read (struct biRecord *pRec) {
  char *  MyName = "devPmacMbxBi_read";
  /* long status; */

  long valL;

  PMAC_MBX_DPVT *pDpvt = (PMAC_MBX_DPVT *) pRec->dpvt;
  PMAC_MBX_IO	*pMbxIo = &pDpvt->MbxIo;

  if (pRec->pact) {

    PMAC_TRACE (2,
      PMAC_MESSAGE ("%s: %s \"Command [%s] response [%s].\"\n", MyName, pRec->name, pMbxIo->command, pMbxIo->response);
    )

    if (pMbxIo->status == -1) { /*error in communication*/
      PMAC_MESSAGE ("%s : %s \"Communication problems.\"\n",MyName, pRec->name);
      recGblSetSevr (pRec, COMM_ALARM, INVALID_ALARM);
      return 0;
    }

    if (pMbxIo->status == -2) { /*PMAC reported an error*/
      PMAC_MESSAGE ("%s : %s \"PMAC reported an error code: %s.\"\n", MyName, pRec->name,pMbxIo->errmsg);
      recGblSetSevr (pRec, COMM_ALARM, INVALID_ALARM);
      return 0;
    }

    sscanf (pMbxIo->response, "%ld", &valL);

    pRec->rval = (unsigned long) valL;
    pRec->udf = FALSE;

    return 0;

  } else {

    sprintf (pMbxIo->command, "%s", pRec->inp.value.vmeio.parm);

    PMAC_TRACE (2,
      PMAC_MESSAGE ("%s: %s \"Command to send [%s].\"\n", MyName, pRec->name, pMbxIo->command);
    )

    if (drvPmacMbxScan (pMbxIo) < 0) {
      PMAC_MESSAGE ("%s: %s  \"PMAC device number %d is not configured!\"\n",MyName,pRec->name,pMbxIo->card);
      recGblSetSevr (pRec, COMM_ALARM, INVALID_ALARM);
    }

    pRec->pact = TRUE;


    return 0;
  }
}

/*******************************************************************************
 *
 * devPmacMbxBo_init - EPICS PMAC device support bo init
 *
 */
static long devPmacMbxBo_init (struct boRecord *pRec) {
  char          *MyName = "devPmacMbxBo_init";
  PMAC_MBX_DPVT *pDpvt;

  switch (pRec->out.type) {
    case (VME_IO) :
      PMAC_DEBUG (1, PMAC_MESSAGE ("%s: %s: \"card %d parm %s.\"\n", MyName,pRec->name,
  	pRec->out.value.vmeio.card, pRec->out.value.vmeio.parm);)
      pRec->dpvt = (void *) devPmacMbxDpvtInit ((struct dbCommon *) pRec, (int) pRec->out.value.vmeio.card);

      if(pRec->out.value.vmeio.signal == 2) {
  	pDpvt = (PMAC_MBX_DPVT *)pRec->dpvt;
  	pDpvt->MbxIo.init=1;
      }     

      return 2;

    default :
      recGblRecordError (S_db_badField, (void *) pRec, "devPmacMbxBo_init: Illegal OUT field");
      return S_db_badField;
  }

}
/*******************************************************************************
 *
 * devPmacMbxBo_write - EPICS PMAC device support bo write
 *
 */
static long devPmacMbxBo_write (struct boRecord *pRec) {
  char *MyName = "devPmacMbxBo_write";
  /* long status; */

  /*temporary string: 31 = max length of out.value.vmeio.parm*/
  char temp[31];

  long valL;

  PMAC_MBX_DPVT * pDpvt = (PMAC_MBX_DPVT *) pRec->dpvt;
  PMAC_MBX_IO *   pMbxIo = &pDpvt->MbxIo;

  if (pRec->pact) {

    PMAC_TRACE (2, PMAC_MESSAGE ("%s: %s \"Command  [%s] response [%s].\"\n", MyName, pRec->name, pMbxIo->command, pMbxIo->response);)
    if(pMbxIo->status == -1) { /*error in communication*/
      PMAC_MESSAGE ("%s : %s \"Communication problems.\"\n",MyName, pRec->name);
      recGblSetSevr (pRec, COMM_ALARM, INVALID_ALARM);
      pMbxIo->init = 0;
      return 0;
    }
    if(pMbxIo->status == -2) { /*PMAC reported an error*/
      PMAC_MESSAGE ("%s : %s \"PMAC reported an error code: %s.\"\n", MyName, pRec->name,pMbxIo->errmsg);
      recGblSetSevr (pRec, COMM_ALARM, INVALID_ALARM);
      pMbxIo->init = 0;
      return 0;
    }
    if(pMbxIo->init == 1) {
      sscanf (pMbxIo->response, "%ld", &valL);
      pRec->val = valL;
      pMbxIo->init = 0;
      return 0;
    }
    return 0;
  } else {
    if(pMbxIo->init == 1) {
      strcpy(temp, pRec->out.value.vmeio.parm);
      temp[strlen(temp)-1] = 0;
      sprintf (pMbxIo->command, "%s", temp);

      PMAC_TRACE (2, PMAC_MESSAGE ("%s: %s \"Command to send [%s].\"\n", MyName, pRec->name, pMbxIo->command);)
  
      if(drvPmacMbxScan (pMbxIo) < 0) {
  	PMAC_MESSAGE ("%s: %s  \"PMAC device number %d is not configured!\"\n",MyName,pRec->name,pMbxIo->card);
  	recGblSetSevr (pRec, COMM_ALARM, INVALID_ALARM);
      }
  
      pRec->pact = TRUE;

      return 0;
    } else {
      valL = (long) pRec->val;

      sprintf (pMbxIo->command,"%s%ld", pRec->out.value.vmeio.parm, valL);

      PMAC_TRACE (2, PMAC_MESSAGE ("%s: %s \"Command to send [%s].\"\n", MyName, pRec->name, pMbxIo->command);)

      if(drvPmacMbxScan (pMbxIo) < 0) {
  	PMAC_MESSAGE ("%s: %s  \"PMAC device number %d is not configured!\"\n",MyName,pRec->name,pMbxIo->card);
  	recGblSetSevr (pRec, COMM_ALARM, INVALID_ALARM);
      }
  
      pRec->pact = TRUE;

      return 0;
    }
  }
}

/*******************************************************************************
 *
 * devPmacMbxLi_init - EPICS PMAC device support longin init
 *
 */
static long devPmacMbxLi_init (struct longinRecord *pRec) {
  char *MyName = "devPmacMbxLi_init";
  /* long status; */

  switch (pRec->inp.type) {
    case (VME_IO) :

      PMAC_DEBUG (1, PMAC_MESSAGE ("%s: %s: \"card %d parm %s.\"\n", MyName,pRec->name,
  	pRec->inp.value.vmeio.card, pRec->inp.value.vmeio.parm);)

      pRec->dpvt = (void *) devPmacMbxDpvtInit ((struct dbCommon *) pRec, (int) pRec->inp.value.vmeio.card);
      return 0;

    default :
      recGblRecordError (S_db_badField, (void *) pRec, "devPmacMbxLi_init: Illegal INP field");
      return S_db_badField;
  }
}
/*******************************************************************************
 *
 * devPmacMbxLi_read - EPICS PMAC device support longin read
 *
 */
static long devPmacMbxLi_read (struct longinRecord *pRec) {
  char *MyName = "devPmacMbxLi_read";
  /* long status; */

  long valL;

  PMAC_MBX_DPVT *pDpvt = (PMAC_MBX_DPVT *) pRec->dpvt;
  PMAC_MBX_IO	*pMbxIo = &pDpvt->MbxIo;

  if (pRec->pact) {

    PMAC_TRACE (2, PMAC_MESSAGE ("%s: %s \"Command  [%s] response [%s].\"\n", MyName, pRec->name, pMbxIo->command, pMbxIo->response);)

    if(pMbxIo->status == -1) { /*error in communication*/
      PMAC_MESSAGE ("%s : %s \"Communication problems.\"\n",MyName, pRec->name);
      recGblSetSevr (pRec, COMM_ALARM, INVALID_ALARM);
      return 0;
    }
    if(pMbxIo->status == -2) { /*PMAC reported an error*/
      PMAC_MESSAGE ("%s : %s \"PMAC reported an error code: %s.\"\n", MyName, pRec->name,pMbxIo->errmsg);
      recGblSetSevr (pRec, COMM_ALARM, INVALID_ALARM);
      return 0;
    }

    sscanf (pMbxIo->response, "%ld", &valL);

    pRec->val = valL;
    pRec->udf = FALSE;

    return 0;
  } else {

    sprintf (pMbxIo->command, "%s", pRec->inp.value.vmeio.parm);

    PMAC_TRACE (2, PMAC_MESSAGE ("%s: %s \"Command to send [%s].\"\n", MyName, pRec->name, pMbxIo->command);)

    if(drvPmacMbxScan (pMbxIo) < 0) {
      PMAC_MESSAGE ("%s: %s  \"PMAC device number %d is not configured!\"\n",MyName,pRec->name,pMbxIo->card);
      recGblSetSevr (pRec, COMM_ALARM, INVALID_ALARM);
    }
  
    pRec->pact = TRUE;
  	    
    return 0;
  }
}

/*******************************************************************************
 *
 * devPmacMbxLo_init - EPICS PMAC device support longout init
 *
 */
static long devPmacMbxLo_init (struct longoutRecord *pRec) {
  char          *MyName = "devPmacMbxLo_init";
  PMAC_MBX_DPVT *pDpvt;

  switch (pRec->out.type) {

    case (VME_IO) :
      PMAC_DEBUG (1, PMAC_MESSAGE ("%s: %s: \"card %d parm %s.\"\n", MyName,pRec->name,
        pRec->out.value.vmeio.card, pRec->out.value.vmeio.parm);)
      pRec->dpvt = (void *) devPmacMbxDpvtInit ((struct dbCommon *) pRec, (int) pRec->out.value.vmeio.card);

      if(pRec->out.value.vmeio.signal == 2) {
        pDpvt = (PMAC_MBX_DPVT *)pRec->dpvt;
        pDpvt->MbxIo.init = 1;
      } 					    
      return 0;

    default :
      recGblRecordError (S_db_badField, (void *) pRec, "devPmacMbxLo_init: Illegal OUT field");
      return S_db_badField;
  }

}

/*******************************************************************************
 *
 * devPmacMbxLo_write - EPICS PMAC device support longout write
 *
 */
static long devPmacMbxLo_write (struct longoutRecord *pRec) {
  char *MyName = "devPmacMbxLo_write";
  /* long status; */

  /*temporary string: 31 = max length of out.value.vmeio.parm*/
  char temp[31];

  long valL;

  PMAC_MBX_DPVT *pDpvt = (PMAC_MBX_DPVT *) pRec->dpvt;
  PMAC_MBX_IO   *pMbxIo = &pDpvt->MbxIo;

  if (pRec->pact) {

    PMAC_TRACE (2, PMAC_MESSAGE ("%s: %s \"Command  [%s] response [%s].\"\n", MyName, pRec->name, pMbxIo->command, pMbxIo->response);)

    if(pMbxIo->status == -1) { /*error in communication*/
      PMAC_MESSAGE ("%s : %s \"Communication problems.\"\n",MyName, pRec->name);
      recGblSetSevr (pRec, COMM_ALARM, INVALID_ALARM);
      pMbxIo->init = 0;
      return 0;
    }

    if(pMbxIo->status == -2) { /*PMAC reported an error*/
      PMAC_MESSAGE ("%s : %s \"PMAC reported an error code: %s.\"\n", MyName, pRec->name,pMbxIo->errmsg);
      recGblSetSevr (pRec, COMM_ALARM, INVALID_ALARM);
      pMbxIo->init = 0;
      return 0;
    }

    if(pMbxIo->init == 1) {
      sscanf (pMbxIo->response, "%ld", &valL);
      pRec->val = valL;
      pMbxIo->init = 0;
      return 0;
    }

  } else {

    if(pMbxIo->init == 1) {
      strcpy(temp, pRec->out.value.vmeio.parm);
      temp[strlen(temp)-1] = 0;
      sprintf (pMbxIo->command, "%s", temp);

      PMAC_TRACE (2, PMAC_MESSAGE ("%s: %s \"Command to send [%s].\"\n", MyName, pRec->name, pMbxIo->command);)
    
      if(drvPmacMbxScan (pMbxIo) < 0) {
  	PMAC_MESSAGE ("%s: %s  \"PMAC device number %d is not configured!\"\n",MyName,pRec->name,pMbxIo->card);
  	recGblSetSevr (pRec, COMM_ALARM, INVALID_ALARM);
      }
    
      pRec->pact = TRUE;

    } else {
      valL = (long) pRec->val;

      sprintf (pMbxIo->command,"%s%ld", pRec->out.value.vmeio.parm, valL);

      PMAC_TRACE (2, PMAC_MESSAGE ("%s: %s \"Command to send [%s].\"\n", MyName, pRec->name, pMbxIo->command);)

      if(drvPmacMbxScan (pMbxIo) < 0) {
  	PMAC_MESSAGE ("%s: %s  \"PMAC device number %d is not configured!\"\n",MyName,pRec->name,pMbxIo->card);
  	recGblSetSevr (pRec, COMM_ALARM, INVALID_ALARM);
      }
    
      pRec->pact = TRUE;
  	    
    }
  }
  return 0;
}

/*******************************************************************************
 *
 * devPmacMbxMbbi_init - EPICS PMAC device support mbbi init
 *
 */
static long devPmacMbxMbbi_init (struct mbbiRecord *pRec) {
  char *MyName = "devPmacMbxMbbi_init";
  /* long status; */

  switch (pRec->inp.type) {
    case (VME_IO) :
      PMAC_DEBUG (1, PMAC_MESSAGE ("%s: %s: \"card %d parm %s.\"\n", MyName, pRec->name,
	pRec->inp.value.vmeio.card, pRec->inp.value.vmeio.parm);)
      pRec->dpvt = (void *) devPmacMbxDpvtInit ((struct dbCommon *) pRec, (int) pRec->inp.value.vmeio.card);
      return 0;

    default :
      recGblRecordError (S_db_badField, (void *) pRec, "devPmacMbxMbbi_init: Illegal INP field");
      return S_db_badField;
  }

}

/*******************************************************************************
 *
 * devPmacMbxMbbi_read - EPICS PMAC device support mbbi read
 *
 */
static long devPmacMbxMbbi_read (struct mbbiRecord *pRec) {
  char *MyName = "devPmacMbxMbbi_read";
  /* long status; */

  long valL;

  PMAC_MBX_DPVT *pDpvt = (PMAC_MBX_DPVT *) pRec->dpvt;
  PMAC_MBX_IO   *pMbxIo = &pDpvt->MbxIo;

  if (pRec->pact) {

    PMAC_TRACE (2, PMAC_MESSAGE ("%s: %s \"Command  [%s] response [%s].\"\n", MyName, pRec->name, pMbxIo->command, pMbxIo->response);)

    if(pMbxIo->status == -1) { /*error in communication*/
      PMAC_MESSAGE ("%s : %s \"Communication problems.\"\n", MyName, pRec->name);
      recGblSetSevr (pRec, COMM_ALARM, INVALID_ALARM);
      return 0;
    }

    if(pMbxIo->status == -2) { /*PMAC reported an error*/
      PMAC_MESSAGE ("%s : %s \"PMAC reported an error code: %s.\"\n", MyName, pRec->name,pMbxIo->errmsg);
      recGblSetSevr (pRec, COMM_ALARM, INVALID_ALARM);
      return 0;
    }

    sscanf (pMbxIo->response, "%ld", &valL);

    pRec->rval = (unsigned long) valL;
    pRec->udf = FALSE;

  } else {

    sprintf (pMbxIo->command, "%s", pRec->inp.value.vmeio.parm);

    PMAC_TRACE (2, PMAC_MESSAGE ("%s: %s \"Command to send [%s].\"\n", MyName, pRec->name, pMbxIo->command);)

    if(drvPmacMbxScan (pMbxIo) < 0) {
      PMAC_MESSAGE ("%s: %s  \"PMAC device number %d is not configured!\"\n", MyName, pRec->name, pMbxIo->card);
      recGblSetSevr (pRec, COMM_ALARM, INVALID_ALARM);
    }
  
    pRec->pact = TRUE;
  	  
  }

  return 0;
}
/*******************************************************************************
 *
 * devPmacMbxMbbo_init - EPICS PMAC device support mbbo init
 *
 */
static long devPmacMbxMbbo_init (struct mbboRecord *pRec) {
  char *MyName = "devPmacMbxMbbo_init";
  long status = 0;

  switch (pRec->out.type) {

    case (VME_IO) :
      PMAC_DEBUG (1, PMAC_MESSAGE ("%s: %s: \"card %d parm %s.\"\n", MyName, pRec->name,
  	pRec->out.value.vmeio.card, pRec->out.value.vmeio.parm);)
      pRec->dpvt = (void *) devPmacMbxDpvtInit ((struct dbCommon *) pRec, (int) pRec->out.value.vmeio.card);
      return status;

    default :
      recGblRecordError (S_db_badField, (void *) pRec, "devPmacMbxMbbo_init: Illegal INP field");
      return S_db_badField;
  }

}

/*******************************************************************************
 *
 * devPmacMbxMbbo_write - EPICS PMAC device support mbbo write
 *
 */
static long devPmacMbxMbbo_write (struct mbboRecord *pRec) {
  char *MyName = "devPmacMbxMbbo_write";
  /* long status; */
  long valL;

  PMAC_MBX_DPVT *pDpvt = (PMAC_MBX_DPVT *) pRec->dpvt;
  PMAC_MBX_IO   *pMbxIo = &pDpvt->MbxIo;

  if (pRec->pact) {

    PMAC_TRACE (2, PMAC_MESSAGE ("%s: %s \"Command  [%s] response [%s].\"\n", MyName, pRec->name, pMbxIo->command, pMbxIo->response);)

    if(pMbxIo->status == -1) { /*error in communication*/
      PMAC_MESSAGE ("%s : %s \"Communication problems.\"\n", MyName, pRec->name);
      recGblSetSevr (pRec, COMM_ALARM, INVALID_ALARM);
      return 0;
    }

    if(pMbxIo->status == -2) { /*PMAC reported an error*/
      PMAC_MESSAGE ("%s : %s \"PMAC reported an error code: %s.\"\n", MyName, pRec->name,pMbxIo->errmsg);
      recGblSetSevr (pRec, COMM_ALARM, INVALID_ALARM);
    }

  } else {

    valL = (long) pRec->rval;
    sprintf (pMbxIo->command,"%s%ld", pRec->out.value.vmeio.parm, valL);
    PMAC_TRACE (2, PMAC_MESSAGE ("%s: %s \"Command to send [%s].\"\n", MyName, pRec->name, pMbxIo->command);)

    if(drvPmacMbxScan (pMbxIo) < 0) {
      PMAC_MESSAGE ("%s: %s  \"PMAC device number %d is not configured!\"\n", MyName, pRec->name, pMbxIo->card);
      recGblSetSevr (pRec, COMM_ALARM, INVALID_ALARM);
    }
  
    pRec->pact = TRUE;
  
  }

  return 0;
}
/*******************************************************************************
 *
 * devPmacMbxSi_init - EPICS PMAC device support stringin init
 *
 */
static long devPmacMbxSi_init (struct stringinRecord *pRec) {
  char *MyName = "devPmacMbxSi_init";
  long status = 0;

  switch (pRec->inp.type) {
  case (VME_IO) :

    PMAC_DEBUG (1, PMAC_MESSAGE ("%s: name %s card %d parm %s\n", MyName, pRec->name,
      pRec->inp.value.vmeio.card, pRec->inp.value.vmeio.parm);)

    pRec->dpvt = (void *) devPmacMbxDpvtInit ((struct dbCommon *) pRec, (int) pRec->inp.value.vmeio.card);
    return status;

  default :
    recGblRecordError (S_db_badField, (void *) pRec, "devPmacMbxSi_init: Illegal INP field");
    return S_db_badField;
  }

}
/*******************************************************************************
 *
 * devPmacMbxSi_read - EPICS PMAC device support stringin read
 *
 */
static long devPmacMbxSi_read (struct stringinRecord *pRec) {
  char *MyName = "devPmacMbxSi_read";
  /* long status; */

  PMAC_MBX_DPVT *pDpvt = (PMAC_MBX_DPVT *) pRec->dpvt;
  PMAC_MBX_IO   *pMbxIo = &pDpvt->MbxIo;

  if (pRec->pact) {

    PMAC_TRACE (2, PMAC_MESSAGE ("%s: %s \"Command  [%s] response [%s].\"\n", MyName, pRec->name, pMbxIo->command, pMbxIo->response);)

    if(pMbxIo->status == -1) { /*error in communication*/
      PMAC_MESSAGE ("%s : %s \"Communication problems.\"\n", MyName, pRec->name);
      recGblSetSevr (pRec, COMM_ALARM, INVALID_ALARM);
      return 0;
    }

    if(pMbxIo->status == -2) { /*PMAC reported an error*/
      PMAC_MESSAGE ("%s : %s \"PMAC reported an error code: %s.\"\n", MyName, pRec->name, pMbxIo->errmsg);
      recGblSetSevr (pRec, COMM_ALARM, INVALID_ALARM);
      return 0;
    }	  
    pRec->val[39] = '\0';
    strncpy (pRec->val, pMbxIo->response, 39);

    pRec->udf = FALSE;
  } else {
    switch (pRec->inp.value.vmeio.signal) {
    case (1):
      sprintf (pMbxIo->command,"%s%s", pRec->inp.value.vmeio.parm, pRec->val);
      break;
    case (0):
    default:
      sprintf (pMbxIo->command,"%s", pRec->inp.value.vmeio.parm);
      break;
    }

    PMAC_TRACE (2, PMAC_MESSAGE ("%s: %s \"Command to send [%s].\"\n", MyName, pRec->name, pMbxIo->command);)

    if(drvPmacMbxScan (pMbxIo) < 0) {
      PMAC_MESSAGE ("%s: %s  \"PMAC device number %d is not configured!\"\n", MyName, pRec->name, pMbxIo->card);
      recGblSetSevr (pRec, COMM_ALARM, INVALID_ALARM);
    }

    pRec->pact = TRUE;
  }
  return 0;
}

/*******************************************************************************
 *
 * devPmacMbxWf_init - EPICS PMAC device support waveform init
 *
 */
LOCAL long devPmacMbxWf_init (struct waveformRecord *pRec, int pass) {
  char *MyName = "devPmacMbxWf_init";
  long status = 0;

  if (pass==0) {
    if (pRec->nelm <= 0)
      pRec->nelm = 1;
    if (pRec->ftvl > DBF_ENUM) pRec->ftvl = DBF_UCHAR;
    pRec->bptr = callocMustSucceed(pRec->nelm, dbValueSize(pRec->ftvl), "waveform calloc failed");
    pRec->val = pRec->bptr;
    if (pRec->nelm == 1) {
      pRec->nord = 1;
    } else {
      pRec->nord = 0;
    }
    return 0;
  }

  switch (pRec->inp.type) {
    case (VME_IO) :
      PMAC_DEBUG (1, PMAC_MESSAGE ("%s: name %s card %d parm %s\n", MyName, pRec->name,
	pRec->inp.value.vmeio.card, pRec->inp.value.vmeio.parm);)
      pRec->dpvt = (void *) devPmacMbxDpvtInit ((struct dbCommon *) pRec, (int) pRec->inp.value.vmeio.card);
      /* signal 2 is for PMAC error messages; tell PMAC driver where to report error message */
      if (pRec->inp.value.vmeio.signal == 2) drvPmacStrErr (pRec);
      return status;

    default :
      recGblRecordError (S_db_badField, (void *)pRec, "devPmacMbxWf_init: Illegal OUT field");
      return S_db_badField;
  }

}

/*******************************************************************************
 *
 * devPmacMbxSo_init - EPICS PMAC device support stringout init
 *
 */
static long devPmacMbxSo_init (struct stringoutRecord *pRec) {
  char *MyName = "devPmacMbxSo_init";
  long status = 0;

  switch (pRec->out.type) {

    case (VME_IO) :
      PMAC_DEBUG (1, PMAC_MESSAGE ("%s: name %s card %d parm %s\n", MyName, pRec->name,
	pRec->out.value.vmeio.card, pRec->out.value.vmeio.parm);)
      pRec->dpvt = (void *) devPmacMbxDpvtInit ((struct dbCommon *) pRec, (int) pRec->out.value.vmeio.card);
      return status;

    default :
      recGblRecordError (S_db_badField, (void *) pRec, "devPmacMbxSo_init: Illegal OUT field");
      return S_db_badField;
  }

}
static long devPmacMbxWf_write (struct stringoutRecord *pRec) {
  return 0;
}

/*******************************************************************************
 *
 * devPmacMbxSo_write - EPICS PMAC device support stringout write
 *
 */
static long devPmacMbxSo_write (struct stringoutRecord *pRec) {
  char *MyName = "devPmacMbxSo_write";
  /* long status; */

  PMAC_MBX_DPVT *pDpvt = (PMAC_MBX_DPVT *) pRec->dpvt;
  PMAC_MBX_IO   *pMbxIo = &pDpvt->MbxIo;

  if (pRec->pact) {
    PMAC_TRACE (2, PMAC_MESSAGE ("%s: %s \"Command  [%s] response [%s].\"\n", MyName, pRec->name, pMbxIo->command, pMbxIo->response);)
  	  
    if(pMbxIo->status == -1) { /*error in communication*/
      PMAC_MESSAGE ("%s : %s \"Communication problems.\"\n", MyName, pRec->name);
      recGblSetSevr (pRec, COMM_ALARM, INVALID_ALARM);
      return 0;
    }

    if(pMbxIo->status == -2) { /*PMAC reported an error*/
      PMAC_MESSAGE ("%s: %s: \"%s\" while executing \"%s\".\n", MyName, pRec->name, pMbxIo->errmsg, pMbxIo->command);
      recGblSetSevr (pRec, COMM_ALARM, INVALID_ALARM);
      return 0;
    }

    switch (pRec->out.value.vmeio.signal) {
      case (1):
  	pRec->val[39] = '\0';
  	strncpy (pRec->val, pMbxIo->response, 39);
  	break;
      case (0):
      default: break;
    }

  } else {

    sprintf (pMbxIo->command,"%s%s", pRec->out.value.vmeio.parm, pRec->val);

    PMAC_TRACE (2, PMAC_MESSAGE ("%s: %s \"Command to send [%s].\"\n", MyName, pRec->name, pMbxIo->command);)

    if(drvPmacMbxScan (pMbxIo) < 0) {
      PMAC_MESSAGE ("%s: %s  \"PMAC device number %d is not configured!\"\n", MyName, pRec->name, pMbxIo->card);
      recGblSetSevr (pRec, COMM_ALARM, INVALID_ALARM);
    }

    pRec->pact = TRUE;

  }
  return 0;
}

#ifdef STATUS_RECORD
  /*******************************************************************************
   *
   * devPmacMbxStatus_init - EPICS PMAC device support status init
   *
   */
  static long devPmacMbxStatus_init (struct statusRecord *pRec) {
    char *MyName = "devPmacMbxStatus_init";
    /* long status; */

    switch (pRec->inp.type) {
      case (VME_IO) :

	PMAC_DEBUG (1, PMAC_MESSAGE ("%s: %s: \"card %d parm %s.\"\n", MyName, pRec->name,
	  pRec->inp.value.vmeio.card, pRec->inp.value.vmeio.parm);)

	pRec->dpvt = (void *) devPmacMbxDpvtInit ((struct dbCommon *) pRec, (int) pRec->inp.value.vmeio.card);
	return 0;

      default :
	recGblRecordError (S_db_badField, (void *) pRec, "devPmacMbxStatus_init: Illegal INP field");
	return S_db_badField;
    }
  }
#endif	/* STATUS_RECORD */

#ifdef STATUS_RECORD
  /*******************************************************************************
   *
   * devPmacMbxStatus_read - EPICS PMAC device support status read
   *
   */
  static long devPmacMbxStatus_read (struct statusRecord *pRec) {
    char *MyName = "devPmacMbxStatus_read";
    /* long status; */

    /* used for breaking the response in to words */

    char buff[7];
    char *temp;
    long valL;



    PMAC_MBX_DPVT *pDpvt = (PMAC_MBX_DPVT *) pRec->dpvt;
    PMAC_MBX_IO   *pMbxIo = &pDpvt->MbxIo;

    temp=buff;

    if (pRec->pact) {

      PMAC_TRACE (2, PMAC_MESSAGE ("%s: %s \"Command  [%s] response [%s].\"\n", MyName, pRec->name, pMbxIo->command, pMbxIo->response);)

      if(pMbxIo->status == -1) { /*error in communication*/
	PMAC_MESSAGE ("%s : %s \"Communication problems.\"\n", MyName, pRec->name);
	recGblSetSevr (pRec, COMM_ALARM, INVALID_ALARM);
	return 0;
      }

      if(pMbxIo->status == -2) { /*PMAC reported an error*/
	PMAC_MESSAGE ("%s : %s \"PMAC reported an error code: %s.\"\n", MyName, pRec->name,pMbxIo->errmsg);
	recGblSetSevr (pRec, COMM_ALARM, INVALID_ALARM);
	return 0;
      }

      switch (pRec->inp.value.vmeio.signal) {
	case 0:
  	  sscanf (pMbxIo->response, "%ld", &valL);

  	  pRec->val = valL;
  	  pRec->udf = FALSE;

  	  return 0;
	case 1:
  	  memcpy(temp,pMbxIo->response, 6);
  	  *(temp+6)=0;
  	  valL=strtol(temp, NULL, 16);

  	  pRec->val = valL;
  	  pRec->udf = FALSE;

  	  return 0;
	case 2:
  	  memcpy(temp,pMbxIo->response+6, 6);
  	  *(temp+6)=0;
  	  valL=strtol(temp, NULL, 16);

  	  pRec->val = valL;
  	  pRec->udf = FALSE;		
  	  return 0;
	case 3:
  	  memcpy(temp,pMbxIo->response+12, 6);
  	  *(temp+6)=0;
  	  valL=strtol(temp, NULL, 16);

  	  pRec->val = valL;
  	  pRec->udf = FALSE;	
  	  return 0;
	default:
  	  return -1;
      }
    } else {
      sprintf (pMbxIo->command, "%s", pRec->inp.value.vmeio.parm);
      PMAC_TRACE (2, PMAC_MESSAGE ("%s: %s \"Command to send [%s].\"\n", MyName, pRec->name, pMbxIo->command);)

      if(drvPmacMbxScan (pMbxIo) < 0) {
	PMAC_MESSAGE ("%s: %s  \"PMAC device number %d is not configured!\"\n",MyName,pRec->name,pMbxIo->card);
	recGblSetSevr (pRec, COMM_ALARM, INVALID_ALARM);
      }

      pRec->pact = TRUE;
      return 0;
    }
  }
#endif	/* STATUS_RECORD */

/*******************************************************************************
 *
 * devPmacMbxBoWatchdog_init - EPICS PMAC device support bo watchdog init
 *
 */
static long devPmacMbxBoWatchdog_init (struct boRecord *pRec) {
  char *MyName = "devPmacMbxBoWatchdog_init";
  /* long status = 0; */

  switch (pRec->out.type) {
  case (VME_IO) :

    PMAC_DEBUG (1, PMAC_MESSAGE ("%s: %s: \"card %d parm %s.\"\n", MyName, pRec->name,
      pRec->out.value.vmeio.card, pRec->out.value.vmeio.parm);)

    pRec->dpvt = (void *) devPmacMbxDpvtInit ((struct dbCommon *) pRec, (int) pRec->out.value.vmeio.card);
    return 0;

  default :
    recGblRecordError (S_db_badField, (void *) pRec, "devPmacMbxBo_init: Illegal OUT field");
    return S_db_badField;
  }

}
/*******************************************************************************
 *
 * devPmacMbxBoWatchdog_write - EPICS PMAC device support bo watchdog write
 *
 */
static long devPmacMbxBoWatchdog_write (struct boRecord	*pRec) {
  char *MyName = "devPmacMbxBoWatchdog_write";
  /* long status; */

  PMAC_MBX_DPVT *pDpvt = (PMAC_MBX_DPVT *) pRec->dpvt;
  PMAC_MBX_IO   *pMbxIo = &pDpvt->MbxIo;

  if (pRec->pact) {

    PMAC_TRACE (2, PMAC_MESSAGE ("%s: %s \"Command  [%s] response [%s].\"\n", MyName, pRec->name, pMbxIo->command, pMbxIo->response);)
    if(pMbxIo->status == -1) { /*error in communication*/
      PMAC_MESSAGE ("%s : %s \"Communication problems.\"\n",MyName, pRec->name);
      recGblSetSevr (pRec, COMM_ALARM, INVALID_ALARM);
      return 0;
    }
    if(pMbxIo->status == -2) { /*PMAC reported an error*/
      PMAC_MESSAGE("%s : %s \"PMAC reported an error code: %s.\"\n", MyName, pRec->name,pMbxIo->errmsg);
      recGblSetSevr (pRec, COMM_ALARM, INVALID_ALARM);
      return 0;
    }
    return 0;
  } else {
    watchdog[pRec->out.value.vmeio.card]++;
    if(watchdog[pRec->out.value.vmeio.card] > 9999) watchdog[pRec->out.value.vmeio.card] = 0;

    sprintf (pMbxIo->command,"%s%ld", pRec->out.value.vmeio.parm, watchdog[pRec->out.value.vmeio.card]);

    PMAC_TRACE (2, PMAC_MESSAGE ("%s: %s \"Command to send [%s].\"\n", MyName, pRec->name, pMbxIo->command);)

    if(drvPmacMbxScan (pMbxIo) < 0) {
      PMAC_MESSAGE ("%s: %s  \"PMAC device number %d is not configured!\"\n",MyName,pRec->name,pMbxIo->card);
      recGblSetSevr (pRec, COMM_ALARM, INVALID_ALARM);
    }

    pRec->pact = TRUE;

    return 0;
  }
}

/*******************************************************************************
 *
 * devPmacMbxCallback - EPICS device support Callback
 *
 */
static void devPmacMbxCallback (CALLBACK *pCallback) {
  char            *MyName = "devPmacMbxCallback";
  /* long         status = 0; */

  struct dbCommon *pRec;
  struct rset     *pRset;

  callbackGetUser (pRec, pCallback);

  pRset = (struct rset *) (pRec->rset);

  PMAC_TRACE (3, PMAC_MESSAGE ("%s: %s \"CALLBACK.\"\n", MyName, pRec->name);)

  dbScanLock (pRec);
  (*(pRset->process))(pRec);
  dbScanUnlock (pRec);
  return;
}


