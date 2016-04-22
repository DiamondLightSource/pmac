/* devPmacRam.c -  EPICS Device Support for PMAC-VME */

/*
 * Author:      Thomas A. Coleman
 * Date:        97/05/06
 *
 *      Experimental Physics and Industrial Control System (EPICS)
 *
 * Modification History:
 * ---------------------
 * .01  6-7-95        tac     initial
 * .02  2-27-96       tac     initial
 * .03  7-24-96       wfl     ifdef'ed out use of status record
 * .04  2-27-04       oam     updated for epics 3.14.5
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
 * DESCRIPTION:
 * ------------
 * This module implements EPICS Device Support for PMAC-VME DPRAM.
 *
 */

/*
 * INCLUDES
 */

/* VxWorks Includes */

#include <vxWorks.h>
#include <stdlib.h>	 /* Sergey */
#include <types.h>
#include <stdioLib.h>
#include <string.h>
#define __PROTOTYPE_5_0		/* Sergey */
#include <logLib.h>	/* Sergey */

/* EPICS Includes */

#include <alarm.h>
#include <cvtTable.h>
#include <dbAccess.h>
#include <dbDefs.h>
#include <recSup.h>
#include <devSup.h>
#include <dbAccess.h>
#include <dbScan.h>
#include <link.h>
#include <module_types.h>
#include <callback.h>

#include <aiRecord.h>
#include <aoRecord.h>
#include <biRecord.h>
#include <boRecord.h>
#include <eventRecord.h>
#include <longinRecord.h>
#include <longoutRecord.h>
#include <mbbiRecord.h>
#include <mbboRecord.h>
#include <errlog.h>

/* local includes */

#ifdef STATUS_RECORD
#include <statusRecord.h>
#endif

#include "drvPmac.h"
#include "recGbl.h"
#include "epicsExport.h"
/*
 * DEFINES
 */

#define PMAC_DIAGNOSTICS TRUE
#define PMAC_PRIVATE FALSE

#if PMAC_PRIVATE
#define PMAC_LOCAL LOCAL
#else
#define PMAC_LOCAL
#endif

#if PMAC_DIAGNOSTICS
#define PMAC_MESSAGE	logMsg
#define PMAC_DEBUG(level,code)       { if (devPmacRamDebug >= (level)) { code } }
#define PMAC_TRACE(level,code)       { if ( (pRec->tpro >= (level)) || (devPmacRamDebug == (level)) ) { code } }
#else
#define PMAC_DEBUG(level,code)      ;
#define PMAC_TRACE(level,code)      ;
#endif

#define NO_ERR_STATUS   (-1)

/*
 * TYPEDEFS
 */
typedef struct  /* PMAC_DSET_AI */
{
  long  	  number;
  DEVSUPFUN	  report;
  DEVSUPFUN	  init;
  DEVSUPFUN	  init_record;
  DEVSUPFUN	  get_ioint_info;
  DEVSUPFUN	  read;
  DEVSUPFUN	  special_linconv;
} PMAC_DSET_AI;

typedef struct  /* PMAC_DSET_AO */
{
  long  	  number;
  DEVSUPFUN	  report;
  DEVSUPFUN	  init;
  DEVSUPFUN	  init_record;
  DEVSUPFUN	  get_ioint_info;
  DEVSUPFUN	  write;
  DEVSUPFUN	  special_linconv;
} PMAC_DSET_AO;

typedef struct  /* PMAC_DSET_BI */
{
  long  	  number;
  DEVSUPFUN	  report;
  DEVSUPFUN	  init;
  DEVSUPFUN	  init_record;
  DEVSUPFUN	  get_ioint_info;
  DEVSUPFUN	  read;
} PMAC_DSET_BI;

typedef struct  /* PMAC_DSET_BO */
{
  long  	  number;
  DEVSUPFUN	  report;
  DEVSUPFUN	  init;
  DEVSUPFUN	  init_record;
  DEVSUPFUN	  get_ioint_info;
  DEVSUPFUN	  write;
} PMAC_DSET_BO;

typedef struct  /* PMAC_DSET_EVENT */
{
  long  	  number;
  DEVSUPFUN	  report;
  DEVSUPFUN	  init;
  DEVSUPFUN	  init_record;
  DEVSUPFUN	  get_ioint_info;
  DEVSUPFUN	  read;
} PMAC_DSET_EVENT;

typedef struct  /* PMAC_DSET_LI */
{
  long  	  number;
  DEVSUPFUN	  report;
  DEVSUPFUN	  init;
  DEVSUPFUN	  init_record;
  DEVSUPFUN	  get_ioint_info;
  DEVSUPFUN	  read;
} PMAC_DSET_LI;

typedef struct  /* PMAC_DSET_LO */
{
  long  	  number;
  DEVSUPFUN	  report;
  DEVSUPFUN	  init;
  DEVSUPFUN	  init_record;
  DEVSUPFUN	  get_ioint_info;
  DEVSUPFUN	  write;
} PMAC_DSET_LO;

typedef struct  /* PMAC_DSET_MBBI */
{
  long  	  number;
  DEVSUPFUN	  report;
  DEVSUPFUN	  init;
  DEVSUPFUN	  init_record;
  DEVSUPFUN	  get_ioint_info;
  DEVSUPFUN	  read;
} PMAC_DSET_MBBI;

typedef struct  /* PMAC_DSET_MBBO */
{
  long  	  number;
  DEVSUPFUN	  report;
  DEVSUPFUN	  init;
  DEVSUPFUN	  init_record;
  DEVSUPFUN	  get_ioint_info;
  DEVSUPFUN	  write;
} PMAC_DSET_MBBO;

#ifdef STATUS_RECORD
typedef struct  /* PMAC_DSET_STATUS */
{
  long  	  number;
  DEVSUPFUN	  report;
  DEVSUPFUN	  init;
  DEVSUPFUN	  init_record;
  DEVSUPFUN	  get_ioint_info;
  DEVSUPFUN	  read;
} PMAC_DSET_STATUS;
#endif

typedef struct  /* PMAC_RAM_DATA */
{
  long  	  ramLong;
  double	  ramDouble;
} PMAC_RAM_DATA;

typedef struct  /* PMAC_RAM_DPVT */
{
  int		  card;
  PMAC_RAM_DATA   dpramData;
  PMAC_RAM_DATA   dpramDataPrev;
  PMAC_RAM_IO	  * pRamIo;
  IOSCANPVT	  ioscanpvt;
  CALLBACK	  callback;
  struct dbCommon *pRecord;	/* pointer to the record that owns this private */
  void  	(*process)();	/* callback to perform forward db processing */
  int		processPri;	/* process callback's priority */
} PMAC_RAM_DPVT;

/*
 * FORWARD DECLARATIONS
 */

LOCAL long devPmacRam_init();

LOCAL long devPmacRamAi_init();
LOCAL long devPmacRamAi_read();
LOCAL long devPmacRamAi_get_ioint_info();

LOCAL long devPmacRamAo_init();
LOCAL long devPmacRamAo_write();

LOCAL long devPmacRamBi_init();
LOCAL long devPmacRamBi_read();
LOCAL long devPmacRamBi_get_ioint_info();

LOCAL long devPmacRamBo_init();
LOCAL long devPmacRamBo_write();

LOCAL long devPmacRamEvent_init();
LOCAL long devPmacRamEvent_read();
LOCAL long devPmacRamEvent_get_ioint_info();

LOCAL long devPmacRamLi_init();
LOCAL long devPmacRamLi_read();
LOCAL long devPmacRamLi_get_ioint_info();

LOCAL long devPmacRamLo_init();
LOCAL long devPmacRamLo_write();

LOCAL long devPmacRamMbbi_init();
LOCAL long devPmacRamMbbi_read();
LOCAL long devPmacRamMbbi_get_ioint_info();

LOCAL long devPmacRamMbbo_init();
LOCAL long devPmacRamMbbo_write();

LOCAL long devPmacRamStatus_init();
LOCAL long devPmacRamStatus_read();
LOCAL long devPmacRamStatus_get_ioint_info();

/*
 * GLOBALS
 */

char * devPmacRamVersion = "@(#) devPmacRam.c 2.1 2004/02/27";

#if PMAC_DIAGNOSTICS
volatile int devPmacRamDebug = 0;
#endif

PMAC_DSET_AI devPmacRamAi =
{
  6,
  NULL,
  devPmacRam_init,
  devPmacRamAi_init,
  devPmacRamAi_get_ioint_info,
  devPmacRamAi_read,
  NULL
};

PMAC_DSET_AO devPmacRamAo =
{
  6,
  NULL,
  devPmacRam_init,
  devPmacRamAo_init,
  NULL,
  devPmacRamAo_write,
  NULL
};

PMAC_DSET_BI devPmacRamBi =
{
  5,
  NULL,
  devPmacRam_init,
  devPmacRamBi_init,
  devPmacRamBi_get_ioint_info,
  devPmacRamBi_read
};

PMAC_DSET_BO devPmacRamBo =
{
  5,
  NULL,
  devPmacRam_init,
  devPmacRamBo_init,
  NULL,
  devPmacRamBo_write
};

PMAC_DSET_EVENT devPmacRamEvent =
{
  5,
  NULL,
  devPmacRam_init,
  devPmacRamEvent_init,
  devPmacRamEvent_get_ioint_info,
  devPmacRamEvent_read
};

PMAC_DSET_LI devPmacRamLi =
{
  5,
  NULL,
  devPmacRam_init,
  devPmacRamLi_init,
  devPmacRamLi_get_ioint_info,
  devPmacRamLi_read
};

PMAC_DSET_LO devPmacRamLo =
{
  5,
  NULL,
  devPmacRam_init,
  devPmacRamLo_init,
  NULL,
  devPmacRamLo_write
};

PMAC_DSET_MBBI devPmacRamMbbi =
{
  5,
  NULL,
  devPmacRam_init,
  devPmacRamMbbi_init,
  devPmacRamMbbi_get_ioint_info,
  devPmacRamMbbi_read
};

PMAC_DSET_MBBO devPmacRamMbbo =
{
  5,
  NULL,
  devPmacRam_init,
  devPmacRamMbbo_init,
  NULL,
  devPmacRamMbbo_write
};

#ifdef STATUS_RECORD
PMAC_DSET_STATUS devPmacRamStatus =
{
  5,
  NULL,
  devPmacRam_init,
  devPmacRamStatus_init,
  devPmacRamStatus_get_ioint_info,
  devPmacRamStatus_read
};
epicsExportAddress(dset,devPmacRamStatus);
#endif

epicsExportAddress(dset,devPmacRamAi);
epicsExportAddress(dset,devPmacRamAo);
epicsExportAddress(dset,devPmacRamBi);
epicsExportAddress(dset,devPmacRamBo);
epicsExportAddress(dset,devPmacRamEvent);
epicsExportAddress(dset,devPmacRamLi);
epicsExportAddress(dset,devPmacRamLo);
epicsExportAddress(dset,devPmacRamMbbi);
epicsExportAddress(dset,devPmacRamMbbo);

/*
 * LOCALS
 */

/*******************************************************************************
 *
 * devPmacRam_init - EPICS device support init function
 *
 */
LOCAL long devPmacRam_init (int after) {
/* char *MyName = "devPmacRam_init"; */
  long status = 0;

  if (after == 1) {
    status = drvPmacStartup ();
  }

  return (status);
}


/*******************************************************************************
 *
 * devPmacRamUpdated - function called when driver has next value
 *
 */
void devPmacRamUpdated (void *pvoid) {
  char *MyName = "devPmacRamUpdated";
  /* int  i; */
  /* long data32; */

  PMAC_RAM_DPVT *pDpvt = (PMAC_RAM_DPVT *)pvoid;

  PMAC_RAM_IO	*pRamIo = pDpvt->pRamIo;

  PMAC_DEBUG (7,
    PMAC_MESSAGE ("%s: pDpvt=%010lx\n", MyName, pDpvt,0,0,0,0);
    PMAC_MESSAGE ("%s: RamIo valLong=%#010x valDouble=%lf\n", MyName, pRamIo->valLong, pRamIo->valDouble,0,0,0);
  )

  if ((pRamIo->valLong != pDpvt->dpramData.ramLong)
  	  || (pRamIo->valDouble != pDpvt->dpramData.ramDouble)) {
    pDpvt->dpramDataPrev.ramLong = pDpvt->dpramData.ramLong;
    pDpvt->dpramDataPrev.ramDouble = pDpvt->dpramData.ramDouble;

    pDpvt->dpramData.ramLong = pRamIo->valLong;
    pDpvt->dpramData.ramDouble = pRamIo->valDouble;

    scanIoRequest (pDpvt->ioscanpvt);
  }

  return;
}

/*******************************************************************************
 *
 * devPmacRamDpvtInit - EPICS PMAC_RAM_DPVT init
 *
 */
PMAC_RAM_DPVT *devPmacRamDpvtInit (struct dbCommon *pRec, int card) {
/* char *MyName = "devPmacRamDpvtInit"; */
  PMAC_RAM_DPVT *pDpvt;

  pDpvt = (PMAC_RAM_DPVT *) malloc (sizeof(PMAC_RAM_DPVT));
  pDpvt->pRecord = pRec;
  pDpvt->card = card;

  return (pDpvt);
}

/*******************************************************************************
 *
 * devPmacRamAi_init - EPICS device support init function for ai record
 *
 */
LOCAL long devPmacRamAi_init (struct aiRecord *pRec) {
  char *  MyName = "devPmacRamAi_init";
  long  	  status;
  PMAC_RAM_DPVT   *pDpvt;

  switch (pRec->inp.type) {
    case (VME_IO) :

      PMAC_DEBUG (1,
  	PMAC_MESSAGE ("%s: card %d signal %d parm %s\n", MyName, pRec->inp.value.vmeio.card, pRec->inp.value.vmeio.signal, pRec->inp.value.vmeio.parm,0,0);
      )

      pDpvt = devPmacRamDpvtInit ( (struct dbCommon *) pRec, (int) pRec->inp.value.vmeio.card );
      pRec->dpvt = (void *) pDpvt;

      scanIoInit (&pDpvt->ioscanpvt);

      status = drvPmacDpramRequest ( pRec->inp.value.vmeio.card, pRec->inp.value.vmeio.signal, pRec->inp.value.vmeio.parm, devPmacRamUpdated, pRec->dpvt, &pDpvt->pRamIo );
      if (!RTN_SUCCESS(status)) {
  	errPrintf (status, __FILE__, __LINE__, "%s: Unsuccessful DPRAM request - card %d pmacAdr %s.",
  	  MyName, pRec->inp.value.vmeio.card, pRec->inp.value.vmeio.parm);
  	return (status);
      }

      break;

    default :
      recGblRecordError(S_db_badField,(void *)pRec, "devPmacRamAi_init: Illegal INP field");
      return(S_db_badField);
  }
  return(0);
}

/*******************************************************************************
 *
 * devPmacRamBi_init - EPICS device support init function for bi record
 *
 */
LOCAL long devPmacRamBi_init (struct biRecord *pRec) {
  char          *MyName = "devPmacRamBi_init";
  long  	status;
  PMAC_RAM_DPVT *pDpvt;

  switch (pRec->inp.type) {
    case (VME_IO) :

      PMAC_DEBUG (1,
  	PMAC_MESSAGE ("%s: card %d signal %d parm %s\n", MyName, pRec->inp.value.vmeio.card,
	  pRec->inp.value.vmeio.signal, pRec->inp.value.vmeio.parm,0,0);
      )

      pDpvt = devPmacRamDpvtInit ( (struct dbCommon *) pRec, (int) pRec->inp.value.vmeio.card );
      pRec->dpvt = (void *) pDpvt;

      scanIoInit (&pDpvt->ioscanpvt);

      status = drvPmacDpramRequest ( pRec->inp.value.vmeio.card, pRec->inp.value.vmeio.signal,
        pRec->inp.value.vmeio.parm, devPmacRamUpdated, pRec->dpvt, &pDpvt->pRamIo );
      if (!RTN_SUCCESS(status)) {
  	errPrintf (status, __FILE__, __LINE__, "%s: Unsuccessful DPRAM request - card %d pmacAdr %s.",
  	  MyName, pRec->inp.value.vmeio.card, pRec->inp.value.vmeio.parm);
  	return (status);
      }

      break;

    default :
      recGblRecordError(S_db_badField,(void *)pRec,
      "devPmacRamBi_init: Illegal INP field");
      return(S_db_badField);
  }
  return(0);
}

/*******************************************************************************
 *
 * devPmacRamEvent_init - EPICS device support init function for event record
 *
 */
LOCAL long devPmacRamEvent_init (struct eventRecord *pRec) {
  char *  MyName = "devPmacRamEvent_init";
  long  	  status;
  PMAC_RAM_DPVT * pDpvt;

  switch (pRec->inp.type) {
    case (VME_IO) :

      PMAC_DEBUG (1,
  	PMAC_MESSAGE ("%s: card %d signal %d parm %s\n", MyName, pRec->inp.value.vmeio.card,
  	  pRec->inp.value.vmeio.signal, pRec->inp.value.vmeio.parm, 0, 0);
      )

      pDpvt = devPmacRamDpvtInit ( (struct dbCommon *) pRec,
  	  			      (int) pRec->inp.value.vmeio.card );
      pRec->dpvt = (void *) pDpvt;

      scanIoInit (&pDpvt->ioscanpvt);

      status = drvPmacDpramRequest (pRec->inp.value.vmeio.card,
  	pRec->inp.value.vmeio.signal, pRec->inp.value.vmeio.parm,
  	devPmacRamUpdated, pRec->dpvt, &pDpvt->pRamIo);
      if (!RTN_SUCCESS(status)) {
  	 errPrintf (status, __FILE__, __LINE__, "%s: Unsuccessful DPRAM request - card %d pmacAdr %s.",
  	   MyName, pRec->inp.value.vmeio.card, pRec->inp.value.vmeio.parm);
  	 return (status);
      }

      break;

    default :
      recGblRecordError(S_db_badField,(void *)pRec,
      "devPmacRamEvent_init: Illegal INP field");
      return(S_db_badField);
  }

  return(0);
}

/*******************************************************************************
 *
 * devPmacRamLi_init - EPICS device support init function for longin record
 *
 */
LOCAL long devPmacRamLi_init (struct longinRecord *pRec) {
  char *  MyName = "devPmacRamLi_init";
  long  	  status;
  PMAC_RAM_DPVT * pDpvt;

  switch (pRec->inp.type) {
  case (VME_IO) :

    PMAC_DEBUG (1,
      PMAC_MESSAGE ("%s: card %d signal %d parm %s\n", MyName, pRec->inp.value.vmeio.card,
  	pRec->inp.value.vmeio.signal, pRec->inp.value.vmeio.parm, 0, 0);
    )

    pDpvt = devPmacRamDpvtInit ( (struct dbCommon *) pRec,
  	  			    (int) pRec->inp.value.vmeio.card );
    pRec->dpvt = (void *) pDpvt;

    scanIoInit (&pDpvt->ioscanpvt);

    status = drvPmacDpramRequest ( pRec->inp.value.vmeio.card,
      pRec->inp.value.vmeio.signal, pRec->inp.value.vmeio.parm,
      devPmacRamUpdated, pRec->dpvt, &pDpvt->pRamIo);
    if (!RTN_SUCCESS(status)) {
      errPrintf (status, __FILE__, __LINE__,
  	"%s: Unsuccessful DPRAM request - card %d pmacAdr %s.",
  	MyName, pRec->inp.value.vmeio.card, pRec->inp.value.vmeio.parm);
      return (status);
    }

    break;

  default :
    recGblRecordError(S_db_badField,(void *)pRec,
    "devPmacRamLi_init: Illegal INP field");
    return(S_db_badField);
  }

  return(0);
}

/*******************************************************************************
 *
 * devPmacRamMbbi_init - EPICS device support init function for mbbi record
 *
 */
LOCAL long devPmacRamMbbi_init (struct mbbiRecord *pRec) {
  char          *MyName = "devPmacRamMbbi_init";
  long          status;
  PMAC_RAM_DPVT *pDpvt;

  switch (pRec->inp.type) {
    case (VME_IO) :

      PMAC_DEBUG (1,
  	PMAC_MESSAGE ("%s: card %d signal %d parm %s\n",
  	  MyName, pRec->inp.value.vmeio.card,
  	  pRec->inp.value.vmeio.signal, pRec->inp.value.vmeio.parm,0,0);
      )

      pDpvt = devPmacRamDpvtInit ( (struct dbCommon *) pRec,
  	  			      (int) pRec->inp.value.vmeio.card );
      pRec->dpvt = (void *) pDpvt;

      scanIoInit (&pDpvt->ioscanpvt);

      status = drvPmacDpramRequest ( pRec->inp.value.vmeio.card,
  	pRec->inp.value.vmeio.signal, pRec->inp.value.vmeio.parm,
  	devPmacRamUpdated, pRec->dpvt, &pDpvt->pRamIo);
      if (!RTN_SUCCESS(status)) {
  	errPrintf (status, __FILE__, __LINE__, "%s: Unsuccessful DPRAM request - card %d pmacAdr %s.",
  	  MyName, pRec->inp.value.vmeio.card, pRec->inp.value.vmeio.parm);
  	return (status);
      }

      break;

    default :
      recGblRecordError(S_db_badField,(void *)pRec, "devPmacRamMbbi_init: Illegal INP field");
      return(S_db_badField);
  }

  return(0);
}

#ifdef STATUS_RECORD
/*******************************************************************************
 *
 * devPmacRamStatus_init - EPICS device support init function for status record
 *
 */
LOCAL long devPmacRamStatus_init (struct statusRecord *pRec) {
  char *  MyName = "devPmacRamStatus_init";
  long  	  status;
  PMAC_RAM_DPVT * pDpvt;

  switch (pRec->inp.type) {
    case (VME_IO) :

      PMAC_DEBUG (1,
  	PMAC_MESSAGE ("%s: card %d signal %d parm %s\n", MyName, pRec->inp.value.vmeio.card,
  	  pRec->inp.value.vmeio.signal, pRec->inp.value.vmeio.parm, 0, 0);
      )

      pDpvt = devPmacRamDpvtInit ( (struct dbCommon *) pRec, (int) pRec->inp.value.vmeio.card );
      pRec->dpvt = (void *) pDpvt;

      scanIoInit (&pDpvt->ioscanpvt);

      status = drvPmacDpramRequest ( pRec->inp.value.vmeio.card,
  	pRec->inp.value.vmeio.signal, pRec->inp.value.vmeio.parm,
  	devPmacRamUpdated, pRec->dpvt, &pDpvt->pRamIo);
      if (!RTN_SUCCESS(status)) {
  	errPrintf (status, __FILE__, __LINE__, "%s: Unsuccessful DPRAM request - card %d pmacAdr %s.",
  	  MyName, pRec->inp.value.vmeio.card, pRec->inp.value.vmeio.parm);
  	return (status);
      }

      break;

    default :
      recGblRecordError(S_db_badField,(void *)pRec, "devPmacRamStatus_init: Illegal INP field");
      return(S_db_badField);
  }

  return(0);
}
#endif	/* STATUS_RECORD */

/*******************************************************************************
 *
 * devPmacRamAo_init - EPICS device support init function for ao record
 *
 */
LOCAL long devPmacRamAo_init (struct aoRecord *pRec) {
  char          *MyName = "devPmacRamAo_init";
  long  	status;
  PMAC_RAM_DPVT *pDpvt;

  switch (pRec->out.type) {
  case (VME_IO) :

    PMAC_DEBUG (1,
      PMAC_MESSAGE ("%s: card %d signal %d parm %s\n", MyName, pRec->out.value.vmeio.card, pRec->out.value.vmeio.signal, pRec->out.value.vmeio.parm,0,0);
    )

    pDpvt = devPmacRamDpvtInit ( (struct dbCommon *) pRec, (int) pRec->out.value.vmeio.card );
    pRec->dpvt = (void *) pDpvt;

    status = drvPmacDpramRequest ( pRec->out.value.vmeio.card, pRec->out.value.vmeio.signal,
      pRec->out.value.vmeio.parm, (void *) NULL, (void *) NULL, &pDpvt->pRamIo);
    if (!RTN_SUCCESS(status)) {
      errPrintf (status, __FILE__, __LINE__, "%s: Unsuccessful DPRAM request - card %d pmacAdr %s.",
  	MyName, pRec->out.value.vmeio.card, pRec->out.value.vmeio.parm);
      return (status);
    }

    break;

  default :
    recGblRecordError(S_db_badField,(void *)pRec, "devPmacRamAo_init: Illegal OUT field");
    return(S_db_badField);
  }

  return(0);
}

/*******************************************************************************
 *
 * devPmacRamBo_init - EPICS device support init function for bo record
 *
 */
LOCAL long devPmacRamBo_init (struct boRecord *pRec) {
  char          *MyName = "devPmacRamBo_init";
  long  	status;
  PMAC_RAM_DPVT *pDpvt;

  switch (pRec->out.type) {
    case (VME_IO) :

      PMAC_DEBUG (1,
  	PMAC_MESSAGE ("%s: card %d signal %d parm %s\n",
  	  MyName, pRec->out.value.vmeio.card, pRec->out.value.vmeio.signal, pRec->out.value.vmeio.parm,0,0);
      )

      pDpvt = devPmacRamDpvtInit ( (struct dbCommon *) pRec,
  	  			      (int) pRec->out.value.vmeio.card );
      pRec->dpvt = (void *) pDpvt;

      status = drvPmacDpramRequest ( pRec->out.value.vmeio.card,
  	pRec->out.value.vmeio.signal,
  	pRec->out.value.vmeio.parm,
  	(void *) NULL, (void *) NULL,
  	&pDpvt->pRamIo);
      if (!RTN_SUCCESS(status)) {
  	errPrintf (status, __FILE__, __LINE__, "%s: Unsuccessful DPRAM request - card %d pmacAdr %s.",
  	  MyName, pRec->out.value.vmeio.card, pRec->out.value.vmeio.parm);
  	return (status);
      }

      break;

    default :
      recGblRecordError(S_db_badField,(void *)pRec, "devPmacRamBo_init: Illegal OUT field");
      return(S_db_badField);
  }

  return(0);
}

/*******************************************************************************
 *
 * devPmacRamLo_init - EPICS device support init function for longout record
 *
 */
LOCAL long devPmacRamLo_init (struct longoutRecord *pRec) {
  char          *MyName = "devPmacRamLo_init";
  long  	status;
  PMAC_RAM_DPVT *pDpvt;

  switch (pRec->out.type) {
    case (VME_IO) :

      PMAC_DEBUG (1,
  	PMAC_MESSAGE ("%s: card %d signal %d parm %s\n",
  	  MyName, pRec->out.value.vmeio.card, pRec->out.value.vmeio.signal, pRec->out.value.vmeio.parm, 0, 0);
      )

      pDpvt = devPmacRamDpvtInit ( (struct dbCommon *) pRec, (int) pRec->out.value.vmeio.card );
      pRec->dpvt = (void *) pDpvt;

      status = drvPmacDpramRequest ( pRec->out.value.vmeio.card,
  	pRec->out.value.vmeio.signal, pRec->out.value.vmeio.parm,
  	(void *) NULL, (void *) NULL, &pDpvt->pRamIo);
      if (!RTN_SUCCESS(status)) {
  	errPrintf (status, __FILE__, __LINE__, "%s: Unsuccessful DPRAM request - card %d pmacAdr %s.",
  	  MyName, pRec->out.value.vmeio.card, pRec->out.value.vmeio.parm);
  	return (status);
      }

      break;

    default :
      recGblRecordError(S_db_badField,(void *)pRec, "devPmacRamLo_init: Illegal OUT field");
      return(S_db_badField);
  }

  return(0);
}

/*******************************************************************************
 *
 * devPmacRamMbbo_init - EPICS device support init function for mbbo record
 *
 */
LOCAL long devPmacRamMbbo_init (struct mbboRecord *pRec) {
  char *  MyName = "devPmacRamMbbo_init";
  long  	  status;
  PMAC_RAM_DPVT * pDpvt;

  switch (pRec->out.type) {
  case (VME_IO) :

    PMAC_DEBUG (1,
      PMAC_MESSAGE ("%s: card %d signal %d parm %s\n", MyName, pRec->out.value.vmeio.card,
  	pRec->out.value.vmeio.signal, pRec->out.value.vmeio.parm,0,0);
    )

    pDpvt = devPmacRamDpvtInit ( (struct dbCommon *) pRec, (int) pRec->out.value.vmeio.card );
    pRec->dpvt = (void *) pDpvt;

    status = drvPmacDpramRequest ( pRec->out.value.vmeio.card, pRec->out.value.vmeio.signal,
      pRec->out.value.vmeio.parm, (void *) NULL, (void *) NULL, &pDpvt->pRamIo);
    if (!RTN_SUCCESS(status)) {
      errPrintf (status, __FILE__, __LINE__, "%s: Unsuccessful DPRAM request - card %d pmacAdr %s.",
  	MyName, pRec->out.value.vmeio.card, pRec->out.value.vmeio.parm);
      return (status);
    }

    break;

  default :
    recGblRecordError(S_db_badField,(void *)pRec, "devPmacRamMbbo_init: Illegal OUT field");
    return(S_db_badField);
  }

  return(0);
}

/*******************************************************************************
 *
 * devPmacRamAi_get_ioint_info - EPICS device support get_ioint_info function for ai record
 *
 */
LOCAL long devPmacRamAi_get_ioint_info (int cmd, struct aiRecord *pRec, IOSCANPVT *ppvt) {
  PMAC_RAM_DPVT *pDpvt;

  pDpvt = (PMAC_RAM_DPVT *) pRec->dpvt;
  *ppvt = pDpvt->ioscanpvt;
  return(0);
}

/*******************************************************************************
 *
 * devPmacRamBi_get_ioint_info - EPICS device support get_ioint_info function for bi record
 *
 */
LOCAL long devPmacRamBi_get_ioint_info (int cmd, struct biRecord *pRec, IOSCANPVT *ppvt) {
  PMAC_RAM_DPVT *pDpvt;

  pDpvt = (PMAC_RAM_DPVT *) pRec->dpvt;
  *ppvt = pDpvt->ioscanpvt;
  return(0);
}

/*******************************************************************************
 *
 * devPmacRamEvent_get_ioint_info - EPICS device support get_ioint_info function for event record
 *
 */
LOCAL long devPmacRamEvent_get_ioint_info (int cmd, struct eventRecord *pRec, IOSCANPVT *ppvt) {
  PMAC_RAM_DPVT *pDpvt;

  pDpvt = (PMAC_RAM_DPVT *) pRec->dpvt;
  *ppvt = pDpvt->ioscanpvt;
  return(0);
}


/*******************************************************************************
 *
 * devPmacRamLi_get_ioint_info - EPICS device support get_ioint_info function for longin record
 *
 */
LOCAL long devPmacRamLi_get_ioint_info (int cmd, struct longinRecord *pRec, IOSCANPVT *ppvt) {
  PMAC_RAM_DPVT *pDpvt;

  pDpvt = (PMAC_RAM_DPVT *) pRec->dpvt;
  *ppvt = pDpvt->ioscanpvt;
  return(0);
}

/*******************************************************************************
 *
 * devPmacRamMbbi_get_ioint_info - EPICS device support get_ioint_info function for mbbi record
 *
 */
LOCAL long devPmacRamMbbi_get_ioint_info (int cmd, struct mbbiRecord *pRec, IOSCANPVT *ppvt) {
  PMAC_RAM_DPVT *pDpvt;

  pDpvt = (PMAC_RAM_DPVT *) pRec->dpvt;
  *ppvt = pDpvt->ioscanpvt;
  return(0);
}

#ifdef STATUS_RECORD
/*******************************************************************************
 *
 * devPmacRamStatus_get_ioint_info - EPICS device support get_ioint_info function for status record
 *
 */
LOCAL long devPmacRamStatus_get_ioint_info (int cmd, struct statusRecord *pRec, IOSCANPVT *ppvt) {
  /* char *MyName = "devPmacRamStatus_get_ioint_info"; */
  PMAC_RAM_DPVT   *pDpvt;

  pDpvt = (PMAC_RAM_DPVT *) pRec->dpvt;
  *ppvt = pDpvt->ioscanpvt;
  return(0);
}
#endif	/* STATUS_RECORD */

/*******************************************************************************
 *
 * devPmacRamAi_read - EPICS device support read function for ai record
 *
 */
LOCAL long devPmacRamAi_read (struct aiRecord *pRec) {
/* char *MyName = "devPmacRamAi_read"; */
/* long status; */
  PMAC_RAM_DPVT   *pDpvt;
  double  val;

  pDpvt = (PMAC_RAM_DPVT *) pRec->dpvt;

  /* Raw Value */
  val = pDpvt->dpramData.ramDouble;

  /* this is for raw timer value */
  if ((val >= -2147483648.0) && (val < 2147483647.1)) pRec->rval = (int) val;

  /* Adjust Slope And Offset */
  if (pRec->aslo != 0.0) {
    val *= pRec->aslo;
  }
  if (pRec->aoff != 0.0) {
    val += pRec->aoff;
  }

  /* pRec->linr Conversion Ignored */

  /* Apply Smoothing Algorithm */
  if (pRec->smoo != 0.0) {
    if (pRec->init == TRUE) pRec->val = val;	/* initial condition */
    pRec->val = val * (1.00 - pRec->smoo) + (pRec->val * pRec->smoo);
  } else {
    pRec->val = val;
  }

  pRec->udf = FALSE;
  return (2);
}

/*******************************************************************************
 *
 * devPmacRamBi_read - EPICS device support read function for bi record
 *
 */
LOCAL long devPmacRamBi_read (struct biRecord *pRec) {
/* char *MyName = "devPmacRamBi_read"; */
/* long status; */
  PMAC_RAM_DPVT   *pDpvt;

  pDpvt = (PMAC_RAM_DPVT *) pRec->dpvt;

  pRec->rval = (unsigned long) pDpvt->dpramData.ramLong;
  pRec->udf = FALSE;

  return (0);
}

/*******************************************************************************
 *
 * devPmacRamEvent_read - EPICS device support read function for event record
 *
 */
LOCAL long devPmacRamEvent_read (struct eventRecord *pRec) {
/* char *MyName = "devPmacRamEvent_read"; */
/* long status; */
  PMAC_RAM_DPVT *pDpvt;

  pDpvt = (PMAC_RAM_DPVT *) pRec->dpvt;

  pRec->val = (short) (0x0000ffff & pDpvt->dpramData.ramLong);
  pRec->udf = FALSE;

  return(0);
}

/*******************************************************************************
 *
 * devPmacRamLi_read - EPICS device support read function for longin record
 *
 */
LOCAL long devPmacRamLi_read (struct longinRecord *pRec) {
/* char *MyName = "devPmacRamLi_read"; */
/* long status; */
  PMAC_RAM_DPVT *pDpvt;

  pDpvt = (PMAC_RAM_DPVT *) pRec->dpvt;

  pRec->val = pDpvt->dpramData.ramLong;
  pRec->udf = FALSE;

  return(0);
}

/*******************************************************************************
 *
 * devPmacRamMbbi_read - EPICS device support read function for mbbi record
 *
 */
LOCAL long devPmacRamMbbi_read (struct mbbiRecord *pRec) {
/* char *MyName = "devPmacRamMbbi_read"; */
/* long status; */
  PMAC_RAM_DPVT *pDpvt;

  pDpvt = (PMAC_RAM_DPVT *) pRec->dpvt;

  pRec->rval = (unsigned long) pDpvt->dpramData.ramLong;
  pRec->udf = FALSE;

  return(0);
}

#ifdef STATUS_RECORD
/*******************************************************************************
 *
 * devPmacRamStatus_read - EPICS device support read function for status record
 *
 */
LOCAL long devPmacRamStatus_read (struct statusRecord *pRec) {
/* char *MyName = "devPmacRamStatus_read"; */
/* long status; */
  PMAC_RAM_DPVT *pDpvt;

  pDpvt = (PMAC_RAM_DPVT *) pRec->dpvt;

  pRec->val = pDpvt->dpramData.ramLong;
  pRec->udf = FALSE;

  return(0);
}
#endif	/* STATUS_RECORD */

/*******************************************************************************
 *
 * devPmacRamAo_write - EPICS device support write function for ao record
 *
 */
LOCAL long devPmacRamAo_write (struct aoRecord *pRec) {
/* char *MyName = "devPmacRamAo_write"; */
/* long status; */
  PMAC_RAM_DPVT   *pDpvt;
  double val;

  pDpvt = (PMAC_RAM_DPVT *) pRec->dpvt;

  /* Output Value */
  val = (double) pRec->oval;
  	  
  /* Adjust Slope And Offset */
  if (pRec->aoff != 0.0) {
    val -= (double) pRec->aoff;
  }
  if (pRec->aslo != 0.0) {
    val /= (double) pRec->aslo;
  }

  /* pRec->linr Conversion Ignored */

  pDpvt->pRamIo->valDouble = val; 
  pDpvt->pRamIo->valLong = (long) val;
  drvPmacRamPutData (pDpvt->pRamIo);

  return(0);
}

/*******************************************************************************
 *
 * devPmacRamBo_write - EPICS device support write function for bo record
 *
 */
LOCAL long devPmacRamBo_write (struct boRecord *pRec) {
/* char *MyName = "devPmacRamBo_write"; */
/* long status; */
  PMAC_RAM_DPVT *pDpvt;

  pDpvt = (PMAC_RAM_DPVT *) pRec->dpvt;

  pDpvt->pRamIo->valLong = (long) pRec->val;
  pDpvt->pRamIo->valDouble = (double) pDpvt->pRamIo->valLong;
  drvPmacRamPutData (pDpvt->pRamIo);

  return(0);
}

/*******************************************************************************
 *
 * devPmacRamLo_write - EPICS device support write function for longout record
 *
 */
LOCAL long devPmacRamLo_write (struct longoutRecord *pRec) {
/* char *MyName = "devPmacRamLo_write"; */
/* long status; */
  PMAC_RAM_DPVT *pDpvt;

  pDpvt = (PMAC_RAM_DPVT *) pRec->dpvt;

  pDpvt->pRamIo->valLong = (long) pRec->val;
  pDpvt->pRamIo->valDouble = (double) pDpvt->pRamIo->valLong;
  drvPmacRamPutData (pDpvt->pRamIo);

  return(0);
}

/*******************************************************************************
 *
 * devPmacRamMbbo_write - EPICS device support write function for mbbo record
 *
 */
LOCAL long devPmacRamMbbo_write (struct mbboRecord *pRec) {
/* char *MyName = "devPmacRamMbbo_write"; */
/* long status; */
  PMAC_RAM_DPVT *pDpvt;

  pDpvt = (PMAC_RAM_DPVT *) pRec->dpvt;

  pDpvt->pRamIo->valLong = (long) pRec->rval;
  pDpvt->pRamIo->valDouble = (double) pDpvt->pRamIo->valLong;
  drvPmacRamPutData (pDpvt->pRamIo);

  return(0);
}

/*******************************************************************************
 *
 * devPmacRamDpvtShow - EPICS report device private area
 *
 */
long devPmacRamDpvtShow (PMAC_RAM_DPVT *pDpvt) {
  char *MyName = "devPmacRamDpvtShow";

  printf ("%s: Device Private for record name %s\n", MyName, pDpvt->pRecord->name);
  printf ("%s: dpramDataPrev\tramLong %#010lx\tramDouble %f\n", MyName, pDpvt->dpramDataPrev.ramLong, pDpvt->dpramDataPrev.ramDouble);
  printf ("%s: dpramData\t\tramLong %#010lx\tramDouble %f\n", MyName, pDpvt->dpramData.ramLong, pDpvt->dpramData.ramDouble);
  printf ("%s: pRamIo\t\tvalLong %#010lx\tvalDouble %f\n", MyName, pDpvt->pRamIo->valLong, pDpvt->pRamIo->valDouble);

  return (0);
}		
