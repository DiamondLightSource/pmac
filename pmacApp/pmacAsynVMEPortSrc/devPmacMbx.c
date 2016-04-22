/* devPmacMbx.c -  EPICS Device Support for PMAC-VME Mailbox */

/*
 * Author:      Thomas A. Coleman
 * Date:        97/05/06
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
 * .01  6-7-95        tac     initial
 * .02  7-24-96       wfl     ifdef'ed out use of status record
 * .03  7-03-97       wfl     ifdef'ed out use of load record
 * 2.1  2-27-04       oam     updated for epics 3.14.5
 */

/*
 * DESCRIPTION:
 * ------------
 * This module implements EPICS Device Support for PMAC-VME Mailbox.
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
#include <dbScan.h>
#include <link.h>
#include <module_types.h>
#include <callback.h>
#include <cantProceed.h>

#include <aiRecord.h>
#include <biRecord.h>
#include <longinRecord.h>
#include <mbbiRecord.h>
#ifdef STATUS_RECORD
#include <statusRecord.h>
#endif
#include <aoRecord.h>
#include <boRecord.h>
#include <longoutRecord.h>
#include <mbboRecord.h>
#include <stringinRecord.h>
#include <stringoutRecord.h>
#include <waveformRecord.h>
#ifdef LOAD_RECORD
#include <loadRecord.h>
#endif

/* local includes */

#include <drvPmac.h>
#include <pmacError.h>
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
#define PMAC_DEBUG(level,code)       { if (devPmacMbxDebug >= (level)) { code } }
#define PMAC_TRACE(level,code)       { if ( (pRec->tpro >= (level)) || (devPmacMbxDebug == (level)) ) { code } }
#else
#define PMAC_DEBUG(level,code)      ;
#define PMAC_TRACE(level,code)      ;
#endif

#define NO_ERR_STATUS   (-1)

/*
 * TYPEDEFS
 */
typedef struct {  /* PMAC_DSET_AI */
  long	   number;
  DEVSUPFUN report;
  DEVSUPFUN init;
  DEVSUPFUN init_record;
  DEVSUPFUN get_ioint_info;
  DEVSUPFUN read;
  DEVSUPFUN special_linconv;
} PMAC_DSET_AI;

typedef struct {  /* PMAC_DSET_AO */
  long	   number;
  DEVSUPFUN report;
  DEVSUPFUN init;
  DEVSUPFUN init_record;
  DEVSUPFUN get_ioint_info;
  DEVSUPFUN write;
  DEVSUPFUN special_linconv;
} PMAC_DSET_AO;

typedef struct {  /* PMAC_DSET_BI */
  long	   number;
  DEVSUPFUN report;
  DEVSUPFUN init;
  DEVSUPFUN init_record;
  DEVSUPFUN get_ioint_info;
  DEVSUPFUN read;
} PMAC_DSET_BI;

typedef struct {  /* PMAC_DSET_BO */
  long	   number;
  DEVSUPFUN report;
  DEVSUPFUN init;
  DEVSUPFUN init_record;
  DEVSUPFUN get_ioint_info;
  DEVSUPFUN write;
} PMAC_DSET_BO;

typedef struct {  /* PMAC_DSET_LI */
  long	   number;
  DEVSUPFUN report;
  DEVSUPFUN init;
  DEVSUPFUN init_record;
  DEVSUPFUN get_ioint_info;
  DEVSUPFUN read;
} PMAC_DSET_LI;

typedef struct {  /* PMAC_DSET_LO */
  long	   number;
  DEVSUPFUN report;
  DEVSUPFUN init;
  DEVSUPFUN init_record;
  DEVSUPFUN get_ioint_info;
  DEVSUPFUN write;
} PMAC_DSET_LO;

typedef struct {  /* PMAC_DSET_MBBI */
  long	   number;
  DEVSUPFUN report;
  DEVSUPFUN init;
  DEVSUPFUN init_record;
  DEVSUPFUN get_ioint_info;
  DEVSUPFUN read;
} PMAC_DSET_MBBI;

typedef struct {  /* PMAC_DSET_MBBO */
  long	   number;
  DEVSUPFUN report;
  DEVSUPFUN init;
  DEVSUPFUN init_record;
  DEVSUPFUN get_ioint_info;
  DEVSUPFUN write;
} PMAC_DSET_MBBO;

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

typedef struct {  /* PMAC_DSET_SI */
  long      number;
  DEVSUPFUN report;
  DEVSUPFUN init;
  DEVSUPFUN init_record;
  DEVSUPFUN get_ioint_info;
  DEVSUPFUN read;
} PMAC_DSET_SI;

typedef struct {  /* PMAC_DSET_SO */
  long	   number;
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
  DEVSUPFUN write;
} PMAC_DSET_WF;

#ifdef LOAD_RECORD
typedef struct {  /* PMAC_DSET_LOAD */
  long      number;
  DEVSUPFUN report;
  DEVSUPFUN init;
  DEVSUPFUN init_record;
  DEVSUPFUN get_ioint_info;
  DEVSUPFUN proc;
} PMAC_DSET_LOAD;
#endif


typedef struct {  /* PMAC_MBX_DPVT */
  PMAC_MBX_IO	MbxIo;
} PMAC_MBX_DPVT;

/*
 * FORWARD DECLARATIONS
 */

LOCAL long devPmacMbx_init();

LOCAL long devPmacMbxAi_init();
LOCAL long devPmacMbxAi_read();

LOCAL long devPmacMbxAo_init();
LOCAL long devPmacMbxAo_write();

LOCAL long devPmacMbxBi_init();
LOCAL long devPmacMbxBi_read();

LOCAL long devPmacMbxBo_init();
LOCAL long devPmacMbxBo_write();

LOCAL long devPmacMbxLi_init();
LOCAL long devPmacMbxLi_read();

LOCAL long devPmacMbxLo_init();
LOCAL long devPmacMbxLo_write();

LOCAL long devPmacMbxMbbi_init();
LOCAL long devPmacMbxMbbi_read();

LOCAL long devPmacMbxMbbo_init();
LOCAL long devPmacMbxMbbo_write();

#ifdef STATUS_RECORD
LOCAL long devPmacMbxStatus_init();
LOCAL long devPmacMbxStatus_read();
#endif

LOCAL long devPmacMbxSi_init();
LOCAL long devPmacMbxSi_read();

LOCAL long devPmacMbxSo_init();
LOCAL long devPmacMbxSo_write();

LOCAL long devPmacMbxWf_init();
LOCAL long devPmacMbxWf_write();

#ifdef LOAD_RECORD
LOCAL long devPmacMbxLoad_init();
LOCAL long devPmacMbxLoad_proc();
#endif

void drvPmacMbxScan (PMAC_MBX_IO * pMbxIo);
void drvPmacFldScan (PMAC_MBX_IO * pMbxIo);

LOCAL void devPmacMbxCallback (CALLBACK * pCallback);

/*
 * GLOBALS
 */

char * devPmacMbxVersion = "@(#) devPmacMbx.c 2.1 2004/02/27";

#if PMAC_DIAGNOSTICS
  volatile int devPmacMbxDebug = 0;
#endif

PMAC_DSET_AI devPmacMbxAi = {
  6,
  NULL,
  devPmacMbx_init,
  devPmacMbxAi_init,
  NULL,
  devPmacMbxAi_read,
  NULL
};

PMAC_DSET_AO devPmacMbxAo = {
  6,
  NULL,
  devPmacMbx_init,
  devPmacMbxAo_init,
  NULL,
  devPmacMbxAo_write,
  NULL
};

PMAC_DSET_BI devPmacMbxBi = {
  5,
  NULL,
  devPmacMbx_init,
  devPmacMbxBi_init,
  NULL,
  devPmacMbxBi_read
};

PMAC_DSET_BO devPmacMbxBo = {
  5,
  NULL,
  devPmacMbx_init,
  devPmacMbxBo_init,
  NULL,
  devPmacMbxBo_write
};

PMAC_DSET_LI devPmacMbxLi = {
  5,
  NULL,
  devPmacMbx_init,
  devPmacMbxLi_init,
  NULL,
  devPmacMbxLi_read
};

PMAC_DSET_LO devPmacMbxLo = {
  5,
  NULL,
  devPmacMbx_init,
  devPmacMbxLo_init,
  NULL,
  devPmacMbxLo_write
};

PMAC_DSET_MBBI devPmacMbxMbbi = {
  5,
  NULL,
  devPmacMbx_init,
  devPmacMbxMbbi_init,
  NULL,
  devPmacMbxMbbi_read
};

PMAC_DSET_MBBO devPmacMbxMbbo = {
  5,
  NULL,
  devPmacMbx_init,
  devPmacMbxMbbo_init,
  NULL,
  devPmacMbxMbbo_write
};

#ifdef STATUS_RECORD
PMAC_DSET_STATUS devPmacMbxStatus = {
  5,
  NULL,
  devPmacMbx_init,
  devPmacMbxStatus_init,
  NULL,
  devPmacMbxStatus_read
};
epicsExportAddress(dset,devPmacMbxStatus);
#endif

PMAC_DSET_SI devPmacMbxSi = {
  5,
  NULL,
  devPmacMbx_init,
  devPmacMbxSi_init,
  NULL,
  devPmacMbxSi_read
};

PMAC_DSET_SO devPmacMbxSo = {
  5,
  NULL,
  devPmacMbx_init,
  devPmacMbxSo_init,
  NULL,
  devPmacMbxSo_write
};

PMAC_DSET_WF devPmacMbxWf = {
  5,
  NULL,
  devPmacMbx_init,
  devPmacMbxWf_init,
  NULL,
  devPmacMbxWf_write
};

#ifdef LOAD_RECORD
PMAC_DSET_LOAD devPmacMbxLoad = {
  5,
  NULL,
  devPmacMbx_init,
  devPmacMbxLoad_init,
  NULL,
  devPmacMbxLoad_proc
};
epicsExportAddress(dset,devPmacMbxLoad);
#endif
epicsExportAddress(dset,devPmacMbxAi);
epicsExportAddress(dset,devPmacMbxAo);
epicsExportAddress(dset,devPmacMbxBi);
epicsExportAddress(dset,devPmacMbxBo);
epicsExportAddress(dset,devPmacMbxLi);
epicsExportAddress(dset,devPmacMbxLo);
epicsExportAddress(dset,devPmacMbxMbbi);
epicsExportAddress(dset,devPmacMbxMbbo);
epicsExportAddress(dset,devPmacMbxSi);
epicsExportAddress(dset,devPmacMbxSo);
epicsExportAddress(dset,devPmacMbxWf);


/*
 * LOCALS
 */

/*******************************************************************************
 *
 * devPmacMbx_init - EPICS device support init function
 *
 */
LOCAL long devPmacMbx_init
(
	int	after
)
{
/*	char *	MyName = "devPmacMbx_init"; */
	long		status = 0;

	if (after == 1)
	{
		status = drvPmacStartup ();
	}

	return (status);
}


/*******************************************************************************
 *
 * devPmacMbxDpvtInit - EPICS PMAC_MBX_DPVT init
 *
 */
PMAC_MBX_DPVT * devPmacMbxDpvtInit
(
	struct dbCommon *	pRec,
	int			card
)
{
/*	char *	MyName = "devPmacMbxDpvtInit"; */
	PMAC_MBX_DPVT *	pDpvt;

	pDpvt = (PMAC_MBX_DPVT *) malloc (sizeof(PMAC_MBX_DPVT));

	pDpvt->MbxIo.pRec = pRec;
	pDpvt->MbxIo.card = card;
	pDpvt->MbxIo.terminator = 0;
	pDpvt->MbxIo.command[0] = (char) NULL;
	pDpvt->MbxIo.response[0] = (char) NULL;

	callbackSetCallback (devPmacMbxCallback, &pDpvt->MbxIo.callback);
	callbackSetPriority (pRec->prio, &pDpvt->MbxIo.callback);
	callbackSetUser ( (void *) pRec, &pDpvt->MbxIo.callback);

	return (pDpvt);
}

/*******************************************************************************
 *
 * devPmacMbxAi_init - EPICS PMAC device support ai init
 *
 */
LOCAL long devPmacMbxAi_init
(
	struct aiRecord	*pRec
)
{
	char *	MyName = "devPmacMbxAi_init";
	/* long status; */

	switch (pRec->inp.type) {
	case (VME_IO) :

		PMAC_DEBUG
		(	1,
			PMAC_MESSAGE ("%s: card %d parm %s\n", MyName,
				pRec->inp.value.vmeio.card, pRec->inp.value.vmeio.parm,0,0,0);
		)

		pRec->dpvt = (void *) devPmacMbxDpvtInit ( (struct dbCommon *) pRec,
						(int) pRec->inp.value.vmeio.card );
		break;

	default :
		recGblRecordError(S_db_badField,(void *)pRec,
		"devPmacMbxAi_init: Illegal INP field");
		return(S_db_badField);
	}
	return(0);
}

/*******************************************************************************
 *
 * devPmacMbxBi_init - EPICS PMAC device support bi init
 *
 */
LOCAL long devPmacMbxBi_init
(
	struct biRecord	*pRec
)
{
	char *	MyName = "devPmacMbxBi_init";
	/* long status; */

	switch (pRec->inp.type) {
	case (VME_IO) :

		PMAC_DEBUG
		(	1,
			PMAC_MESSAGE ("%s: card %d parm %s\n", MyName,
				pRec->inp.value.vmeio.card, pRec->inp.value.vmeio.parm,0,0,0);
		)

		pRec->dpvt = (void *) devPmacMbxDpvtInit ( (struct dbCommon *) pRec,
						(int) pRec->inp.value.vmeio.card );
		break;

	 default :
		recGblRecordError(S_db_badField,(void *)pRec,
			"devPmacMbxBi_init: Illegal INP field");
		return(S_db_badField);
	}
	return(0);
}

/*******************************************************************************
 *
 * devPmacMbxLi_init - EPICS PMAC device support longin init
 *
 */
LOCAL long devPmacMbxLi_init
(
	struct longinRecord	*pRec
)
{
	char *	MyName = "devPmacMbxLi_init";
	/* long status; */

	switch (pRec->inp.type) {
	case (VME_IO) :

		PMAC_DEBUG
		(	1,
			PMAC_MESSAGE ("%s: card %d parm %s\n", MyName,
				pRec->inp.value.vmeio.card, pRec->inp.value.vmeio.parm,0,0,0);
		)

		pRec->dpvt = (void *) devPmacMbxDpvtInit ( (struct dbCommon *) pRec,
						(int) pRec->inp.value.vmeio.card );
		break;

	default :
		recGblRecordError(S_db_badField,(void *)pRec,
			"devPmacMbxLi_init: Illegal INP field");
		return(S_db_badField);
	}
	return(0);
}

/*******************************************************************************
 *
 * devPmacMbxMbbi_init - EPICS PMAC device support mbbi init
 *
 */
LOCAL long devPmacMbxMbbi_init
(
	struct mbbiRecord	*pRec
)
{
	char *	MyName = "devPmacMbxMbbi_init";
	/* long status; */

	switch (pRec->inp.type) {
	case (VME_IO) :

		PMAC_DEBUG
		(	1,
			PMAC_MESSAGE ("%s: card %d parm %s\n", MyName,
				pRec->inp.value.vmeio.card, pRec->inp.value.vmeio.parm,0,0,0);
		)

		pRec->dpvt = (void *) devPmacMbxDpvtInit ( (struct dbCommon *) pRec,
						(int) pRec->inp.value.vmeio.card );
		break;

	default :
		recGblRecordError(S_db_badField,(void *)pRec,
			"devPmacMbxMbbi_init: Illegal INP field");
		return(S_db_badField);
	}
	 return(0);
}

#ifdef STATUS_RECORD
/*******************************************************************************
 *
 * devPmacMbxStatus_init - EPICS PMAC device support status init
 *
 */
LOCAL long devPmacMbxStatus_init
(
	struct statusRecord	*pRec
)
{
	char *	MyName = "devPmacMbxStatus_init";
	/* long status; */

	switch (pRec->inp.type) {
	case (VME_IO) :

		PMAC_DEBUG
		(	1,
   			PMAC_MESSAGE ("%s: card %d parm %s\n", MyName,
				pRec->inp.value.vmeio.card, pRec->inp.value.vmeio.parm,0,0,0);
		)

		pRec->dpvt = (void *) devPmacMbxDpvtInit ( (struct dbCommon *) pRec,
						(int) pRec->inp.value.vmeio.card );
		break;

	default :
		recGblRecordError(S_db_badField,(void *)pRec,
			"devPmacMbxStatus_init: Illegal INP field");
		return(S_db_badField);
	}
	return(0);
}
#endif	/* STATUS_RECORD */

/*******************************************************************************
 *
 * devPmacMbxAo_init - EPICS PMAC device support ao init
 *
 */
LOCAL long devPmacMbxAo_init
(
	struct aoRecord *pRec
)
{
	char *	MyName = "devPmacMbxAo_init";
	/* long status=0; */

	switch (pRec->out.type) {
	case (VME_IO) :

		PMAC_DEBUG
		(	1,
			PMAC_MESSAGE ("%s: card %d parm %s\n", MyName,
				pRec->out.value.vmeio.card, pRec->out.value.vmeio.parm,0,0,0);
		)

		pRec->dpvt = (void *) devPmacMbxDpvtInit ( (struct dbCommon *) pRec,
						(int) pRec->out.value.vmeio.card );
		break;

	default :
		recGblRecordError(S_db_badField,(void *)pRec,
			"devPmacMbxAo_init: Illegal OUT field");
		return(S_db_badField);
	}

	return (0);

}

/*******************************************************************************
 *
 * devPmacMbxBo_init - EPICS PMAC device support bo init
 *
 */
LOCAL long devPmacMbxBo_init
(
	struct boRecord *pRec
)
{
	char *	MyName = "devPmacMbxBo_init";
	/* long	status = 0; */

	switch (pRec->out.type) {
	case (VME_IO) :

		PMAC_DEBUG
		(	1,
			PMAC_MESSAGE ("%s: card %d parm %s\n", MyName,
				pRec->out.value.vmeio.card, pRec->out.value.vmeio.parm,0,0,0);
		)

		pRec->dpvt = (void *) devPmacMbxDpvtInit ( (struct dbCommon *) pRec,
						(int) pRec->out.value.vmeio.card );
		break;

	default :
		recGblRecordError(S_db_badField,(void *)pRec,
			"devPmacMbxBo_init: Illegal OUT field");
		return(S_db_badField);
	}

    return (0);

}

/*******************************************************************************
 *
 * devPmacMbxLo_init - EPICS PMAC device support longout init
 *
 */
LOCAL long devPmacMbxLo_init
(
	struct longoutRecord *pRec
)
{
	char *	MyName = "devPmacMbxLo_init";
	/* long	status = 0; */

	switch (pRec->out.type) {
	case (VME_IO) :

		PMAC_DEBUG
		(	1,
			PMAC_MESSAGE ("%s: card %d parm %s\n", MyName,
				pRec->out.value.vmeio.card, pRec->out.value.vmeio.parm,0,0,0);
		)

		pRec->dpvt = (void *) devPmacMbxDpvtInit ( (struct dbCommon *) pRec,
						(int) pRec->out.value.vmeio.card );
		break;

	default :
		recGblRecordError(S_db_badField,(void *)pRec,
			"devPmacMbxLo_init: Illegal OUT field");
		return(S_db_badField);
	}

	return (0);

}

/*******************************************************************************
 *
 * devPmacMbxMbbo_init - EPICS PMAC device support mbbo init
 *
 */
LOCAL long devPmacMbxMbbo_init
(
	struct mbboRecord *pRec
)
{
	char *	MyName = "devPmacMbxMbbo_init";
	long	status = 0;

	switch (pRec->out.type) {
	case (VME_IO) :

		PMAC_DEBUG
		(	1,
			PMAC_MESSAGE ("%s: card %d parm %s\n", MyName,
				pRec->out.value.vmeio.card, pRec->out.value.vmeio.parm,0,0,0);
		)

		pRec->dpvt = (void *) devPmacMbxDpvtInit ( (struct dbCommon *) pRec,
						(int) pRec->out.value.vmeio.card );
		break;

	default :
		recGblRecordError(S_db_badField,(void *)pRec,
			"devPmacMbxMbbo_init: Illegal INP field");
		return(S_db_badField);
	}

	return (status);

}

/*******************************************************************************
 *
 * devPmacMbxSi_init - EPICS PMAC device support stringin init
 *
 */
LOCAL long devPmacMbxSi_init
(
	struct stringinRecord *pRec
)
{
	char *	MyName = "devPmacMbxSi_init";
	long status = 0;

	switch (pRec->inp.type) {
	case (VME_IO) :

		PMAC_DEBUG
		(	1,
			PMAC_MESSAGE ("%s: name %s card %d parm %s\n", MyName, pRec->name,
				pRec->inp.value.vmeio.card, pRec->inp.value.vmeio.parm,0,0);
		)

		pRec->dpvt = (void *) devPmacMbxDpvtInit ( (struct dbCommon *) pRec,
						(int) pRec->inp.value.vmeio.card );
		break;

	default :
		recGblRecordError(S_db_badField,(void *)pRec,
			"devPmacMbxSi_init: Illegal INP field");
		return(S_db_badField);
	}

	return (status);

}

/*******************************************************************************
 *
 * devPmacMbxSo_init - EPICS PMAC device support stringout init
 *
 */
LOCAL long devPmacMbxSo_init
(
	struct stringoutRecord *pRec
)
{
	char *	MyName = "devPmacMbxSo_init";
	long status = 0;

	switch (pRec->out.type) {
	case (VME_IO) :

		PMAC_DEBUG
		(	1,
			PMAC_MESSAGE ("%s: name %s card %d parm %s\n", MyName, pRec->name,
				pRec->out.value.vmeio.card, pRec->out.value.vmeio.parm,0,0);
		)

		pRec->dpvt = (void *) devPmacMbxDpvtInit ( (struct dbCommon *) pRec, (int) pRec->out.value.vmeio.card );
		break;

	default :
		recGblRecordError(S_db_badField,(void *)pRec,
			"devPmacMbxSo_init: Illegal OUT field");
		return(S_db_badField);
	}

	return (status);

}

/*******************************************************************************
 *
 * devPmacMbxWf_init - EPICS PMAC device support waveform init
 *
 */
LOCAL long devPmacMbxWf_init
(
	struct waveformRecord *pRec,
	int pass
)
{
	char *	MyName = "devPmacMbxWf_init";
	long status = 0;

	if (pass==0){
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

		PMAC_DEBUG
		(	1,
			PMAC_MESSAGE ("%s: name %s card %d parm %s\n", MyName, pRec->name,
				pRec->inp.value.vmeio.card, pRec->inp.value.vmeio.parm,0,0);
		)

		pRec->dpvt = (void *) devPmacMbxDpvtInit ( (struct dbCommon *) pRec, (int) pRec->inp.value.vmeio.card );
		/* signal 2 is for PMAC error messages; tell PMAC driver where to report error message */
		if (pRec->inp.value.vmeio.signal == 2) {drvPmacStrErr (pRec);}
		break;

	default :
		recGblRecordError(S_db_badField,(void *)pRec, "devPmacMbxWf_init: Illegal OUT field");
		return(S_db_badField);
	}
	
 
	return (status);

}

#ifdef LOAD_RECORD
/*******************************************************************************
 *
 * devPmacMbxLoad_init - EPICS PMAC device support load init
 *
 */
LOCAL long devPmacMbxLoad_init
(
	struct loadRecord *pRec
)
{
	char *	MyName = "devPmacMbxLoad_init";
	long status = 0;

	switch (pRec->out.type) {
	case (VME_IO) :

		PMAC_DEBUG
		(	1,
			PMAC_MESSAGE ("%s: name %s card %d parm %s\n", MyName, pRec->name,
				pRec->out.value.vmeio.card, pRec->out.value.vmeio.parm,0,0);
		)

		pRec->dpvt = (void *) devPmacMbxDpvtInit ( (struct dbCommon *) pRec,
						(int) pRec->out.value.vmeio.card );
		break;

	default :
		recGblRecordError(S_db_badField,(void *)pRec,
			"devPmacMbxLoad_init: Illegal OUT field");
		return(S_db_badField);
	}

	return (status);

}
#endif	/* LOAD_RECORD */

/*******************************************************************************
 *
 * devPmacMbxAi_read - EPICS PMAC device support ai read
 *
 */
static long devPmacMbxAi_read
(
	struct aiRecord	*pRec
)
{
 	char *	MyName = "devPmacMbxAi_read";
	/* long	status; */

	double	valD;

	PMAC_MBX_DPVT *	pDpvt = (PMAC_MBX_DPVT *) pRec->dpvt;
	PMAC_MBX_IO *	pMbxIo = &pDpvt->MbxIo;

	if (pRec->pact)
	{

		PMAC_TRACE
		(	2,
			PMAC_MESSAGE ("%s: %s response [%s]\n", MyName, pRec->name, pMbxIo->response,0,0,0);
			PMAC_MESSAGE ("%s: TPRO=%d\n", MyName, pRec->tpro,0,0,0,0);
		)

		if (pRec->inp.value.vmeio.signal == 0xfff) {
		   /* this is for the ACC59E ADC conversion */
		   int	valI;
		   sscanf (pMbxIo->response, "%d", &valI);
		   valD =  (valI & 0x800) ? (double)(0xFFF-valI) * pRec->egul / 0x7FF : (double)valI * pRec->eguf / 0x7ff;
		}else{
		   sscanf (pMbxIo->response, "%lf", &valD);

		   /* Adjust Slope And Offset */
		   if (pRec->aslo != 0.0)
		   {
			valD *= pRec->aslo;
		   }
		   if (pRec->aoff != 0.0)
		   {
			valD += pRec->aoff;
		   }
		}

		/* pRec->linr Conversion Ignored */

		/* Apply Smoothing Algorithm */
		if (pRec->smoo != 0.0)
		{
	    		if (pRec->init == TRUE) pRec->val = valD;	/* initial condition */
	    		pRec->val = valD * (1.00 - pRec->smoo) + (pRec->val * pRec->smoo);
		}
		else
		{
	    		pRec->val = valD;
		}

		pRec->udf = FALSE;
		return (2);
	}

	else
	{

		sprintf (pMbxIo->command, "%s", pRec->inp.value.vmeio.parm);

		PMAC_TRACE
		(	2,
			PMAC_MESSAGE ("%s: %s command [%s]\n", MyName, pRec->name, pMbxIo->command,0,0,0);
			PMAC_MESSAGE ("%s: TPRO=%d\n", MyName, pRec->tpro,0,0,0,0);
		)

		pRec->pact = TRUE;
		drvPmacMbxScan (pMbxIo);

		return (0);
	}

}

/*******************************************************************************
 *
 * devPmacMbxBi_read - EPICS PMAC device support bi read
 *
 */
LOCAL long devPmacMbxBi_read
(
	struct biRecord	*pRec
)
{
 	char *	MyName = "devPmacMbxBi_read";
	/* long status; */

	long valL;

	PMAC_MBX_DPVT *	pDpvt = (PMAC_MBX_DPVT *) pRec->dpvt;
	PMAC_MBX_IO *	pMbxIo = &pDpvt->MbxIo;

	if (pRec->pact)
	{

		PMAC_TRACE
		(	2,
			PMAC_MESSAGE ("%s: %s response [%s]\n", MyName, pRec->name, pMbxIo->response,0,0,0);
		)

		sscanf (pMbxIo->response, "%ld", &valL);

		pRec->rval = (unsigned long) valL;
		pRec->udf = FALSE;

		return (0);
	}

	else
	{

		sprintf (pMbxIo->command, "%s", pRec->inp.value.vmeio.parm);

		PMAC_TRACE
		(	2,
			PMAC_MESSAGE ("%s: %s command [%s]\n", MyName, pRec->name, pMbxIo->command,0,0,0);
		)

		pRec->pact = TRUE;
		drvPmacMbxScan (pMbxIo);

		return (0);
	}

}

/*******************************************************************************
 *
 * devPmacMbxLi_read - EPICS PMAC device support longin read
 *
 */
LOCAL long devPmacMbxLi_read
(
	struct longinRecord	*pRec
)
{
 	char *	MyName = "devPmacMbxLi_read";
    	/* long status; */

	long valL;

	PMAC_MBX_DPVT *	pDpvt = (PMAC_MBX_DPVT *) pRec->dpvt;
	PMAC_MBX_IO *	pMbxIo = &pDpvt->MbxIo;

	if (pRec->pact)
	{

		PMAC_TRACE
		(	2,
			PMAC_MESSAGE ("%s: %s response [%s]\n", MyName, pRec->name, pMbxIo->response,0,0,0);
		)

		sscanf (pMbxIo->response, "%ld", &valL);

		pRec->val = valL;
		pRec->udf = FALSE;

		return (0);
	}

	else
	{

		sprintf (pMbxIo->command, "%s", pRec->inp.value.vmeio.parm);

		PMAC_TRACE
		(	2,
			PMAC_MESSAGE ("%s: %s command [%s]\n", MyName, pRec->name, pMbxIo->command,0,0,0);
		)

		pRec->pact = TRUE;
		drvPmacMbxScan (pMbxIo);

		return (0);
	}

}

/*******************************************************************************
 *
 * devPmacMbxMbbi_read - EPICS PMAC device support mbbi read
 *
 */
LOCAL long devPmacMbxMbbi_read
(
	struct mbbiRecord	*pRec
)
{
 	char *	MyName = "devPmacMbxMbbi_read";
    	/* long status; */

	long valL;

	PMAC_MBX_DPVT *	pDpvt = (PMAC_MBX_DPVT *) pRec->dpvt;
	PMAC_MBX_IO *	pMbxIo = &pDpvt->MbxIo;

	if (pRec->pact)
	{

		PMAC_TRACE
		(	2,
			PMAC_MESSAGE ("%s: %s response [%s]\n", MyName, pRec->name, pMbxIo->response,0,0,0);
		)

		sscanf (pMbxIo->response, "%ld", &valL);

		pRec->rval = (unsigned long) valL;
		pRec->udf = FALSE;

		return (0);
	}

	else
	{

		sprintf (pMbxIo->command, "%s", pRec->inp.value.vmeio.parm);

		PMAC_TRACE
		(	2,
			PMAC_MESSAGE ("%s: %s command [%s]\n", MyName, pRec->name, pMbxIo->command,0,0,0);
		)

		pRec->pact = TRUE;
		drvPmacMbxScan (pMbxIo);

		return (0);
	}

}

#ifdef STATUS_RECORD
/*******************************************************************************
 *
 * devPmacMbxStatus_read - EPICS PMAC device support status read
 *
 */
LOCAL long devPmacMbxStatus_read
(
	struct statusRecord	*pRec
)
{
 	char *	MyName = "devPmacMbxStatus_read";
    	/* long status; */

	long valL;

	PMAC_MBX_DPVT *	pDpvt = (PMAC_MBX_DPVT *) pRec->dpvt;
	PMAC_MBX_IO *	pMbxIo = &pDpvt->MbxIo;

	if (pRec->pact)
	{

		PMAC_TRACE
		(	2,
			PMAC_MESSAGE ("%s: %s response [%s]\n", MyName, pRec->name, pMbxIo->response,0,0,0);
		)

		sscanf (pMbxIo->response, "%ld", &valL);

		pRec->val = valL;
		pRec->udf = FALSE;

		return (0);
	}

	else
	{

		sprintf (pMbxIo->command, "%s", pRec->inp.value.vmeio.parm);

		PMAC_TRACE
		(	2,
			PMAC_MESSAGE ("%s: %s command [%s]\n", MyName, pRec->name, pMbxIo->command,0,0,0);
		)

		pRec->pact = TRUE;
		drvPmacMbxScan (pMbxIo);

		return (0);
	}

}
#endif	/* STATUS_RECORD */

/*******************************************************************************
 *
 * devPmacMbxAo_write - EPICS PMAC device support ao write
 *
 */
LOCAL long devPmacMbxAo_write
(
	struct aoRecord	*pRec
)
{
 	char *	MyName = "devPmacMbxAo_write";
	/* long status; */

	double	valD;

	PMAC_MBX_DPVT *	pDpvt = (PMAC_MBX_DPVT *) pRec->dpvt;
	PMAC_MBX_IO *	pMbxIo = &pDpvt->MbxIo;

	if (pRec->pact)
	{

		PMAC_TRACE
		(	2,
			PMAC_MESSAGE ("%s: %s response [%s]\n", MyName, pRec->name, pMbxIo->response,0,0,0);
		)

		return (0);
	}

	else
	{

		/* Output Value */
		valD = (double) pRec->oval;

		if(pRec->out.value.vmeio.signal == 0xfff) {
			/* this is for the ACC59E DAC conversion */
		  	valD = (double)0xfff * (pRec->oval - pRec->egul) / (pRec->eguf - pRec->egul) + 0.5;
		
			sprintf (pMbxIo->command, "%s%d", pRec->out.value.vmeio.parm, (int)valD);
		} else {

			/* Adjust Slope And Offset */
			if (pRec->aoff != 0.0)
			{
				valD -= (double) pRec->aoff;
			}
			if (pRec->aslo != 0.0)
			{
				valD /= (double) pRec->aslo;
			}

			/* pRec->linr Conversion Ignored */

			sprintf (pMbxIo->command, "%s%f", pRec->out.value.vmeio.parm, valD);
		}

		PMAC_TRACE
		(	2,
			PMAC_MESSAGE ("%s: %s command [%s]\n", MyName, pRec->name, pMbxIo->command,0,0,0);
		)

		pRec->pact = TRUE;
		drvPmacMbxScan (pMbxIo);

		return (0);
	}

}

/*******************************************************************************
 *
 * devPmacMbxBo_write - EPICS PMAC device support bo write
 *
 */
LOCAL long devPmacMbxBo_write
(
	struct boRecord	*pRec
)
{
 	char *	MyName = "devPmacMbxBo_write";
	/* long status; */

	long valL;

	PMAC_MBX_DPVT *	pDpvt = (PMAC_MBX_DPVT *) pRec->dpvt;
	PMAC_MBX_IO *	pMbxIo = &pDpvt->MbxIo;

	if (pRec->pact)
	{

		PMAC_TRACE
		(	2,
			PMAC_MESSAGE ("%s: %s response [%s]\n", MyName, pRec->name, pMbxIo->response,0,0,0);
		)

		return (0);
	}

	else
	{

		valL = (long) pRec->val;

		sprintf (pMbxIo->command,"%s%ld", pRec->out.value.vmeio.parm, valL);

		PMAC_TRACE
		(	2,
			PMAC_MESSAGE ("%s: %s command [%s]\n", MyName, pRec->name, pMbxIo->command,0,0,0);
		)

		pRec->pact = TRUE;
		drvPmacMbxScan (pMbxIo);

		return (0);
	}
}

/*******************************************************************************
 *
 * devPmacMbxLo_write - EPICS PMAC device support longout write
 *
 */
LOCAL long devPmacMbxLo_write
(
	struct longoutRecord	*pRec
)
{
 	char *	MyName = "devPmacMbxLo_write";
	/* long status; */

	long valL;

	PMAC_MBX_DPVT *	pDpvt = (PMAC_MBX_DPVT *) pRec->dpvt;
	PMAC_MBX_IO *	pMbxIo = &pDpvt->MbxIo;

	if (pRec->pact)
	{

		PMAC_TRACE
		(	2,
			PMAC_MESSAGE ("%s: %s response [%s]\n", MyName, pRec->name, pMbxIo->response,0,0,0);
		)

		return (0);
	}

	else
	{

		valL = (long) pRec->val;

		sprintf (pMbxIo->command,"%s%ld", pRec->out.value.vmeio.parm, valL);

		PMAC_TRACE
		(	2,
			PMAC_MESSAGE ("%s: %s command [%s]\n", MyName, pRec->name, pMbxIo->command,0,0,0);
		)

		pRec->pact = TRUE;
		drvPmacMbxScan (pMbxIo);

		return (0);
	}

}

/*******************************************************************************
 *
 * devPmacMbxMbbo_write - EPICS PMAC device support mbbo write
 *
 */
LOCAL long devPmacMbxMbbo_write
(
	struct mbboRecord	*pRec
)
{
 	char *	MyName = "devPmacMbxMbbo_write";
	/* long status; */

	long valL;

	PMAC_MBX_DPVT *	pDpvt = (PMAC_MBX_DPVT *) pRec->dpvt;
	PMAC_MBX_IO *	pMbxIo = &pDpvt->MbxIo;

	if (pRec->pact)
	{

		PMAC_TRACE
		(	2,
			PMAC_MESSAGE ("%s: %s response [%s]\n", MyName, pRec->name, pMbxIo->response,0,0,0);
		)

		return (0);
	}

	else
	{

		valL = (long) pRec->rval;

		sprintf (pMbxIo->command,"%s%ld", pRec->out.value.vmeio.parm, valL);

		PMAC_TRACE
		(	2,
			PMAC_MESSAGE ("%s: %s command [%s]\n", MyName, pRec->name, pMbxIo->command,0,0,0);
		)

		pRec->pact = TRUE;
		drvPmacMbxScan (pMbxIo);

		return (0);
	}

}

/*******************************************************************************
 *
 * devPmacMbxSi_read - EPICS PMAC device support stringin read
 *
 */
LOCAL long devPmacMbxSi_read
(
	struct stringinRecord	*pRec
)
{
 	char *	MyName = "devPmacMbxSi_read";
	/* long status; */

	PMAC_MBX_DPVT *	pDpvt = (PMAC_MBX_DPVT *) pRec->dpvt;
	PMAC_MBX_IO *	pMbxIo = &pDpvt->MbxIo;

	if (pRec->pact)
	{

		PMAC_TRACE
		(	2,
			PMAC_MESSAGE ("%s: %s response [%s]\n", MyName, pRec->name, pMbxIo->response,0,0,0);
		)

		pRec->val[39] = '\0';
		strncpy (pRec->val, pMbxIo->response, 39);

		pRec->udf = FALSE;
		return (0);
	}

	else
	{

		switch (pRec->inp.value.vmeio.signal)
		{
		case (1):
			sprintf (pMbxIo->command,"%s%s", pRec->inp.value.vmeio.parm, pRec->val);
			break;
		case (0):
		default:
			sprintf (pMbxIo->command,"%s", pRec->inp.value.vmeio.parm);
			break;
		}

		PMAC_TRACE
		(	2,
			PMAC_MESSAGE ("%s: %s command [%s]\n", MyName, pRec->name, pMbxIo->command,0,0,0);
		)

		pRec->pact = TRUE;
		drvPmacMbxScan (pMbxIo);

		return (0);
	}

}

/*******************************************************************************
 *
 * devPmacMbxSo_write - EPICS PMAC device support stringout write
 *
 */
LOCAL long devPmacMbxSo_write
(
	struct stringoutRecord	*pRec
)
{
 	char *	MyName = "devPmacMbxSo_write";
	/* long status; */

	PMAC_MBX_DPVT *	pDpvt = (PMAC_MBX_DPVT *) pRec->dpvt;
	PMAC_MBX_IO *	pMbxIo = &pDpvt->MbxIo;

	if (pRec->pact)
	{

		PMAC_TRACE
		(	2,
			PMAC_MESSAGE ("%s: %s response [%s]\n", MyName, pRec->name, pMbxIo->response,0,0,0);
		)
		switch (pRec->out.value.vmeio.signal)
		{
		case (1):
			pRec->val[39] = '\0';
			strncpy (pRec->val, pMbxIo->response, 39);
			break;
		case (0):
		default:
			break;
		}

	} else {


	  /*MRP Commeted this change out because it broke I06 hexapod IOC databases. 9 March 2011*/
	  /*
		switch (pRec->out.value.vmeio.signal)
		{
		case (1):
			pRec->pact = TRUE;
			sprintf (pMbxIo->command,"%s%s", pRec->out.value.vmeio.parm, pRec->val);

			PMAC_TRACE
			(	2,
				PMAC_MESSAGE ("%s: %s command [%s]\n", MyName, pRec->name, pMbxIo->command,0,0,0);
			)

			drvPmacMbxScan (pMbxIo);
			break;
		default:
			PMAC_MESSAGE ("%s: %s s[%d] not supported\n", MyName, pRec->name, pRec->out.value.vmeio.signal,0,0,0);
			break;
		}

	  */

	  /*This is the old code from tpmac 3-5dls9.*/
	  sprintf (pMbxIo->command,"%s%s", pRec->out.value.vmeio.parm, pRec->val);
	  
	  PMAC_TRACE
	  (       2,
		    PMAC_MESSAGE ("%s: %s command [%s]\n", MyName, pRec->name, pMbxIo->command,0,0,0);
	  )
	    
	    drvPmacMbxScan (pMbxIo);
	  
	  pRec->pact = TRUE;
	  

	}

		return (0);
}

/*******************************************************************************
 *
 * devPmacMbxWf_write - EPICS PMAC device support stringout write
 *
 */
LOCAL long devPmacMbxWf_write ( struct waveformRecord *pRec)
{
	return (0); /* Nothing to do, data are written in the drvPmacMbxTask (), drvPmac.c */
}

#ifdef LOAD_RECORD
/*******************************************************************************
 *
 * devPmacMbxLoad_proc - EPICS PMAC device support stringout write
 *
 */
LOCAL long devPmacMbxLoad_proc
(
	struct loadRecord	*pRec
)
{
 	char *	MyName = "devPmacMbxLoad_proc";
	/* long status;	*/

	PMAC_MBX_DPVT *	pDpvt = (PMAC_MBX_DPVT *) pRec->dpvt;
	PMAC_MBX_IO *	pMbxIo = &pDpvt->MbxIo;

	if (pRec->pact)
	{

		PMAC_TRACE
		(	2,
			PMAC_MESSAGE ("%s: %s message [%s]\n", MyName, pRec->name, pMbxIo->errmsg,0,0,0);
		)

		pRec->val = pMbxIo->terminator;
		sprintf (pRec->msg, "%s", pMbxIo->errmsg);

		return (0);
	}

	else
	{

		sprintf (pMbxIo->command, "%s", pRec->dnv);
		sprintf (pMbxIo->response, "%s", pRec->upv);

		PMAC_TRACE
		(	2,
			PMAC_MESSAGE ("%s: %s download [%s]\n", MyName, pRec->name, pMbxIo->command,0,0,0);
			PMAC_MESSAGE ("%s: %s upload [%s]\n", MyName, pRec->name, pMbxIo->response,0,0,0);
		)

		pRec->pact = TRUE;
		drvPmacFldScan (pMbxIo);


 		return (0);
	}

}
#endif	/* LOAD_RECORD */

/*******************************************************************************
 *
 * devPmacMbxCallback - EPICS device support Callback
 *
 */
LOCAL void devPmacMbxCallback (CALLBACK *pCallback) {
  char            *MyName = "devPmacMbxCallback";
  /* long         status = 0; */

  struct dbCommon *pRec;
  struct rset     *pRset;

  callbackGetUser (pRec, pCallback);

  pRset = pRec->rset;

  PMAC_TRACE (2, PMAC_MESSAGE ("%s: CALLBACK [%s].\n", MyName, pRec->name,0,0,0,0);)
/*OAM*/
  logMsg("%s: pRec = 0x%x, pRset = 0x%x \n", (int)MyName, (int)pRec,(int)pRset,0,0,0);
  dbScanLock (pRec);
  (*(pRset->process))(pRec);
  dbScanUnlock (pRec);
  return;
}

