/* statusRecord.c - Record Support Routines for Status records */
/*
 *      Author: 	Thomas Coleman
 *      Date:   	96/02/27
 *
 *      Experimental Physics and Industrial Control System (EPICS)
 *
 *      Copyright 1991, the Regents of the University of California,
 *      and the University of Chicago Board of Governors.
 *
 *      This software was produced under  U.S. Government contracts:
 *      (W-7405-ENG-36) at the Los Alamos National Laboratory,
 *      and (W-31-109-ENG-38) at Argonne National Laboratory.
 *
 *      Initial development by:
 *              The Controls and Automation Group (AT-8)
 *              Ground Test Accelerator
 *              Accelerator Technology Division
 *              Los Alamos National Laboratory
 *
 *      Co-developed with
 *              The Controls and Computing Group
 *              Accelerator Systems Division
 *              Advanced Photon Source
 *              Argonne National Laboratory
 *
 * Modification Log:
 * -----------------
 * .01  11-11-91        jba     Moved set and reset of alarm stat and sevr to macros
 * .02  02-05-92	jba	Changed function arguments from paddr to precord
 * .03  02-28-92        jba     Changed get_precision,get_graphic_double,get_control_double
 * .04  02-28-92	jba	ANSI C changes
 * .05  04-10-92        jba     pact now used to test for asyn processing, not status
 * .06  04-18-92        jba     removed process from dev init_record parms
 * .07  06-02-92        jba     changed graphic/control limits for hihi,high,low,lolo
 * .08  07-15-92        jba     changed VALID_ALARM to INVALID alarm
 * .09  07-16-92        jba     added invalid alarm fwd link test and chngd fwd lnk to macro
 * .10  07-21-92        jba     changed alarm limits for non val related fields
 * .11  08-06-92        jba     New algorithm for calculating analog alarms
 * .12  08-13-92        jba     Added simulation processing
 * .13  05-30-92	vong	Convert recLongin to recStatus
 * .14	06-18-95	tac	fixed bug in bit16 & bit24 - bit31
 * .15  02-26-96	tac	Process links on change of stat and sevr;
 *				Decode bits before processing links; fixed bug in bit 24
 *
 *  History
 *  -------
 *  Version 1.0  21/10/97  ajf  Changes for 3.13.
 *  Version 1.1  31/10/97  ajf  Added get_precision function for VERS field.
 */

#define VERSION 1.1

/* VxWorks includes */
#ifdef vxWorks
#include        <vxWorks.h>
#include        <lstLib.h>
#endif
#include        <stdlib.h>
#include        <stdio.h>
#include        <string.h>


#include        <alarm.h>
#include        <dbDefs.h>
#include        <dbEvent.h>
#include        <dbAccess.h>
#include        <dbFldTypes.h>
#include        <errMdef.h>
#include        <recSup.h>
#include        <devSup.h>

#define GEN_SIZE_OFFSET
#include	<statusRecord.h>
#undef GEN_SIZE_OFFSET
#include "epicsTime.h"
#include "recGbl.h"
#include "epicsExport.h"

/* Create RSET - Record Support Entry Table*/
static long init_record();
static long process();
static long get_units();
static long get_graphic_double();
static long get_control_double();
static long get_alarm_double();
static long get_precision();
#define report          NULL
#define initialize      NULL
#define special         NULL
#define cvt_dbaddr      NULL
#define get_value       NULL
#define get_array_info  NULL
#define put_array_info  NULL
static long get_enum_str();
#define get_enum_strs   NULL
#define put_enum_str    NULL
/* Changed for EPICS-3.14.11 where YES/NO are no longer defined in EPICS includes */
#if !defined(YES) || !defined(NO)
#define YES 1
#define NO 0
#endif

rset statusRSET={
	RSETNUMBER,
	report,
	initialize,
	init_record,
	process,
	special,
	get_value,
	cvt_dbaddr,
	get_array_info,
	put_array_info,
	get_units,
	get_precision,
	get_enum_str,
	get_enum_strs,
	put_enum_str,
	get_graphic_double,
	get_control_double,
	get_alarm_double };
epicsExportAddress(rset,statusRSET);

struct statusdset { /* status input dset */
	long		number;
	DEVSUPFUN	dev_report;
	DEVSUPFUN	init;
	DEVSUPFUN	init_record; /*returns: (-1,0)=>(failure,success)*/
	DEVSUPFUN	get_ioint_info;
	DEVSUPFUN	read_status; /*returns: (-1,0)=>(failure,success)*/
};
static void alarm();
static void monitor();
static long readValue();
static void decodeBits();
static void activateAllLink();
static void activateNewLink();

static long get_enum_str(paddr,pstring)
    struct dbAddr *paddr;
    char	  *pstring;
{
    int                 index;
    unsigned short      *pfield = (unsigned short *)paddr->pfield;
    index = dbGetFieldIndex(paddr);
    if(index >= statusRecordBI00 && index <= statusRecordBI31) {
      if(*pfield==0) {
	strcpy(pstring,"OFF");
      } else {
	strcpy(pstring,"ON");
      }
    } else {
	strcpy(pstring,"Illegal_Value");
    }
    return(0);
}

static long init_record(pstatus,pass)
    struct statusRecord	*pstatus;
    int pass;
{
    struct statusdset *pdset;
    long status;

    if (pass==0)
    {
      pstatus->vers = VERSION;
      return(0);
    }

    if( pstatus->siml.type == CONSTANT )
      recGblInitConstantLink(&pstatus->siml, DBF_ENUM, &pstatus->simm);

    if( pstatus->siol.type == CONSTANT )
      recGblInitConstantLink(&pstatus->siol, DBF_LONG, &pstatus->sval);

    if(!(pdset = (struct statusdset *)(pstatus->dset))) {
	recGblRecordError(S_dev_noDSET,(void *)pstatus,"statusRecord: init_record");
	return(S_dev_noDSET);
    }
    /* must have read_status function defined */
    if( (pdset->number < 5) || (pdset->read_status == NULL) ) {
	recGblRecordError(S_dev_missingSup,(void *)pstatus,"statusRecord: init_record");
	return(S_dev_missingSup);
    }
    if( pdset->init_record ) {
	if((status=(*pdset->init_record)(pstatus))) return(status);
    }
    return(0);
}

static long process(pstatus)
	struct statusRecord     *pstatus;
{
	struct statusdset	*pdset = (struct statusdset *)(pstatus->dset);
	long		 status;
	unsigned char    pact=pstatus->pact;

	if( (pdset==NULL) || (pdset->read_status==NULL) ) {
		pstatus->pact=TRUE;
		recGblRecordError(S_dev_missingSup,(void *)pstatus,"statusRecord: read_status");
		return(S_dev_missingSup);
	}

	status=readValue(pstatus); /* read the new value */
	/* check if device support set pact */
	if ( !pact && pstatus->pact ) return(0);
	pstatus->pact = TRUE;

	epicsTimeGetCurrent (&pstatus->time);

	/* check for alarms */
	alarm(pstatus);
	/* check event list */
	monitor(pstatus);
	/* decode the bits */
        decodeBits(pstatus);
	/* activate links for changed bits */
        activateNewLink(pstatus);
	/* process the forward scan link record */
	recGblFwdLink(pstatus);

	pstatus->pact=FALSE;
	return(status);
}


static long get_units(paddr,units)
    struct dbAddr *paddr;
    char	  *units;
{
    struct statusRecord	*pstatus=(struct statusRecord *)paddr->precord;

    strncpy(units,pstatus->egu,sizeof(pstatus->egu));
    return(0);
}


static long get_graphic_double(paddr,pgd)
    struct dbAddr *paddr;
    struct dbr_grDouble	*pgd;
{
    struct statusRecord	*pstatus=(struct statusRecord *)paddr->precord;

    if(paddr->pfield==(void *)&pstatus->val
    || paddr->pfield==(void *)&pstatus->hihi
    || paddr->pfield==(void *)&pstatus->high
    || paddr->pfield==(void *)&pstatus->low
    || paddr->pfield==(void *)&pstatus->lolo){
        pgd->upper_disp_limit = pstatus->hopr;
        pgd->lower_disp_limit = pstatus->lopr;
    } else recGblGetGraphicDouble(paddr,pgd);
    return(0);
}

static long get_control_double(paddr,pcd)
    struct dbAddr *paddr;
    struct dbr_ctrlDouble *pcd;
{
    struct statusRecord	*pstatus=(struct statusRecord *)paddr->precord;

    if(paddr->pfield==(void *)&pstatus->val
    || paddr->pfield==(void *)&pstatus->hihi
    || paddr->pfield==(void *)&pstatus->high
    || paddr->pfield==(void *)&pstatus->low
    || paddr->pfield==(void *)&pstatus->lolo){
        pcd->upper_ctrl_limit = pstatus->hopr;
        pcd->lower_ctrl_limit = pstatus->lopr;
    } else recGblGetControlDouble(paddr,pcd);
    return(0);
}

static long get_alarm_double(paddr,pad)
    struct dbAddr *paddr;
    struct dbr_alDouble	*pad;
{
    struct statusRecord	*pstatus=(struct statusRecord *)paddr->precord;

    if(paddr->pfield==(void *)&pstatus->val){
         pad->upper_alarm_limit = pstatus->hihi;
         pad->upper_warning_limit = pstatus->high;
         pad->lower_warning_limit = pstatus->low;
         pad->lower_alarm_limit = pstatus->lolo;
    } else recGblGetAlarmDouble(paddr,pad);
    return(0);
}

static void alarm(pstatus)
    struct statusRecord	*pstatus;
{
	double		val;
	float		hyst, lalm, hihi, high, low, lolo;
	unsigned short	hhsv, llsv, hsv, lsv;

	if(pstatus->udf == TRUE ){
 		recGblSetSevr(pstatus,UDF_ALARM,INVALID_ALARM);
		return;
	}
	hihi = pstatus->hihi; lolo = pstatus->lolo; high = pstatus->high; low = pstatus->low;
	hhsv = pstatus->hhsv; llsv = pstatus->llsv; hsv = pstatus->hsv; lsv = pstatus->lsv;
	val = pstatus->val; hyst = pstatus->hyst; lalm = pstatus->lalm;

	/* alarm condition hihi */
	if (hhsv && (val >= hihi || ((lalm==hihi) && (val >= hihi-hyst)))){
	        if (recGblSetSevr(pstatus,HIHI_ALARM,pstatus->hhsv)) pstatus->lalm = hihi;
		return;
	}

	/* alarm condition lolo */
	if (llsv && (val <= lolo || ((lalm==lolo) && (val <= lolo+hyst)))){
	        if (recGblSetSevr(pstatus,LOLO_ALARM,pstatus->llsv)) pstatus->lalm = lolo;
		return;
	}

	/* alarm condition high */
	if (hsv && (val >= high || ((lalm==high) && (val >= high-hyst)))){
	        if (recGblSetSevr(pstatus,HIGH_ALARM,pstatus->hsv)) pstatus->lalm = high;
		return;
	}

	/* alarm condition low */
	if (lsv && (val <= low || ((lalm==low) && (val <= low+hyst)))){
	        if (recGblSetSevr(pstatus,LOW_ALARM,pstatus->lsv)) pstatus->lalm = low;
		return;
	}

	/* we get here only if val is out of alarm by at least hyst */
	pstatus->lalm = val;
	return;
}

static void monitor(pstatus)
    struct statusRecord	*pstatus;
{
	unsigned short	monitor_mask;
	long		delta;
	int		linkActivation = FALSE;

	/* get previous stat and sevr  and new stat and sevr*/
        monitor_mask = recGblResetAlarms(pstatus);
        /* notice any change of stat and sevr */
        if (monitor_mask != 0) linkActivation = TRUE;

	/* check for value change */
	delta = pstatus->mlst - pstatus->val;
	if(delta<0) delta = -delta;
	if (delta > pstatus->mdel) {
		/* post events for value change */
		monitor_mask |= DBE_VALUE;
		/* update last value monitored */
		pstatus->mlst = pstatus->val;
	}

	/* check for archive change */
	delta = pstatus->alst - pstatus->val;
	if(delta<0) delta = -delta;
	if (delta > pstatus->adel) {
		/* post events on value field for archive change */
		monitor_mask |= DBE_LOG;
		/* update last archive value monitored */
		pstatus->alst = pstatus->val;
	}

	/* send out monitors connected to the value field */
	if (monitor_mask){
		db_post_events(pstatus,&pstatus->val,monitor_mask);
	}
	
	/* decode and activate all links if stat or sevr changed */
        if (linkActivation) {
        	decodeBits(pstatus);
       		activateAllLink(pstatus);
       	}
	
	return;
}

static long readValue(pstatus)
	struct statusRecord	*pstatus;
{
	long		status;
        struct statusdset 	*pdset = (struct statusdset *) (pstatus->dset);
	long            nRequest=1;
	long            options=0;

	if (pstatus->pact == TRUE){
		status=(*pdset->read_status)(pstatus);
		return(status);
	}

	status = dbGetLink( &(pstatus->siml), DBR_ENUM, &(pstatus->simm),
                            &options, &nRequest );
	if (status)
		return(status);

	if (pstatus->simm == NO){
		status=(*pdset->read_status)(pstatus);
		return(status);
	}
	if (pstatus->simm == YES){
		status = dbGetLink( &(pstatus->siol), DBR_LONG, &(pstatus->sval),
                                    &options, &nRequest );
		if (status==0){
			pstatus->val=pstatus->sval;
			pstatus->udf=FALSE;
		}
	} else {
		status=-1;
		recGblSetSevr(pstatus,SOFT_ALARM,INVALID_ALARM);
		return(status);
	}
        recGblSetSevr(pstatus,SIMM_ALARM,pstatus->sims);

	return(status);
}

static void decodeBits(pstatus)
	struct statusRecord    *pstatus;
{
  if (pstatus->lval != pstatus->val) {
    if ((pstatus->lval & 0x0000ffff) != (pstatus->val & 0x0000ffff)) {
	if ((pstatus->lval & 0x0000000f) != (pstatus->val & 0x0000000f)) {
	  pstatus->bi00 = (pstatus->val & 0x00000001) ? 1 : 0;
	  pstatus->bi01 = (pstatus->val & 0x00000002) ? 1 : 0;
	  pstatus->bi02 = (pstatus->val & 0x00000004) ? 1 : 0;
	  pstatus->bi03 = (pstatus->val & 0x00000008) ? 1 : 0;
	}
	if ((pstatus->lval & 0x000000f0) != (pstatus->val & 0x000000f0)) {
	  pstatus->bi04 = (pstatus->val & 0x00000010) ? 1 : 0;
	  pstatus->bi05 = (pstatus->val & 0x00000020) ? 1 : 0;
	  pstatus->bi06 = (pstatus->val & 0x00000040) ? 1 : 0;
	  pstatus->bi07 = (pstatus->val & 0x00000080) ? 1 : 0;
	}
	if ((pstatus->lval & 0x00000f00) != (pstatus->val & 0x00000f00)) {
	  pstatus->bi08 = (pstatus->val & 0x00000100) ? 1 : 0;
	  pstatus->bi09 = (pstatus->val & 0x00000200) ? 1 : 0;
	  pstatus->bi10 = (pstatus->val & 0x00000400) ? 1 : 0;
	  pstatus->bi11 = (pstatus->val & 0x00000800) ? 1 : 0;
	}
	if ((pstatus->lval & 0x0000f000) != (pstatus->val & 0x0000f000)) {
	  pstatus->bi12 = (pstatus->val & 0x00001000) ? 1 : 0;
	  pstatus->bi13 = (pstatus->val & 0x00002000) ? 1 : 0;
	  pstatus->bi14 = (pstatus->val & 0x00004000) ? 1 : 0;
	  pstatus->bi15 = (pstatus->val & 0x00008000) ? 1 : 0;
	}
    }
    if ((pstatus->lval & 0xffff0000) != (pstatus->val & 0xffff0000)) {
	if ((pstatus->lval & 0x000f0000) != (pstatus->val & 0x000f0000)) {
	  pstatus->bi16 = (pstatus->val & 0x00010000) ? 1 : 0;
	  pstatus->bi17 = (pstatus->val & 0x00020000) ? 1 : 0;
	  pstatus->bi18 = (pstatus->val & 0x00040000) ? 1 : 0;
	  pstatus->bi19 = (pstatus->val & 0x00080000) ? 1 : 0;
	}
	if ((pstatus->lval & 0x00f00000) != (pstatus->val & 0x00f00000)) {
	  pstatus->bi20 = (pstatus->val & 0x00100000) ? 1 : 0;
	  pstatus->bi21 = (pstatus->val & 0x00200000) ? 1 : 0;
	  pstatus->bi22 = (pstatus->val & 0x00400000) ? 1 : 0;
	  pstatus->bi23 = (pstatus->val & 0x00800000) ? 1 : 0;
	}
	if ((pstatus->lval & 0x0f000000) != (pstatus->val & 0x0f000000)) {
	  pstatus->bi24 = (pstatus->val & 0x01000000) ? 1 : 0;
	  pstatus->bi25 = (pstatus->val & 0x02000000) ? 1 : 0;
	  pstatus->bi26 = (pstatus->val & 0x04000000) ? 1 : 0;
	  pstatus->bi27 = (pstatus->val & 0x08000000) ? 1 : 0;
	}
	if ((pstatus->lval & 0xf0000000) != (pstatus->val & 0xf0000000)) {
	  pstatus->bi28 = (pstatus->val & 0x10000000) ? 1 : 0;
	  pstatus->bi29 = (pstatus->val & 0x20000000) ? 1 : 0;
	  pstatus->bi30 = (pstatus->val & 0x40000000) ? 1 : 0;
	  pstatus->bi31 = (pstatus->val & 0x80000000) ? 1 : 0;
	}
    }
  }
}

static void activateNewLink(pstatus)
	struct statusRecord    *pstatus;
{
  if (pstatus->lval != pstatus->val) {
    if ((pstatus->lval & 0x0000ffff) != (pstatus->val & 0x0000ffff)) {
      if ((pstatus->lval & 0x000000ff) != (pstatus->val & 0x000000ff)) {
	if ((pstatus->lval & 0x00000001) != (pstatus->val & 0x00000001)) {
	  if (pstatus->lk00.type == DB_LINK) {
	    dbScanFwdLink( &(pstatus->lk00) );
	  }
	}
	if ((pstatus->lval & 0x00000002) != (pstatus->val & 0x00000002)) {
	  if (pstatus->lk01.type == DB_LINK) {
	    dbScanFwdLink( &(pstatus->lk01) );
	  }
	}
	if ((pstatus->lval & 0x00000004) != (pstatus->val & 0x00000004)) {
	  if (pstatus->lk02.type == DB_LINK) {
	    dbScanFwdLink( &(pstatus->lk02) );
	  }
	}
	if ((pstatus->lval & 0x00000008) != (pstatus->val & 0x00000008)) {
	  if (pstatus->lk03.type == DB_LINK) {
	    dbScanFwdLink( &(pstatus->lk03) );
	  }
	}
	if ((pstatus->lval & 0x00000010) != (pstatus->val & 0x00000010)) {
	  if (pstatus->lk04.type == DB_LINK) {
	    dbScanFwdLink( &(pstatus->lk04) );
	  }
	}
	if ((pstatus->lval & 0x00000020) != (pstatus->val & 0x00000020)) {
	  if (pstatus->lk05.type == DB_LINK) {
	    dbScanFwdLink( &(pstatus->lk05) );
	  }
	}
	if ((pstatus->lval & 0x00000040) != (pstatus->val & 0x00000040)) {
	  if (pstatus->lk06.type == DB_LINK) {
	    dbScanFwdLink( &(pstatus->lk06) );
	  }
	}
	if ((pstatus->lval & 0x00000080) != (pstatus->val & 0x00000080)) {
	  if (pstatus->lk07.type == DB_LINK) {
	    dbScanFwdLink( &(pstatus->lk07) );
	  }
	}
      }
      if ((pstatus->lval & 0x0000ff00) != (pstatus->val & 0x0000ff00)) {
	if ((pstatus->lval & 0x00000100) != (pstatus->val & 0x00000100)) {
	  if (pstatus->lk08.type == DB_LINK) {
	    dbScanFwdLink( &(pstatus->lk08) );
	  }
	}
	if ((pstatus->lval & 0x00000200) != (pstatus->val & 0x00000200)) {
	  if (pstatus->lk09.type == DB_LINK) {
	    dbScanFwdLink( &(pstatus->lk09) );
	  }
	}
	if ((pstatus->lval & 0x00000400) != (pstatus->val & 0x00000400)) {
	  if (pstatus->lk10.type == DB_LINK) {
	    dbScanFwdLink( &(pstatus->lk10) );
	  }
	}
	if ((pstatus->lval & 0x00000800) != (pstatus->val & 0x00000800)) {
	  if (pstatus->lk11.type == DB_LINK) {
	    dbScanFwdLink( &(pstatus->lk11) );
	  }
	}
	if ((pstatus->lval & 0x00001000) != (pstatus->val & 0x00001000)) {
	  if (pstatus->lk12.type == DB_LINK) {
	    dbScanFwdLink( &(pstatus->lk12) );
	  }
	}
	if ((pstatus->lval & 0x00002000) != (pstatus->val & 0x00002000)) {
	  if (pstatus->lk13.type == DB_LINK) {
	    dbScanFwdLink( &(pstatus->lk13) );
	  }
	}
	if ((pstatus->lval & 0x00004000) != (pstatus->val & 0x00004000)) {
	  if (pstatus->lk14.type == DB_LINK) {
	    dbScanFwdLink( &(pstatus->lk14) );
	  }
	}
	if ((pstatus->lval & 0x00008000) != (pstatus->val & 0x00008000)) {
	  if (pstatus->lk15.type == DB_LINK) {
	    dbScanFwdLink( &(pstatus->lk15) );
	  }
	}
      }
    }
    if ((pstatus->lval & 0xffff0000) != (pstatus->val & 0xffff0000)) {
      if ((pstatus->lval & 0x00ff0000) != (pstatus->val & 0x00ff0000)) {
	if ((pstatus->lval & 0x00010000) != (pstatus->val & 0x00010000)) {
	  if (pstatus->lk16.type == DB_LINK) {
	    dbScanFwdLink( &(pstatus->lk16) );
	  }
	}
	if ((pstatus->lval & 0x00020000) != (pstatus->val & 0x00020000)) {
	  if (pstatus->lk17.type == DB_LINK) {
	    dbScanFwdLink( &(pstatus->lk17) );
	  }
	}
	if ((pstatus->lval & 0x00040000) != (pstatus->val & 0x00040000)) {
	  if (pstatus->lk18.type == DB_LINK) {
	    dbScanFwdLink( &(pstatus->lk18) );
	  }
	}
	if ((pstatus->lval & 0x00080000) != (pstatus->val & 0x00080000)) {
	  if (pstatus->lk19.type == DB_LINK) {
	    dbScanFwdLink( &(pstatus->lk19) );
	  }
	}
	if ((pstatus->lval & 0x00100000) != (pstatus->val & 0x00100000)) {
	  if (pstatus->lk20.type == DB_LINK) {
	    dbScanFwdLink( &(pstatus->lk20) );
	  }
	}
	if ((pstatus->lval & 0x00200000) != (pstatus->val & 0x00200000)) {
	  if (pstatus->lk21.type == DB_LINK) {
	    dbScanFwdLink( &(pstatus->lk21) );
	  }
	}
	if ((pstatus->lval & 0x00400000) != (pstatus->val & 0x00400000)) {
	  if (pstatus->lk22.type == DB_LINK) {
	    dbScanFwdLink( &(pstatus->lk22) );
	  }
	}
	if ((pstatus->lval & 0x00800000) != (pstatus->val & 0x00800000)) {
	  if (pstatus->lk23.type == DB_LINK) {
	    dbScanFwdLink( &(pstatus->lk23) );
	  }
	}
      }
      if ((pstatus->lval & 0xff000000) != (pstatus->val & 0xff000000)) {
	if ((pstatus->lval & 0x01000000) != (pstatus->val & 0x01000000)) {
	  if (pstatus->lk24.type == DB_LINK) {
	    dbScanFwdLink( &(pstatus->lk24) );
	  }
	}
	if ((pstatus->lval & 0x02000000) != (pstatus->val & 0x02000000)) {
	  if (pstatus->lk25.type == DB_LINK) {
	    dbScanFwdLink( &(pstatus->lk25) );
	  }
	}
	if ((pstatus->lval & 0x04000000) != (pstatus->val & 0x04000000)) {
	  if (pstatus->lk26.type == DB_LINK) {
	    dbScanFwdLink( &(pstatus->lk26) );
	  }
	}
	if ((pstatus->lval & 0x08000000) != (pstatus->val & 0x08000000)) {
	  if (pstatus->lk27.type == DB_LINK) {
	    dbScanFwdLink( &(pstatus->lk27) );
	  }
	}
	if ((pstatus->lval & 0x10000000) != (pstatus->val & 0x10000000)) {
	  if (pstatus->lk28.type == DB_LINK) {
	    dbScanFwdLink( &(pstatus->lk28) );
	  }
	}
	if ((pstatus->lval & 0x20000000) != (pstatus->val & 0x20000000)) {
	  if (pstatus->lk29.type == DB_LINK) {
	    dbScanFwdLink( &(pstatus->lk29) );
	  }
	}
	if ((pstatus->lval & 0x40000000) != (pstatus->val & 0x40000000)) {
	  if (pstatus->lk30.type == DB_LINK) {
	    dbScanFwdLink( &(pstatus->lk30) );
	  }
	}
	if ((pstatus->lval & 0x80000000) != (pstatus->val & 0x80000000)) {
	  if (pstatus->lk31.type == DB_LINK) {
	    dbScanFwdLink( &(pstatus->lk31) );
	  }
	}
      }
    }
    pstatus->lval = pstatus->val;
  }
}

static void activateAllLink(pstatus)
	struct statusRecord    *pstatus;
{
	  if (pstatus->lk00.type == DB_LINK) {
	    dbScanFwdLink( &(pstatus->lk00) );
	  }
	  if (pstatus->lk01.type == DB_LINK) {
	    dbScanFwdLink( &(pstatus->lk01) );
	  }
	  if (pstatus->lk02.type == DB_LINK) {
	    dbScanFwdLink( &(pstatus->lk02) );
	  }
	  if (pstatus->lk03.type == DB_LINK) {
	    dbScanFwdLink( &(pstatus->lk03) );
	  }
	  if (pstatus->lk04.type == DB_LINK) {
	    dbScanFwdLink( &(pstatus->lk04) );
	  }
	  if (pstatus->lk05.type == DB_LINK) {
	    dbScanFwdLink( &(pstatus->lk05) );
	  }
	  if (pstatus->lk06.type == DB_LINK) {
	    dbScanFwdLink( &(pstatus->lk06) );
	  }
	  if (pstatus->lk07.type == DB_LINK) {
	    dbScanFwdLink( &(pstatus->lk07) );
	  }
	  if (pstatus->lk08.type == DB_LINK) {
	    dbScanFwdLink( &(pstatus->lk08) );
	  }
	  if (pstatus->lk09.type == DB_LINK) {
	    dbScanFwdLink( &(pstatus->lk09) );
	  }
	  if (pstatus->lk10.type == DB_LINK) {
	    dbScanFwdLink( &(pstatus->lk10) );
	  }
	  if (pstatus->lk11.type == DB_LINK) {
	    dbScanFwdLink( &(pstatus->lk11) );
	  }
	  if (pstatus->lk12.type == DB_LINK) {
	    dbScanFwdLink( &(pstatus->lk12) );
	  }
	  if (pstatus->lk13.type == DB_LINK) {
	    dbScanFwdLink( &(pstatus->lk13) );
	  }
	  if (pstatus->lk14.type == DB_LINK) {
	    dbScanFwdLink( &(pstatus->lk14) );
	  }
	  if (pstatus->lk15.type == DB_LINK) {
	    dbScanFwdLink( &(pstatus->lk15) );
	  }
	  if (pstatus->lk16.type == DB_LINK) {
	    dbScanFwdLink( &(pstatus->lk16) );
	  }
	  if (pstatus->lk17.type == DB_LINK) {
	    dbScanFwdLink( &(pstatus->lk17) );
	  }
	  if (pstatus->lk18.type == DB_LINK) {
	    dbScanFwdLink( &(pstatus->lk18) );
	  }
	  if (pstatus->lk19.type == DB_LINK) {
	    dbScanFwdLink( &(pstatus->lk19) );
	  }
	  if (pstatus->lk20.type == DB_LINK) {
	    dbScanFwdLink( &(pstatus->lk20) );
	  }
	  if (pstatus->lk21.type == DB_LINK) {
	    dbScanFwdLink( &(pstatus->lk21) );
	  }
	  if (pstatus->lk22.type == DB_LINK) {
	    dbScanFwdLink( &(pstatus->lk22) );
	  }
	  if (pstatus->lk23.type == DB_LINK) {
	    dbScanFwdLink( &(pstatus->lk23) );
	  }
	  if (pstatus->lk24.type == DB_LINK) {
	    dbScanFwdLink( &(pstatus->lk24) );
	  }
	  if (pstatus->lk25.type == DB_LINK) {
	    dbScanFwdLink( &(pstatus->lk25) );
	  }
	  if (pstatus->lk26.type == DB_LINK) {
	    dbScanFwdLink( &(pstatus->lk26) );
	  }
	  if (pstatus->lk27.type == DB_LINK) {
	    dbScanFwdLink( &(pstatus->lk27) );
	  }
	  if (pstatus->lk28.type == DB_LINK) {
	    dbScanFwdLink( &(pstatus->lk28) );
	  }
	  if (pstatus->lk29.type == DB_LINK) {
	    dbScanFwdLink( &(pstatus->lk29) );
	  }
	  if (pstatus->lk30.type == DB_LINK) {
	    dbScanFwdLink( &(pstatus->lk30) );
	  }
	  if (pstatus->lk31.type == DB_LINK) {
	    dbScanFwdLink( &(pstatus->lk31) );
	  }
    pstatus->lval = pstatus->val;
}


static long get_precision( struct dbAddr *paddr, long *pprecision )
{
  *pprecision = 1;
  return 0;
}
