/* recTsub.c */
/* share/src/rec @(#)recTsub.c	1.19     6/4/93 */

/* recTsub.c - Record Support Routines for Subroutine records */
/*
 *      Original Author: Bob Dalesio
 *      Current Author:  Marty Kraimer
 *      Date:            01-25-90
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
 * .01  10-10-90	mrk	Made changes for new record support
 * .02  11-11-91        jba     Moved set and reset of alarm stat and sevr to macros
 * .03  01-08-92        jba     Added casts in symFindByName to avoid compile warning messages
 * .04  02-05-92	jba	Changed function arguments from paddr to precord
 * .05  02-28-92        jba     Changed get_precision,get_graphic_double,get_control_double
 * .06  02-28-92	jba	ANSI C changes
 * .07  04-10-92        jba     pact now used to test for asyn processing, not status
 * .08  06-02-92        jba     changed graphic/control limits for hihi,high,low,lolo
 * .09  07-15-92        jba     changed VALID_ALARM to INVALID alarm
 * .10  07-16-92        jba     added invalid alarm fwd link test and chngd fwd lnk to macro
 * .11  07-21-92        jba     changed alarm limits for non val related fields
 * .12  08-06-92        jba     New algorithm for calculating analog alarms
 * .13  08-06-92        jba     monitor now posts events for changes in a-l
 * .14  10-10-92        jba     replaced code with recGblGetLinkValue call
 * .15  10-18-93        mhb     Built big subroutine record from sub record
 * .16  02-05-95	jt	use update to r3.12
 */

#include	<vxWorks.h>
#include	<types.h>
#include	<stdioLib.h>
#include	<lstLib.h>
#include	<string.h>
#include	<symLib.h>
#include	<sysSymTbl.h>   /* for sysSymTbl*/
#include	<a_out.h>       /* for N_TEXT */

#include	<alarm.h>
#include	<dbDefs.h>
#include	<dbAccess.h>
#include	<dbFldTypes.h>
#include	<errMdef.h>
#include	<recSup.h>
#include	<dbEvent.h>		/* Sergey */
#include	<epicsDynLink.h>	/* Sergey */

#define GEN_SIZE_OFFSET
#include	<tsubRecord.h>
#undef  GEN_SIZE_OFFSET
#include "recGbl.h"
#include "epicsExport.h"

/* Create RSET - Record Support Entry Table*/
#define report NULL
#define initialize NULL
static long init_record();
static long process();
#define special NULL
static long get_value();
#define cvt_dbaddr NULL
#define get_array_info NULL
#define put_array_info NULL
static long get_units();
static long get_precision();
#define get_enum_str NULL
#define get_enum_strs NULL
#define put_enum_str NULL
static long get_graphic_double();
static long get_control_double();
static long get_alarm_double();

rset tsubRSET={
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
epicsExportAddress(rset,tsubRSET);

static void set_alarms();
static long do_sub();
static long fetch_values();
static long push_values();
static void monitor();

#define IN_ARG_MAX 20
#define INP_ARG_MAX 80
#define OUT_ARG_MAX 70
/* Fldnames should have as many as INP_ARG_MAX */


static long init_record(ptsub,pass)
    struct tsubRecord	*ptsub;
    int pass;
{
    FUNCPTR	psubroutine;
    char	sub_type;
    char	temp[40];
    long	status;
    STATUS	ret;
    struct link *plink;
    int i;
    double *pvalue;

    if (pass==0) {
      return(0);
    }

    plink = &ptsub->inpa;
    pvalue = &ptsub->a;
    for(i=0; i<INP_ARG_MAX; i++, plink++, pvalue++) {
        if(plink->type==CONSTANT){
           recGblInitConstantLink(plink,DBF_DOUBLE,pvalue);
	   /* *pvalue = plink->value.value; */
	}
    }

    /* convert the initialization subroutine name  */
    temp[0] = 0;			/* all global variables start with _ */
    if (ptsub->inam[0] != '_'){
	strcpy(temp,"_");
    }
    strcat(temp,ptsub->inam);
	/* Sergey (symFindByName - > symFindByNameEPICS) */
	ret = symFindByNameEPICS(sysSymTbl,temp,(void *)&ptsub->sadr,(void *)&sub_type);
    if ((ret !=OK) || ((sub_type & N_TEXT) == 0)){
	recGblRecordError(S_db_BadSub,(void *)ptsub,"recTsub(init_record)");
	return(S_db_BadSub);
    }

    /* invoke the initialization subroutine */
    psubroutine = (FUNCPTR)(ptsub->sadr);
    status = psubroutine(ptsub,process);

    /* convert the subroutine name to an address and type */
    /* convert the processing subroutine name  */
    temp[0] = 0;			/* all global variables start with _ */
    if (ptsub->snam[0] != '_'){
    	strcpy(temp,"_");
    }
    strcat(temp,ptsub->snam);
	/* Sergey (symFindByName - > symFindByNameEPICS) */
    ret = symFindByNameEPICS(sysSymTbl, temp, (void *)&ptsub->sadr, (void *)&sub_type);
    if ((ret < 0) || ((sub_type & N_TEXT) == 0)){
	recGblRecordError(S_db_BadSub,(void *)ptsub,"recTsub(init_record)");
	return(S_db_BadSub);
    }
    ptsub->styp = sub_type;
    return(0);
}

static long process(ptsub)
	struct tsubRecord *ptsub;
{
	long		 status=0;
	unsigned char	 pact=ptsub->pact;

        if(!ptsub->pact){
		ptsub->pact = TRUE;
		status = fetch_values(ptsub);
		ptsub->pact = FALSE;
	}
        if(status==0) status = do_sub(ptsub);
	if(!pact && ptsub->pact) {
	  return(0);
	}
        ptsub->pact = TRUE;
	if(status==1) {
	  return(0);
	}
	recGblGetTimeStamp(ptsub);
        /* check for alarms */
        set_alarms(ptsub);
        /* check event list */
        monitor(ptsub);

	/* Push out the output link data values */
	status = push_values(ptsub);

        /* process the forward scan link record */
        recGblFwdLink(ptsub);

        ptsub->pact = FALSE;
        return(status);
}

static long get_value(ptsub,pvdes)
    struct tsubRecord		*ptsub;
    struct valueDes	*pvdes;
{
    pvdes->field_type = DBF_DOUBLE;
    pvdes->no_elements=1;
/*  (double *)(pvdes->pvalue) = &ptsub->val; */
/* Sergey */
    pvdes->pvalue = (double *)(&ptsub->val);
    return(0);
}

static long get_units(paddr,units)
    struct dbAddr *paddr;
    char	  *units;
{
    struct tsubRecord	*ptsub=(struct tsubRecord *)paddr->precord;

    strncpy(units,ptsub->egu,sizeof(ptsub->egu));
    return(0);
}

static long get_precision(paddr,precision)
    struct dbAddr *paddr;
    long	  *precision;
{
    struct tsubRecord	*ptsub=(struct tsubRecord *)paddr->precord;

    *precision = ptsub->prec;
    if(paddr->pfield==(void *)&ptsub->val) return(0);
    recGblGetPrec(paddr,precision);
    return(0);
}


static long get_graphic_double(paddr,pgd)
    struct dbAddr *paddr;
    struct dbr_grDouble	*pgd;
{
    struct tsubRecord	*ptsub=(struct tsubRecord *)paddr->precord;

    if(paddr->pfield==(void *)&ptsub->val
    || paddr->pfield==(void *)&ptsub->hihi
    || paddr->pfield==(void *)&ptsub->high
    || paddr->pfield==(void *)&ptsub->low
    || paddr->pfield==(void *)&ptsub->lolo){
        pgd->upper_disp_limit = ptsub->hopr;
        pgd->lower_disp_limit = ptsub->lopr;
        return(0);
    }

    if(paddr->pfield>=(void *)&ptsub->a && paddr->pfield<=(void *)&ptsub->t){
        pgd->upper_disp_limit = ptsub->hopr;
        pgd->lower_disp_limit = ptsub->lopr;
        return(0);
    }
    if(paddr->pfield>=(void *)&ptsub->la && paddr->pfield<=(void *)&ptsub->lt){
        pgd->upper_disp_limit = ptsub->hopr;
        pgd->lower_disp_limit = ptsub->lopr;
        return(0);
    }
    return(0);
}

static long get_control_double(paddr,pcd)
    struct dbAddr *paddr;
    struct dbr_ctrlDouble *pcd;
{
    struct tsubRecord	*ptsub=(struct tsubRecord *)paddr->precord;

    if(paddr->pfield==(void *)&ptsub->val
    || paddr->pfield==(void *)&ptsub->hihi
    || paddr->pfield==(void *)&ptsub->high
    || paddr->pfield==(void *)&ptsub->low
    || paddr->pfield==(void *)&ptsub->lolo){
        pcd->upper_ctrl_limit = ptsub->hopr;
        pcd->lower_ctrl_limit = ptsub->lopr;
       return(0);
    }

    if(paddr->pfield>=(void *)&ptsub->a && paddr->pfield<=(void *)&ptsub->t){
        pcd->upper_ctrl_limit = ptsub->hopr;
        pcd->lower_ctrl_limit = ptsub->lopr;
        return(0);
    }
    if(paddr->pfield>=(void *)&ptsub->la && paddr->pfield<=(void *)&ptsub->lt){
        pcd->upper_ctrl_limit = ptsub->hopr;
        pcd->lower_ctrl_limit = ptsub->lopr;
        return(0);
    }
    return(0);
}

static long get_alarm_double(paddr,pad)
    struct dbAddr *paddr;
    struct dbr_alDouble	*pad;
{
    struct tsubRecord	*ptsub=(struct tsubRecord *)paddr->precord;

    if(paddr->pfield==(void *)&ptsub->val){
         pad->upper_alarm_limit = ptsub->hihi;
         pad->upper_warning_limit = ptsub->high;
         pad->lower_warning_limit = ptsub->low;
         pad->lower_alarm_limit = ptsub->lolo;
    } else recGblGetAlarmDouble(paddr,pad);
    return(0);
}

static void set_alarms(ptsub)
    struct tsubRecord	*ptsub;
{
	double		val;
	float		hyst, lalm, hihi, high, low, lolo;
	unsigned short	hhsv, llsv, hsv, lsv;

	if(ptsub->udf == TRUE ){
 		recGblSetSevr(ptsub,UDF_ALARM,INVALID_ALARM);
		return;
	}
	hihi = ptsub->hihi; lolo = ptsub->lolo; high = ptsub->high; low = ptsub->low;
	hhsv = ptsub->hhsv; llsv = ptsub->llsv; hsv = ptsub->hsv; lsv = ptsub->lsv;
	val = ptsub->val; hyst = ptsub->hyst; lalm = ptsub->lalm;

	/* alarm condition hihi */
	if (hhsv && (val >= hihi || ((lalm==hihi) && (val >= hihi-hyst)))){
	        if (recGblSetSevr(ptsub,HIHI_ALARM,ptsub->hhsv)) ptsub->lalm = hihi;
		return;
	}

	/* alarm condition lolo */
	if (llsv && (val <= lolo || ((lalm==lolo) && (val <= lolo+hyst)))){
	        if (recGblSetSevr(ptsub,LOLO_ALARM,ptsub->llsv)) ptsub->lalm = lolo;
		return;
	}

	/* alarm condition high */
	if (hsv && (val >= high || ((lalm==high) && (val >= high-hyst)))){
	        if (recGblSetSevr(ptsub,HIGH_ALARM,ptsub->hsv)) ptsub->lalm = high;
		return;
	}

	/* alarm condition low */
	if (lsv && (val <= low || ((lalm==low) && (val <= low+hyst)))){
	        if (recGblSetSevr(ptsub,LOW_ALARM,ptsub->lsv)) ptsub->lalm = low;
		return;
	}

	/* we get here only if val is out of alarm by at least hyst */
	ptsub->lalm = val;
	return;
}

static void monitor(ptsub)
    struct tsubRecord	*ptsub;
{
	unsigned short	monitor_mask;
	double		delta;
	double           *pnew;
	double           *pprev;
	int             i;

        monitor_mask = recGblResetAlarms(ptsub);
        monitor_mask |= (DBE_LOG|DBE_VALUE);
        if(monitor_mask)
        db_post_events(ptsub,(void *)&(ptsub->val),monitor_mask);
        /* check for value change */
        delta = ptsub->mlst - ptsub->val;
        if(delta<0.0) delta = -delta;
        if (delta > ptsub->mdel) {
                /* post events for value change */
                monitor_mask |= DBE_VALUE;
                /* update last value monitored */
                ptsub->mlst = ptsub->val;
        }
        /* check for archive change */
        delta = ptsub->alst - ptsub->val;
        if(delta<0.0) delta = -delta;
        if (delta > ptsub->adel) {
                /* post events on value field for archive change */
                monitor_mask |= DBE_LOG;
                /* update last archive value monitored */
                ptsub->alst = ptsub->val;
        }
        /* send out monitors connected to the value field */
        if (monitor_mask){
                db_post_events(ptsub,&ptsub->val,monitor_mask);
        }
	/* check all link input fields for changes */
	for(i=0, pnew=&ptsub->a, pprev=&ptsub->la; i<INP_ARG_MAX; i++, pnew++, pprev++) {
		if(*pnew != *pprev) {
			db_post_events(ptsub,pnew,monitor_mask|DBE_VALUE);
			*pprev = *pnew;
		}
	}
	/* check all non-link input fields for changes */
	for(i=0, pnew=&ptsub->nla, pprev=&ptsub->lnla; i<IN_ARG_MAX; i++, pnew++, pprev++) {
		if(*pnew != *pprev) {
			db_post_events(ptsub,pnew,monitor_mask|DBE_VALUE);
			*pprev = *pnew;
		}
	}
	/* check all output fields for changes */
	for(i=0, pnew=&ptsub->oa, pprev=&ptsub->loa; i<OUT_ARG_MAX; i++, pnew++, pprev++) {
		if(*pnew != *pprev) {
			db_post_events(ptsub,pnew,monitor_mask|DBE_VALUE);
			*pprev = *pnew;
		}
	}
        return;
}

static long fetch_values(ptsub)
struct tsubRecord *ptsub;
{
        struct link     *plink; /* structure of the link field  */
        double           *pvalue;
        int             i;
	long		status;

        for(i=0, plink=&ptsub->inpa, pvalue=&ptsub->a;
        		i<INP_ARG_MAX; i++, plink++, pvalue++)
        {
		status=dbGetLink(plink,DBR_DOUBLE, pvalue,0,0);
		/*status=recGblGetFastLink(plink,(void *)ptsub,pvalue);*/
		if (!RTN_SUCCESS(status)) return(-1);
        }
        return(0);
}

static long push_values(ptsub)
struct tsubRecord *ptsub;
{
        struct link     *plink; /* structure of the link field  */
        double           *pvalue;
        int             i;
	long		status;

        for(i=0, plink=&ptsub->outa, pvalue=&ptsub->oa;
        		i<OUT_ARG_MAX; i++, plink++, pvalue++)
        {
		status=dbPutLink(plink,DBR_DOUBLE, pvalue,0);
		/*status=recGblPutFastLink(plink,(void *)ptsub,pvalue);*/
		if (!RTN_SUCCESS(status)) return(-1);
        }
        return(0);
}

static long do_sub(ptsub)
struct tsubRecord *ptsub;  /* pointer to subroutine record  */
{
	long	status;
	FUNCPTR	psubroutine;


	/* call the subroutine */
	psubroutine = (FUNCPTR)(ptsub->sadr);
	if(psubroutine==NULL) {
               	recGblSetSevr(ptsub,BAD_SUB_ALARM,INVALID_ALARM);
		return(0);
	}
	status = psubroutine(ptsub);
	if(status < 0){
               	recGblSetSevr(ptsub,SOFT_ALARM,ptsub->brsv);
	} else ptsub->udf = FALSE;
	return(status);
}
