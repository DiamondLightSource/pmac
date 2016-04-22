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
 * .03  02-28-92        jba     Changed get_precision, get_graphic_double, get_control_double
 * .04  02-28-92	jba	ANSI C changes
 * .05  04-10-92        jba     pact now used to test for asyn processing, not status
 * .06  04-18-92        jba     removed process from dev init_record parms
 * .07  06-02-92        jba     changed graphic/control limits for hihi, high, low, lolo
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

/* Modification by Rok Gajsek <rok.gajsek@cosylab.com>
 * ---------------------------------------------------
 * removed :
 *	#include <vxWorks.h>
 *	#include <lstLib.h>
 * in order for the status record to work on linux
 */

#define VERSION 1.1

#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include <alarm.h>
#include <dbDefs.h>
#include <dbEvent.h>
#include <dbAccess.h> /* dbGetLink () */
#include <dbFldTypes.h>
#include <errMdef.h>
#include <recSup.h>
#include <devSup.h>

#define GEN_SIZE_OFFSET
#include <statusRecord.h>
#undef GEN_SIZE_OFFSET
#include "epicsTime.h"
#include "recGbl.h"
#include "epicsExport.h"

/* Create RSET - Record Support Entry Table*/
static long init_record ();
static long process ();
static long get_units ();
static long get_graphic_double ();
static long get_control_double ();
static long get_alarm_double ();
static long get_precision ();
#define report         NULL
#define initialize     NULL
#define special        NULL
#define cvt_dbaddr     NULL
#define get_value      NULL
#define get_array_info NULL
#define put_array_info NULL
#define get_enum_str   NULL
#define get_enum_strs  NULL
#define put_enum_str   NULL

rset statusRSET = {
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
  get_alarm_double
};
epicsExportAddress (rset, statusRSET);

struct statusdset { /* status input dset */
  long      number;
  DEVSUPFUN dev_report;
  DEVSUPFUN init;
  DEVSUPFUN init_record; /*returns: (-1, 0)=>(failure, success)*/
  DEVSUPFUN get_ioint_info;
  DEVSUPFUN read_status; /*returns: (-1, 0)=>(failure, success)*/
};
static void alarm ();
static void monitor ();
static long readValue ();
static void decodeBits ();
static void activateAllLink ();
static void activateNewLink ();


static long init_record (struct statusRecord *pRec, int pass) {
  struct statusdset *pdset;
  long status;

  if (pass == 0) {
    pRec->vers = VERSION;
    return 0;
  }

  if (pRec->siml.type == CONSTANT)
    recGblInitConstantLink (&pRec->siml, DBF_ENUM, &pRec->simm);

  if (pRec->siol.type == CONSTANT)
    recGblInitConstantLink (&pRec->siol, DBF_LONG, &pRec->sval);

  if (!(pdset = (struct statusdset *) (pRec->dset))) {
    recGblRecordError (S_dev_noDSET, (void *) pRec, "statusRecord: init_record");
    return S_dev_noDSET;
  }
  /* must have read_status function defined */
  if ((pdset->number < 5) || (pdset->read_status == NULL)) {
    recGblRecordError (S_dev_missingSup, (void *) pRec, "statusRecord: init_record");
    return S_dev_missingSup;
  }
  if (pdset->init_record) {
    if ((status = (*pdset->init_record) (pRec))) return status;
  }
  return 0;
}

static long process (struct statusRecord *pRec) {
  struct statusdset *pdset = (struct statusdset *) (pRec->dset);
  long  	    status;
  unsigned char     pact = pRec->pact;

  if ((pdset == NULL) || (pdset->read_status == NULL)) {
    pRec->pact = TRUE;
    recGblRecordError (S_dev_missingSup, (void *) pRec, "statusRecord: read_status");
    return S_dev_missingSup;
  }

  status = readValue (pRec); /* read the new value */
  /* check if device support set pact */
  if (!pact && pRec->pact) return 0;
  pRec->pact = TRUE;

  epicsTimeGetCurrent (&pRec->time);

  /* check for alarms */
  alarm (pRec);
  /* check event list */
  monitor (pRec);
  /* decode the bits */
  decodeBits (pRec);
  /* activate links for changed bits */
  activateNewLink (pRec);
  /* process the forward scan link record */
  recGblFwdLink (pRec);

  pRec->pact = FALSE;
  return status;
}


static long get_units (struct dbAddr *paddr, char *units) {
  struct statusRecord *pRec = (struct statusRecord *) paddr->precord;

  strncpy (units, pRec->egu, sizeof (pRec->egu));
  return 0;
}


static long get_graphic_double (struct dbAddr *paddr, struct dbr_grDouble *pgd) {
  struct statusRecord *pRec = (struct statusRecord *) paddr->precord;

  if (paddr->pfield == (void *) &pRec->val
    || paddr->pfield == (void *) &pRec->hihi
    || paddr->pfield == (void *) &pRec->high
    || paddr->pfield == (void *) &pRec->low
    || paddr->pfield == (void *) &pRec->lolo) {
      pgd->upper_disp_limit = pRec->hopr;
      pgd->lower_disp_limit = pRec->lopr;
  } else recGblGetGraphicDouble (paddr, pgd);
  return 0;
}

static long get_control_double (struct dbAddr *paddr, struct dbr_ctrlDouble *pcd) {
  struct statusRecord *pRec = (struct statusRecord *) paddr->precord;

  if (paddr->pfield == (void *) &pRec->val
    || paddr->pfield == (void *) &pRec->hihi
    || paddr->pfield == (void *) &pRec->high
    || paddr->pfield == (void *) &pRec->low
    || paddr->pfield == (void *) &pRec->lolo) {
      pcd->upper_ctrl_limit = pRec->hopr;
      pcd->lower_ctrl_limit = pRec->lopr;
  } else recGblGetControlDouble (paddr, pcd);
  return 0;
}

static long get_alarm_double (struct dbAddr *paddr, struct dbr_alDouble *pad) {
  struct statusRecord *pRec = (struct statusRecord *) paddr->precord;

  if (paddr->pfield == (void *) &pRec->val) {
    pad->upper_alarm_limit = pRec->hihi;
    pad->upper_warning_limit = pRec->high;
    pad->lower_warning_limit = pRec->low;
    pad->lower_alarm_limit = pRec->lolo;
  } else recGblGetAlarmDouble (paddr, pad);
  return 0;
}

static void alarm (struct statusRecord *pRec) {
  double	  val;
  float 	  hyst, lalm, hihi, high, low, lolo;
  unsigned short  hhsv, llsv, hsv, lsv;

  if (pRec->udf == TRUE ) {
    recGblSetSevr (pRec, UDF_ALARM, INVALID_ALARM);
    return;
  }
  hihi = pRec->hihi; lolo = pRec->lolo; high = pRec->high; low = pRec->low;
  hhsv = pRec->hhsv; llsv = pRec->llsv; hsv = pRec->hsv; lsv = pRec->lsv;
  val = pRec->val; hyst = pRec->hyst; lalm = pRec->lalm;

  /* alarm condition hihi */
  if (hhsv && (val >= hihi || ((lalm == hihi) && (val >= hihi-hyst)))){
    if (recGblSetSevr (pRec, HIHI_ALARM, pRec->hhsv)) pRec->lalm = hihi;
    return;
  }

  /* alarm condition lolo */
  if (llsv && (val <= lolo || ((lalm == lolo) && (val <= lolo+hyst)))){
    if (recGblSetSevr (pRec, LOLO_ALARM, pRec->llsv)) pRec->lalm = lolo;
    return;
  }

  /* alarm condition high */
  if (hsv && (val >= high || ((lalm == high) && (val >= high-hyst)))){
    if (recGblSetSevr (pRec, HIGH_ALARM, pRec->hsv)) pRec->lalm = high;
    return;
  }

  /* alarm condition low */
  if (lsv && (val <= low || ((lalm == low) && (val <= low+hyst)))){
    if (recGblSetSevr (pRec, LOW_ALARM, pRec->lsv)) pRec->lalm = low;
    return;
  }

  /* we get here only if val is out of alarm by at least hyst */
  pRec->lalm = val;
  return;
}

static void monitor (struct statusRecord *pRec) {
  unsigned short monitor_mask;
  long  	 delta;
  int		 linkActivation = FALSE;

  /* get previous stat and sevr  and new stat and sevr*/
  monitor_mask = recGblResetAlarms (pRec);
  /* notice any change of stat and sevr */
  if (monitor_mask != 0) linkActivation = TRUE;

  /* check for value change */
  delta = pRec->mlst - pRec->val;
  if (delta < 0) delta = -delta;
  if (delta > pRec->mdel) {
    /* post events for value change */
    monitor_mask |= DBE_VALUE;
    /* update last value monitored */
    pRec->mlst = pRec->val;
  }

  /* check for archive change */
  delta = pRec->alst - pRec->val;
  if (delta < 0) delta = -delta;
  if (delta > pRec->adel) {
    /* post events on value field for archive change */
    monitor_mask |= DBE_LOG;
    /* update last archive value monitored */
    pRec->alst = pRec->val;
  }

  /* send out monitors connected to the value field */
  if (monitor_mask) {
    db_post_events (pRec, &pRec->val, monitor_mask);
  }

  /* decode and activate all links if stat or sevr changed */
  if (linkActivation) {
    decodeBits (pRec);
    activateAllLink (pRec);
  }

  return;
}

static long readValue (struct statusRecord *pRec) {
  long  	    status;
  struct statusdset *pdset = (struct statusdset *) (pRec->dset);
  long  	    nRequest = 1;
  long  	    options = 0;

  if (pRec->pact == TRUE) {
    status = (*pdset->read_status) (pRec);
    return status;
  }

  status = dbGetLink (&(pRec->siml), DBR_ENUM, &(pRec->simm), &options, &nRequest);
  if (status) return status;

  if (pRec->simm == 0) {
    status = (*pdset->read_status) (pRec);
    return status;
  }
  if (pRec->simm == 1) {
    status = dbGetLink (&(pRec->siol), DBR_LONG, &(pRec->sval), &options, &nRequest);
    if (status == 0) {
      pRec->val = pRec->sval;
      pRec->udf = FALSE;
    }
  } else {
    status = -1;
    recGblSetSevr (pRec, SOFT_ALARM, INVALID_ALARM);
    return status;
  }
  recGblSetSevr (pRec, SIMM_ALARM, pRec->sims);

  return status;
}

static void decodeBits (struct statusRecord *pRec) {
  if (pRec->lval != pRec->val) {
    if ((pRec->lval & 0x0000ffff) != (pRec->val & 0x0000ffff)) {
      if ((pRec->lval & 0x0000000f) != (pRec->val & 0x0000000f)) {
        pRec->bi00 = (pRec->val & 0x00000001) ? 1 : 0;
        pRec->bi01 = (pRec->val & 0x00000002) ? 1 : 0;
        pRec->bi02 = (pRec->val & 0x00000004) ? 1 : 0;
        pRec->bi03 = (pRec->val & 0x00000008) ? 1 : 0;
      }
      if ((pRec->lval & 0x000000f0) != (pRec->val & 0x000000f0)) {
        pRec->bi04 = (pRec->val & 0x00000010) ? 1 : 0;
        pRec->bi05 = (pRec->val & 0x00000020) ? 1 : 0;
        pRec->bi06 = (pRec->val & 0x00000040) ? 1 : 0;
        pRec->bi07 = (pRec->val & 0x00000080) ? 1 : 0;
      }
      if ((pRec->lval & 0x00000f00) != (pRec->val & 0x00000f00)) {
        pRec->bi08 = (pRec->val & 0x00000100) ? 1 : 0;
        pRec->bi09 = (pRec->val & 0x00000200) ? 1 : 0;
        pRec->bi10 = (pRec->val & 0x00000400) ? 1 : 0;
        pRec->bi11 = (pRec->val & 0x00000800) ? 1 : 0;
      }
      if ((pRec->lval & 0x0000f000) != (pRec->val & 0x0000f000)) {
        pRec->bi12 = (pRec->val & 0x00001000) ? 1 : 0;
        pRec->bi13 = (pRec->val & 0x00002000) ? 1 : 0;
        pRec->bi14 = (pRec->val & 0x00004000) ? 1 : 0;
        pRec->bi15 = (pRec->val & 0x00008000) ? 1 : 0;
      }
    }
    if ((pRec->lval & 0xffff0000) != (pRec->val & 0xffff0000)) {
      if ((pRec->lval & 0x000f0000) != (pRec->val & 0x000f0000)) {
        pRec->bi16 = (pRec->val & 0x00010000) ? 1 : 0;
        pRec->bi17 = (pRec->val & 0x00020000) ? 1 : 0;
        pRec->bi18 = (pRec->val & 0x00040000) ? 1 : 0;
        pRec->bi19 = (pRec->val & 0x00080000) ? 1 : 0;
      }
      if ((pRec->lval & 0x00f00000) != (pRec->val & 0x00f00000)) {
        pRec->bi20 = (pRec->val & 0x00100000) ? 1 : 0;
        pRec->bi21 = (pRec->val & 0x00200000) ? 1 : 0;
        pRec->bi22 = (pRec->val & 0x00400000) ? 1 : 0;
        pRec->bi23 = (pRec->val & 0x00800000) ? 1 : 0;
      }
      if ((pRec->lval & 0x0f000000) != (pRec->val & 0x0f000000)) {
        pRec->bi24 = (pRec->val & 0x01000000) ? 1 : 0;
        pRec->bi25 = (pRec->val & 0x02000000) ? 1 : 0;
        pRec->bi26 = (pRec->val & 0x04000000) ? 1 : 0;
        pRec->bi27 = (pRec->val & 0x08000000) ? 1 : 0;
      }
      if ((pRec->lval & 0xf0000000) != (pRec->val & 0xf0000000)) {
        pRec->bi28 = (pRec->val & 0x10000000) ? 1 : 0;
        pRec->bi29 = (pRec->val & 0x20000000) ? 1 : 0;
        pRec->bi30 = (pRec->val & 0x40000000) ? 1 : 0;
        pRec->bi31 = (pRec->val & 0x80000000) ? 1 : 0;
      }
    }
  }
}

static void activateNewLink (struct statusRecord *pRec) {
  if (pRec->lval != pRec->val) {
    if ((pRec->lval & 0x0000ffff) != (pRec->val & 0x0000ffff)) {
      if ((pRec->lval & 0x000000ff) != (pRec->val & 0x000000ff)) {
	if ((pRec->lval & 0x00000001) != (pRec->val & 0x00000001)) {
	  if (pRec->lk00.type == DB_LINK) dbScanFwdLink (&(pRec->lk00));
	}
	if ((pRec->lval & 0x00000002) != (pRec->val & 0x00000002)) {
	  if (pRec->lk01.type == DB_LINK) dbScanFwdLink (&(pRec->lk01));
	}
	if ((pRec->lval & 0x00000004) != (pRec->val & 0x00000004)) {
	  if (pRec->lk02.type == DB_LINK) dbScanFwdLink (&(pRec->lk02));
	}
	if ((pRec->lval & 0x00000008) != (pRec->val & 0x00000008)) {
	  if (pRec->lk03.type == DB_LINK) dbScanFwdLink (&(pRec->lk03));
	}
	if ((pRec->lval & 0x00000010) != (pRec->val & 0x00000010)) {
	  if (pRec->lk04.type == DB_LINK) dbScanFwdLink (&(pRec->lk04));
	}
	if ((pRec->lval & 0x00000020) != (pRec->val & 0x00000020)) {
	  if (pRec->lk05.type == DB_LINK) dbScanFwdLink (&(pRec->lk05));
	}
	if ((pRec->lval & 0x00000040) != (pRec->val & 0x00000040)) {
	  if (pRec->lk06.type == DB_LINK) dbScanFwdLink (&(pRec->lk06));
	}
	if ((pRec->lval & 0x00000080) != (pRec->val & 0x00000080)) {
	  if (pRec->lk07.type == DB_LINK) dbScanFwdLink (&(pRec->lk07));
	}
      }
      if ((pRec->lval & 0x0000ff00) != (pRec->val & 0x0000ff00)) {
	if ((pRec->lval & 0x00000100) != (pRec->val & 0x00000100)) {
	  if (pRec->lk08.type == DB_LINK) dbScanFwdLink (&(pRec->lk08));
	}
	if ((pRec->lval & 0x00000200) != (pRec->val & 0x00000200)) {
	  if (pRec->lk09.type == DB_LINK) dbScanFwdLink (&(pRec->lk09));
	}
	if ((pRec->lval & 0x00000400) != (pRec->val & 0x00000400)) {
	  if (pRec->lk10.type == DB_LINK) dbScanFwdLink (&(pRec->lk10));
	}
	if ((pRec->lval & 0x00000800) != (pRec->val & 0x00000800)) {
	  if (pRec->lk11.type == DB_LINK) dbScanFwdLink (&(pRec->lk11));
	}
	if ((pRec->lval & 0x00001000) != (pRec->val & 0x00001000)) {
	  if (pRec->lk12.type == DB_LINK) dbScanFwdLink (&(pRec->lk12));
	}
	if ((pRec->lval & 0x00002000) != (pRec->val & 0x00002000)) {
	  if (pRec->lk13.type == DB_LINK) dbScanFwdLink (&(pRec->lk13));
	}
	if ((pRec->lval & 0x00004000) != (pRec->val & 0x00004000)) {
	  if (pRec->lk14.type == DB_LINK) dbScanFwdLink (&(pRec->lk14));
	}
	if ((pRec->lval & 0x00008000) != (pRec->val & 0x00008000)) {
	  if (pRec->lk15.type == DB_LINK) dbScanFwdLink (&(pRec->lk15));
	}
      }
    }
    if ((pRec->lval & 0xffff0000) != (pRec->val & 0xffff0000)) {
      if ((pRec->lval & 0x00ff0000) != (pRec->val & 0x00ff0000)) {
	if ((pRec->lval & 0x00010000) != (pRec->val & 0x00010000)) {
	  if (pRec->lk16.type == DB_LINK) dbScanFwdLink (&(pRec->lk16));
 	}
	if ((pRec->lval & 0x00020000) != (pRec->val & 0x00020000)) {
	  if (pRec->lk17.type == DB_LINK) dbScanFwdLink (&(pRec->lk17));
 	}
	if ((pRec->lval & 0x00040000) != (pRec->val & 0x00040000)) {
	  if (pRec->lk18.type == DB_LINK) dbScanFwdLink (&(pRec->lk18));
 	}
	if ((pRec->lval & 0x00080000) != (pRec->val & 0x00080000)) {
	  if (pRec->lk19.type == DB_LINK) dbScanFwdLink (&(pRec->lk19));
 	}
	if ((pRec->lval & 0x00100000) != (pRec->val & 0x00100000)) {
	  if (pRec->lk20.type == DB_LINK) dbScanFwdLink (&(pRec->lk20));
 	}
	if ((pRec->lval & 0x00200000) != (pRec->val & 0x00200000)) {
	  if (pRec->lk21.type == DB_LINK) dbScanFwdLink (&(pRec->lk21));
 	}
	if ((pRec->lval & 0x00400000) != (pRec->val & 0x00400000)) {
	  if (pRec->lk22.type == DB_LINK) dbScanFwdLink (&(pRec->lk22));
 	}
	if ((pRec->lval & 0x00800000) != (pRec->val & 0x00800000)) {
	  if (pRec->lk23.type == DB_LINK) dbScanFwdLink (&(pRec->lk23));
 	}
      }
      if ((pRec->lval & 0xff000000) != (pRec->val & 0xff000000)) {
	if ((pRec->lval & 0x01000000) != (pRec->val & 0x01000000)) {
	  if (pRec->lk24.type == DB_LINK) dbScanFwdLink (&(pRec->lk24));
 	}
	if ((pRec->lval & 0x02000000) != (pRec->val & 0x02000000)) {
	  if (pRec->lk25.type == DB_LINK) dbScanFwdLink (&(pRec->lk25));
 	}
	if ((pRec->lval & 0x04000000) != (pRec->val & 0x04000000)) {
	  if (pRec->lk26.type == DB_LINK) dbScanFwdLink (&(pRec->lk26));
 	}
	if ((pRec->lval & 0x08000000) != (pRec->val & 0x08000000)) {
	  if (pRec->lk27.type == DB_LINK) dbScanFwdLink (&(pRec->lk27));
 	}
	if ((pRec->lval & 0x10000000) != (pRec->val & 0x10000000)) {
	  if (pRec->lk28.type == DB_LINK) dbScanFwdLink (&(pRec->lk28));
 	}
	if ((pRec->lval & 0x20000000) != (pRec->val & 0x20000000)) {
	  if (pRec->lk29.type == DB_LINK) dbScanFwdLink (&(pRec->lk29));
 	}
	if ((pRec->lval & 0x40000000) != (pRec->val & 0x40000000)) {
	  if (pRec->lk30.type == DB_LINK) dbScanFwdLink (&(pRec->lk30));
 	}
	if ((pRec->lval & 0x80000000) != (pRec->val & 0x80000000)) {
	  if (pRec->lk31.type == DB_LINK) dbScanFwdLink (&(pRec->lk31));
 	}
      }
    }
    pRec->lval = pRec->val;
  }
}

static void activateAllLink (struct statusRecord *pRec) {
  if (pRec->lk00.type == DB_LINK) dbScanFwdLink (&(pRec->lk00));
  if (pRec->lk01.type == DB_LINK) dbScanFwdLink (&(pRec->lk01));
  if (pRec->lk02.type == DB_LINK) dbScanFwdLink (&(pRec->lk02));
  if (pRec->lk03.type == DB_LINK) dbScanFwdLink (&(pRec->lk03));
  if (pRec->lk04.type == DB_LINK) dbScanFwdLink (&(pRec->lk04));
  if (pRec->lk05.type == DB_LINK) dbScanFwdLink (&(pRec->lk05));
  if (pRec->lk06.type == DB_LINK) dbScanFwdLink (&(pRec->lk06));
  if (pRec->lk07.type == DB_LINK) dbScanFwdLink (&(pRec->lk07));
  if (pRec->lk08.type == DB_LINK) dbScanFwdLink (&(pRec->lk08));
  if (pRec->lk09.type == DB_LINK) dbScanFwdLink (&(pRec->lk09));
  if (pRec->lk10.type == DB_LINK) dbScanFwdLink (&(pRec->lk10));
  if (pRec->lk11.type == DB_LINK) dbScanFwdLink (&(pRec->lk11));
  if (pRec->lk12.type == DB_LINK) dbScanFwdLink (&(pRec->lk12));
  if (pRec->lk13.type == DB_LINK) dbScanFwdLink (&(pRec->lk13));
  if (pRec->lk14.type == DB_LINK) dbScanFwdLink (&(pRec->lk14));
  if (pRec->lk15.type == DB_LINK) dbScanFwdLink (&(pRec->lk15));
  if (pRec->lk16.type == DB_LINK) dbScanFwdLink (&(pRec->lk16));
  if (pRec->lk17.type == DB_LINK) dbScanFwdLink (&(pRec->lk17));
  if (pRec->lk18.type == DB_LINK) dbScanFwdLink (&(pRec->lk18));
  if (pRec->lk19.type == DB_LINK) dbScanFwdLink (&(pRec->lk19));
  if (pRec->lk20.type == DB_LINK) dbScanFwdLink (&(pRec->lk20));
  if (pRec->lk21.type == DB_LINK) dbScanFwdLink (&(pRec->lk21));
  if (pRec->lk22.type == DB_LINK) dbScanFwdLink (&(pRec->lk22));
  if (pRec->lk23.type == DB_LINK) dbScanFwdLink (&(pRec->lk23));
  if (pRec->lk24.type == DB_LINK) dbScanFwdLink (&(pRec->lk24));
  if (pRec->lk25.type == DB_LINK) dbScanFwdLink (&(pRec->lk25));
  if (pRec->lk26.type == DB_LINK) dbScanFwdLink (&(pRec->lk26));
  if (pRec->lk27.type == DB_LINK) dbScanFwdLink (&(pRec->lk27));
  if (pRec->lk28.type == DB_LINK) dbScanFwdLink (&(pRec->lk28));
  if (pRec->lk29.type == DB_LINK) dbScanFwdLink (&(pRec->lk29));
  if (pRec->lk30.type == DB_LINK) dbScanFwdLink (&(pRec->lk30));
  if (pRec->lk31.type == DB_LINK) dbScanFwdLink (&(pRec->lk31));
  pRec->lval = pRec->val;
}


static long get_precision (struct dbAddr *paddr, long *pprecision) {
  *pprecision = 1;
  return 0;
}
