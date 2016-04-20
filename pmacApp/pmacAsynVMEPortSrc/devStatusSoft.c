/* devStatusSoft.c - Device Support Routines for Soft Status Input */
/*
 *      Author:		Janet Anderson
 *      Date:   	09-23-91
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
 * .01  11-11-91        jba     Moved set of alarm stat and sevr to macros
 * .02	03-13-92	jba	ANSI C changes
 * .03  10-10-92        jba     replaced code with recGblGetLinkValue call
 * 2.1  2-27-04       oam     updated for epics 3.14.5
*/


/* #include	<vxWorks.h>
#include	<types.h>
#include	<stdioLib.h> */
#include	<string.h>

#include	<alarm.h>
#include	<dbDefs.h>
#include	<dbAccess.h>
#include        <recGbl.h>
#include        <recSup.h>
#include	<devSup.h>
/*#include	<module_types.h>*/

#include	<statusRecord.h>
#include "epicsExport.h"
/* Create the dset for devStatusSoft */
static long init_record();
static long read_status();

struct {
	long		number;
	DEVSUPFUN	report;
	DEVSUPFUN	init;
	DEVSUPFUN	init_record;
	DEVSUPFUN	get_ioint_info;
	DEVSUPFUN	read_status;
} devStatusSoft={
	5,
	NULL,
	NULL,
	init_record,
	NULL,
	read_status};
epicsExportAddress(dset,devStatusSoft);	

static long init_record( struct statusRecord *pstatus )
{
    if( pstatus->inp.type == CONSTANT )
    {
      if(recGblInitConstantLink(&pstatus->inp, DBF_LONG, &pstatus->val))
	pstatus->udf = FALSE;
    }
    return(0);
}


static long read_status( struct statusRecord *pstatus )
{
    long status;
    long options  = 0;
    long nRequest = 1;

    status = dbGetLink( &(pstatus->inp), DBR_LONG, &(pstatus->val),
                        &options, &nRequest );

    if(RTN_SUCCESS(status)) pstatus->udf=FALSE;

    return(status);
}
