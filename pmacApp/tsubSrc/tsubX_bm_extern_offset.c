/* @(#) tsubXbm.c 1.0 2006/05/20 -- speed propagation */

/* tsubXbm.c - Transformation Subroutines for X-positioners          */
/*           or one-axis rotation stages, i.e. any modular drives  */
/*           This is for 1-motor assemblies -- Stepanov            */
/* Chitra:   Modified tsubX.c 2.2 to include offset from upstream mirror    */

#include	<stdlib.h>
#include	<stdio.h>
#include	<string.h>
#include	<math.h>

#include	<dbDefs.h>
#include	<tsubRecord.h>
#include	<dbCommon.h>
#include	<recSup.h>
#include	<epicsExport.h>		/* Sergey */
#include	<registryFunction.h>	/* Sergey */

volatile int tsubXbmDebug = 0;
#define TSUB_MESSAGE	logMsg
#define TSUB_TRACE(level,code) { if ( (pRec->tpro == (level)) || (tsubXbmDebug == (level)) ) { code } }

/* Distances are measured in mm and mirror angles are in mrad */

/* ===========================================
 * tsubXbmPs - Assembly-X Initialization
 */
static long tsubXbmPs (struct tsubRecord *pRec) {
	return (0);
}


/* ===========================================
 * tsubXbmPsSync
 *	oa = m1:RqsPos
 *	a  = m1:ActPos
 */
static long tsubXbmPsSync (struct tsubRecord *pRec) {
	pRec->oa = pRec->a;
	return (0);
}


/* ===========================================
 * tsubXbmPsMtr - Assembly-X Motors
 *	oa1 = d1
 *	a = m1
 *	oh  = S 	(vertical beam shift at sample due to mirror angle)
 *	a0 = d1:Offset
 *	a1 = d1:Scale
 *	d  = Dms
 *	m  = angleVCM
 *	n  = (0=don't use mirror angle) (1=use mirror angle)
 *	nla = (1=wo/Offsets)[rel,vel] (0=w/Offsets)[abs,pos]
 */
static long tsubXbmPsMtr (struct tsubRecord *pRec) {
	double TwoAlpha=0.0;
	if ( pRec->n != 0.0 ) TwoAlpha = 2.0*(pRec->m)/1000.0;                /* recalc. from mrad */
        else                  TwoAlpha = 0.0;
	pRec->oh = pRec->d * tan(TwoAlpha);                    		      /* S=Dms*tan(2*angleVCM) */
        if (tsubXbmDebug > 2) printf("+++tsub=%s: S=Dms*tan(angleVCM)=%g\n",pRec->name,pRec->oh);

	if (pRec->nla == 0.0)
	{
		pRec->oa1 = pRec->a * pRec->a1 + pRec->a0 + pRec->oh;         /* d1=m1*scale1+offset1+S */
	}
	else
	{
		pRec->oa1 = pRec->a * pRec->a1;                    /* d1=m1*scale1 */
	}
	return (0);
}

/* ===========================================
 * tsubXbmPsDrv - Assembly-X Drives
 *	oa0 = m1
 *	a = d1
 *	oh  = S 	(vertical beam shift at sample due to mirror angle)
 *	a0 = d1:Offset
 *	a1 = d1:Scale
 *	d  = Dms
 *	m  = angleVCM
 *	n  = (0=don't use mirror angle) (1=use mirror angle)
 *	nla = (1=wo/Offsets)[rel,vel] (0=w/Offsets)[abs,pos]
 */
static long tsubXbmPsDrv (struct tsubRecord *pRec) {
	double TwoAlpha=0.0;

	if ( pRec->a1 == 0.0 )
	{
		return (-1);
	}


	if ( pRec->n != 0.0 ) TwoAlpha = 2.0*(pRec->m)/1000.0;                /* recalc. from mrad */
        else                  TwoAlpha = 0.0;
	pRec->oh = pRec->d * tan(TwoAlpha);                    		      /* S=Dms*tan(2*angleVCM) */
        if (tsubXbmDebug > 2) printf("+++tsub=%s: S=Dms*tan(angleVCM)=%g\n",pRec->name,pRec->oh);

	if (pRec->nla == 0.0)
	{
		pRec->oa0 = (pRec->a - pRec->a0 - pRec->oh) / pRec->a1;       /* m1=(d1-offset1-S)/scale1 */
	}
	else
	{
		pRec->oa0 = pRec->a / pRec->a1;                    	      /* m1=d1/scale1 */
	}
	return (0);
}

/* ===========================================
 * tsubXbmPsSpeed - Speed propagation spreadsheet
 *	oa0 = m1
 *	oa1 = d1
 *      oj  = sdis (sdis=1 -- disable record processing)
 *	a = m1_max
 *	a0 = m1
 *	a1 = d1
 *	a3 = d1:Scale
 *	n  = (0=don't use mirror angle) (1=use mirror angle)
 *      nla = Index of input(m1=1, d1=11)
 */
static long tsubXbmPsSpeed (struct tsubRecord *pRec) {
	double prcn = 0.0;
	double m1, d1;

	long ifail = 0;
/* Disable next record processing in order to avoid infinite loop.
 * This actually points to a record linked to the SDIS field of the tsub.
 * The SDIS has to be re-enabled before next tsub record call */
   	pRec->oj = 1;                                    /* sdis=1 */

  	if (tsubXbmDebug > 1) printf ("tsubXbmPsSpeed: called with n=%f \n",pRec->nla);

	if ( pRec->a  == 0.0 || pRec->a3 == 0.0 ) {
	   printf ("tsubXbmPsSpeed: exit on zero calc parameters\n");
	   printf ("tsubXbmPsSpeed: max1=%5.2f sca1=%g\n",pRec->a,pRec->a3);
           return (-1);
	}

	if      (pRec->nla ==  1.0)  /* ------------------------- m1 speed changed */
	{
	   if ( pRec->a0 == 0.0 ) prcn = 1.0;              /* if 0, then set to max */
	   else prcn = fabs( pRec->a0 / pRec->a );         /* prcn=m1/m1_max */
	}
	else if (pRec->nla == 11.0) /* ------------------------- d1 speed changed */
	{
	   if ( pRec->a1  < 0.0 ) ifail = -1;              /* x,d speeds are always > 0 */
	   if ( pRec->a1 <= 0.0 ) prcn = 1.0;              /* if 0, then set to max */
/* m speed must have same sign as m_max */
	   else prcn = fabs( pRec->a1 / (1000.0 * pRec->a * pRec->a3) );
	}
	else
	{
	   return (-1);
	}

	if ( prcn < 0.0 || prcn > 1.0 ) {
/* If **anything** is wrong set speed to normal */
          ifail = -1;
	  prcn = 1.0;
	}
	m1 = prcn * pRec->a;                             /* m1=prcn*m1_max */
	d1 = fabs( 1000.0 *  m1 * pRec->a3 );            /* d1=m1*scale1 */
	pRec->oa0 = m1;
	pRec->oa1 = d1;
  	if (tsubXbmDebug > 1) printf ("tsubXbmPsSpeed: m1=%5.2f\n",m1);
	return (ifail);
}

/* ===========================================
 *               Names registration
 *  =========================================== */
static registryFunctionRef tsubXbmPsRef[] = {
    {"tsubXbmPs",      (REGISTRYFUNCTION)tsubXbmPs},
    {"tsubXbmPsSync",  (REGISTRYFUNCTION)tsubXbmPsSync},
    {"tsubXbmPsMtr",   (REGISTRYFUNCTION)tsubXbmPsMtr},
    {"tsubXbmPsDrv",   (REGISTRYFUNCTION)tsubXbmPsDrv},
    {"tsubXbmPsSpeed", (REGISTRYFUNCTION)tsubXbmPsSpeed}
};

static void tsubXbmPsFunc(void) {				/* declare this via registrar in DBD */
    registryFunctionRefAdd(tsubXbmPsRef,NELEMENTS(tsubXbmPsRef));
}
epicsExportRegistrar(tsubXbmPsFunc);

