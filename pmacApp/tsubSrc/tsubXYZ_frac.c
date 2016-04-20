/* @(#) tsubXYZfra.c 2.2 2004/10/11 -- speed propagation (fractional count) */

/*  tsubXYZfra.c - Transformation Subroutines for XYZ positioners   */
/*                  This is for 3-motors assemblies -- S. Stepanov  */

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

volatile int tsubXYZfraDebug = 0;
#define TSUB_MESSAGE	logMsg
#define TSUB_TRACE(level,code) { if ( (pRec->tpro == (level)) || (tsubXYZfraDebug == (level)) ) { code } }

/* ===========================================
 * tsubXYZfraPs - Assembly-XYZ Initialization
 */
static long tsubXYZfraPs (struct tsubRecord *pRec) {
	return (0);
}


/* ===========================================
 * tsubXYZfraPsSync
 *	oa = m1:RqsPos
 *	ob = m2:RqsPos
 *	oc = m2:RqsPos
 *	a  = m1:ActPos
 *	b  = m2:ActPos
 *	c  = m3:ActPos
 */
static long tsubXYZfraPsSync (struct tsubRecord *pRec) {
	pRec->oa = pRec->a;
	pRec->ob = pRec->b;
	pRec->oc = pRec->c;

	return (0);
}


/* ===========================================
 * tsubXYZfraPsMtr - Assembly-XYZ Motors
 *	oa1 = d1    oa2=m1_int   oa3=m1_fra
 *	ob1 = d2    ob2=m2_int   ob3=m2_fra
 *	oc1 = d3    oc2=m3_int   oc3=m3_fra
 *	a = m1
 *	b = m2
 *	c = m3
 *	a0 = d1:Offset
 *	a1 = d1:Scale
 *	b0 = d2:Offset
 *	b1 = d2:Scale
 *	c0 = d3:Offset
 *	c1 = d3:Scale
 *	nla = (1=wo/Offsets)[rel,vel] (0=w/Offsets)[abs,pos]
 */
static long tsubXYZfraPsMtr (struct tsubRecord *pRec) {
	if (pRec->nla == 0.0)
	{
		pRec->oa1 = pRec->a * pRec->a1 + pRec->a0;         /* d1=m1*scale1+offset1 */
		pRec->ob1 = pRec->b * pRec->b1 + pRec->b0;         /* d2=m2*scale2+offset2 */
		pRec->oc1 = pRec->c * pRec->c1 + pRec->c0;         /* d3=m3*scale3+offset3 */
                pRec->oa2 = (long)(pRec->a);
                pRec->ob2 = (long)(pRec->b);
                pRec->oc2 = (long)(pRec->c);
		pRec->oa3 = pRec->a - pRec->oa2;
		pRec->ob3 = pRec->b - pRec->ob2;
		pRec->oc3 = pRec->c - pRec->oc2;
	}
	else
	{
		pRec->oa1 = pRec->a * pRec->a1;                    /* d1=m1*scale1 */
		pRec->ob1 = pRec->b * pRec->b1;                    /* d2=m2*scale2 */
		pRec->oc1 = pRec->c * pRec->c1;                    /* d3=m3*scale3 */
	}
	return (0);
}

/* ===========================================
 * tsubXYZfraPsDrv - Assembly-XYZ Drives
 *	oa0 = m1   oa2 = m1_int   oa3 = m1_fra
 *	ob0 = m2   ob2 = m2_int   ob3 = m2_fra
 *	oc0 = m3   oc2 = m3_int   oc3 = m2_fra
 *	a = d1
 *	b = d2
 *	c = d3
 *	a0 = d1:Offset
 *	a1 = d1:Scale
 *	b0 = d2:Offset
 *	b1 = d2:Scale
 *	c0 = d3:Offset
 *	c1 = d3:Scale
 *	nla = (1=wo/Offsets)[rel,vel] (0=w/Offsets)[abs,pos]
 */
static long tsubXYZfraPsDrv (struct tsubRecord *pRec) {
	if ( (pRec->a1 == 0.0) ||
	     (pRec->b1 == 0.0) ||
	     (pRec->c1 == 0.0) )
	{
		return (-1);
	}
	if (pRec->nla == 0.0)
	{
		pRec->oa0 = (pRec->a - pRec->a0) / pRec->a1;       /* m1=(d1-offset1)/scale1 */
		pRec->ob0 = (pRec->b - pRec->b0) / pRec->b1;       /* m2=(d2-offset2)/scale2 */
		pRec->oc0 = (pRec->c - pRec->c0) / pRec->c1;       /* m3=(d3-offset3)/scale3 */
                pRec->oa2 = (long)(pRec->oa0);
                pRec->ob2 = (long)(pRec->ob0);
                pRec->oc2 = (long)(pRec->oc0);
		pRec->oa3 = pRec->oa0 - pRec->oa2;
		pRec->ob3 = pRec->ob0 - pRec->ob2;
		pRec->oc3 = pRec->oc0 - pRec->oc2;
	}
	else
	{
		pRec->oa0 = pRec->a / pRec->a1;                    /* m1=d1/scale1 */
		pRec->ob0 = pRec->b / pRec->b1;                    /* m2=d2/scale2 */
		pRec->oc0 = pRec->c / pRec->c1;                    /* m3=d3/scale3 */
	}
	return (0);
}

/* ===========================================
 * tsubXYZfraPsSpeed - Speed propagation spreadsheet
 *	oa0 = m1
 *	ob0 = m2
 *	oc0 = m3
 *	oa1 = d1
 *	ob1 = d2
 *	oc1 = d3
 *      oj  = sdis (sdis=1 -- disable record processing)
 *	a = m1_max
 *	b = m2_max
 *	c = m3_max
 *	a0 = m1
 *	b0 = m2
 *	c0 = m3
 *	a1 = d1
 *	b1 = d2
 *	c1 = d3
 *	a3 = d1:Scale
 *	b3 = d2:Scale
 *	c3 = d3:Scale
 *      nla = Index of input(m1=1, m2=2, d1=11, d2=12)
 */
static long tsubXYZfraPsSpeed (struct tsubRecord *pRec) {
	double prcn = 0.0;
	double m1, m2, m3, d1, d2, d3;
	long ifail = 0;
/* Disable next record processing in order to avoid infinite loop.
 * This actually points to a record linked to the SDIS field of the tsub.
 * The SDIS has to be re-enabled before next tsub record call */
   	pRec->oj = 1;                                    /* sdis=1 */

	if (tsubXYZfraDebug > 1) printf ("tsubXYZfraPsSpeed: called with n=%f \n",pRec->nla);

	if ( pRec->a  == 0.0 || pRec->a3 == 0.0 ||
             pRec->b  == 0.0 || pRec->b3 == 0.0 ||
             pRec->c  == 0.0 || pRec->c3 == 0.0 ) {
	   printf ("tsubXYZfraPsSpeed: exit on zero calc parameters\n");
	   printf ("tsubXYZfraPsSpeed: max1=%5.2f sca1=%g\n",pRec->a,pRec->a3);
	   printf ("tsubXYZfraPsSpeed: max2=%5.2f sca2=%g\n",pRec->b,pRec->b3);
	   printf ("tsubXYZfraPsSpeed: max3=%5.2f sca3=%g\n",pRec->c,pRec->c3);
           return (-1);
	}

	if      (pRec->nla ==  1.0)  /* ------------------------- m1 speed changed */
	{
	   if ( pRec->a0 == 0.0 ) prcn = 1.0;              /* if 0, then set to max */
	   else prcn = fabs( pRec->a0 / pRec->a );         /* prcn=m1/m1_max */
	}
	else if (pRec->nla ==  2.0)  /* ------------------------- m2 speed changed */
	{
	   if ( pRec->b0 == 0.0 ) prcn = 1.0;              /* if 0, then set to max */
	   else prcn = fabs( pRec->b0 / pRec->b );         /* prcn=m2/m2_max */
	}
	else if (pRec->nla ==  3.0)  /* ------------------------- m3 speed changed */
	{
	   if ( pRec->c0 == 0.0 ) prcn = 1.0;              /* if 0, then set to max */
	   else prcn = fabs( pRec->c0 / pRec->c );         /* prcn=m3/m3_max */
	}
	else if (pRec->nla == 11.0) /* ------------------------- d1 speed changed */
	{
	   if ( pRec->a1  < 0.0 ) ifail = -1;              /* x,d speeds are always > 0 */
	   if ( pRec->a1 <= 0.0 ) prcn = 1.0;              /* if 0, then set to max */
/* m speed must have same sign as m_max */
	   else prcn = fabs( pRec->a1 / (1000.0 * pRec->a * pRec->a3) );
	}
	else if (pRec->nla == 12.0) /* ------------------------- d2 speed changed */
	{
	   if ( pRec->b1  < 0.0 ) ifail = -1;              /* x,d speeds are always > 0 */
	   if ( pRec->b1 <= 0.0 ) prcn = 1.0;              /* if 0, then set to max */
/* m speed must have same sign as m_max */
	   else prcn = fabs( pRec->b1 / (1000.0 * pRec->b * pRec->b3) );
	}
	else if (pRec->nla == 13.0) /* ------------------------- d3 speed changed */
	{
	   if ( pRec->c1  < 0.0 ) ifail = -1;              /* x,d speeds are always > 0 */
	   if ( pRec->c1 <= 0.0 ) prcn = 1.0;              /* if 0, then set to max */
/* m speed must have same sign as m_max */
	   else prcn = fabs( pRec->c1 / (1000.0 * pRec->c * pRec->c3) );
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
	m2 = prcn * pRec->b;                             /* m2=prcn*m2_max */
	m3 = prcn * pRec->c;                             /* m3=prcn*m3_max */
	d1 = fabs( 1000.0 *  m1 * pRec->a3 );            /* d1=m1*scale1 */
	d2 = fabs( 1000.0 *  m2 * pRec->b3 );            /* d2=m2*scale2 */
	d3 = fabs( 1000.0 *  m3 * pRec->c3 );            /* d3=m3*scale3 */
	pRec->oa0 = m1;
	pRec->ob0 = m2;
	pRec->oc0 = m3;
	pRec->oa1 = d1;
	pRec->ob1 = d2;
	pRec->oc1 = d3;
        if (tsubXYZfraDebug > 1) printf ("tsubXYZfraPsSpeed: m1=%5.2f  m2=%5.2f  m3=%5.2f\n",m1,m2,m3);
	return (ifail);
}

/* ===========================================
 *               Names registration
 *  =========================================== */
static registryFunctionRef tsubXYZfraPsRef[] = {
    {"tsubXYZfraPs",      (REGISTRYFUNCTION)tsubXYZfraPs},
    {"tsubXYZfraPsSync",  (REGISTRYFUNCTION)tsubXYZfraPsSync},
    {"tsubXYZfraPsMtr",   (REGISTRYFUNCTION)tsubXYZfraPsMtr},
    {"tsubXYZfraPsDrv",   (REGISTRYFUNCTION)tsubXYZfraPsDrv},
    {"tsubXYZfraPsSpeed", (REGISTRYFUNCTION)tsubXYZfraPsSpeed}
};

static void tsubXYZfraPsFunc(void) {				/* declare this via registrar in DBD */
    registryFunctionRefAdd(tsubXYZfraPsRef,NELEMENTS(tsubXYZfraPsRef));
}
epicsExportRegistrar(tsubXYZfraPsFunc);

