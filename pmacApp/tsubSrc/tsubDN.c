/* @(#) tsubDN.c 1.4 97/04/03 */

/* tsubDN.c - Transformation Subroutines For Downstream Support */

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

#define TSUB_DIAGNOSTIC tsubDNDebug

volatile int tsubDNDebug = 0;
#define TSUB_MESSAGE	logMsg
#define TSUB_TRACE(level,code) { if ( (pRec->tpro == (level)) || (tsubDNDebug == (level)) ) { code } }


/* if x1=mm, x2=mrad, base=m (BioCAT), then FCDN=1; if base=mm, then FCDN=1000. */
double FCDN = 1.0;

/* ===========================================
 * tsubDNSt - Support Initialization
 */
static long tsubDNSt (struct tsubRecord *pRec) {
	return (0);
}


/* ===========================================
 * tsubDNStSync
 *	oa = m1:RqsPos
 *	ob = m2:RqsPos
 *	a  = m1:ActPos
 *	b  = m2:ActPos
 */
static long tsubDNStSync (struct tsubRecord *pRec) {
	pRec->oa = pRec->a;
	pRec->ob = pRec->b;
	return (0);
}


/* ===========================================
 * tsubDNStMtr - Support Motors
 *	oa = x1
 *	ob = x2
 *	oa1 = d1
 *	ob1 = d2
 *	a = m1
 *	b = m2
 *	k = BaseLength
 *	l = RotOrigin
 *	a0 = d1:Offset
 *	a1 = d1:Scale
 *	b0 = d2:Offset
 *	b1 = d2:Scale
 *	nla = (1=wo/Offsets)[rel,vel] (0=w/Offsets)[abs,pos]
 */
static long tsubDNStMtr (struct tsubRecord *pRec) {
	if ( (pRec->k == 0.0) )
	{
		return (-1);
	}
	if (pRec->nla == 0.0)
	{
		pRec->oa1 = pRec->a * pRec->a1 + pRec->a0;         /* d1=m1*scale1+offset1 */
		pRec->ob1 = pRec->b * pRec->b1 + pRec->b0;         /* d2=m2*scale2+offset2 */
	}
	else
	{
		pRec->oa1 = pRec->a * pRec->a1;                    /* d1=m1*scale1 */
		pRec->ob1 = pRec->b * pRec->b1;                    /* d2=m2*scale2 */
	}
	pRec->ob = FCDN * (pRec->ob1 - pRec->oa1) / pRec->k;       /* x2=(d2-d1)/BaseLength */
	pRec->oa = pRec->oa1 + (pRec->ob * pRec->l / FCDN);        /* x1=d1+x2*RotOrigin */
	return (0);				   /* another way is: x1=d1*(1-R/B)+d2*(R/B) */
}

/* ===========================================
 * tsubDNStDrv - Support Drives
 *	oa = x1
 *	ob = x2
 *	oa0 = m1
 *	ob0 = m2
 *	a = d1
 *	b = d2
 *	k = BaseLength
 *	l = RotOrigin
 *	a0 = d1:Offset
 *	a1 = d1:Scale
 *	b0 = d2:Offset
 *	b1 = d2:Scale
 *	nla = (1=wo/Offsets)[rel,vel] (0=w/Offsets)[abs,pos]
 */
static long tsubDNStDrv (struct tsubRecord *pRec) {
	if ( (pRec->k == 0.0) ||
	     (pRec->a1 == 0.0) ||
	     (pRec->b1 == 0.0) )
	{
		return (-1);
	}
	if (pRec->nla == 0.0)
	{
		pRec->oa0 = (pRec->a - pRec->a0) / pRec->a1;       /* m1=(d1-offset1)/scale1 */
		pRec->ob0 = (pRec->b - pRec->b0) / pRec->b1;       /* m2=(d2-offset2)/scale2 */
	}
	else
	{
		pRec->oa0 = (pRec->a) / pRec->a1;                  /* m1=d1/scale1 */
		pRec->ob0 = (pRec->b) / pRec->b1;                  /* m2=d2/scale2 */
	}
	pRec->ob = FCDN * (pRec->b - pRec->a) / pRec->k;           /* x2=(d2-d1)/BaseLength */
	pRec->oa = pRec->a + (pRec->ob * pRec->l / FCDN);          /* x1=d1+x2*RotOrigin    */
	return (0); 				   /* another way is: x1=d1*(1-R/B)+d2*(R/B) */

}

/* ===========================================
 * tsubDNStAxs - Support Axes
 *	oa0 = m1
 *	oa1 = d1
 *	ob0 = m2
 *	ob1 = d2
 *	a = x1
 *	b = x2
 *	k = BaseLength
 *	l = RotOrigin
 *	a0 = d1:Offset
 *	a1 = d1:Scale
 *	b0 = d2:Offset
 *	b1 = d2:Scale
 *	nla = (1=wo/Offsets)[rel,vel] (0=w/Offsets)[abs,pos]
 */
static long tsubDNStAxs (struct tsubRecord *pRec) {
	if ( (pRec->a1 == 0.0) || (pRec->b1 == 0.0) )
	{
		return (-1);
	}
	pRec->oa1 = pRec->a - (pRec->b * pRec->l / FCDN);          /* d1=x1-x2*RotOrigin     */
	pRec->ob1 = pRec->b * (pRec->k / FCDN) + pRec->oa1;        /* d2=x2*BaseLength+d1    */
	if (pRec->nla == 0.0)
	{
		pRec->oa0 = (pRec->oa1 - pRec->a0) / pRec->a1;     /* m1=(d1-offset1)/scale1 */
		pRec->ob0 = (pRec->ob1 - pRec->b0) / pRec->b1;     /* m2=(d2-offset2)/scale2 */
	}
	else
	{
		pRec->oa0 = (pRec->oa1) / pRec->a1;                /* m1=d1/scale1 */
		pRec->ob0 = (pRec->ob1) / pRec->b1;                /* m2=d2/scale2 */
	}
	return (0);
}


/* ===========================================
 * tsubDNStSpeed - Speed propagation spreadsheet
 *	oa0 = m1
 *	ob0 = m2
 *	oa1 = d1
 *	ob1 = d2
 *	oa2 = x1
 *	ob2 = x2
 *      oj  = sdis (sdis=1 -- disable record processing)
 *	a = m1_max
 *	b = m2_max
 *	a0 = m1
 *	b0 = m2
 *	a1 = d1
 *	b1 = d2
 *	a2 = x1
 *	b2 = x2
 *	a3 = d1:Scale
 *	b3 = d2:Scale
 *	k = BaseLength
 *	l = RotOrigin
 *      nla = Index of input(m1=1, m2=2, d1=11, d2=12, x1=21, x2=22)
 */
static long tsubDNStSpeed (struct tsubRecord *pRec) {
	double prcn = 0.0;
	double m1, m2, d1, d2, x1, x2, r;
	long ifail = 0;
/* Disable next record processing in order to avoid infinite loop.
 * This actually points to a record linked to the SDIS field of the tsub.
 * The SDIS has to be re-enabled before next tsub record call */
   	pRec->oj = 1;                                    /* sdis=1 */

  	if (tsubDNDebug > 1) printf ("tsubDNStSpeed: called with n=%f \n",pRec->nla);

	if ( pRec->a  == 0.0 || pRec->b  == 0.0 ||
             pRec->a3 == 0.0 || pRec->b3 == 0.0 ||
             pRec->k  == 0.0) {
	   printf ("tsubDNStSpeed: exit on zero calc parameters\n");
	   printf ("tsubDNStSpeed: max1=%5.2f sca1=%g\n",pRec->a,pRec->a3);
	   printf ("tsubDNStSpeed: max2=%5.2f sca2=%g\n",pRec->b,pRec->b3);
	   printf ("tsubDNStSpeed: BaseLength=%g\n",pRec->k);
           return (-1);
	}
	r = pRec->l/pRec->k;				   /* r=RotOrigin/BaseLength */

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
	else if (pRec->nla == 21.0) /* ------------------------- x1 speed changed */
	{
	   if ( pRec->a2  < 0.0 ) ifail = -1;              /* x,d speeds are always > 0 */
	   if ( pRec->a2 <= 0.0 ) prcn = 1.0;              /* if 0, then set to max */
	   else {
/* Speed of center is the average of the speeds of individual drives */
	      d1 = fabs( 1000.0 *  pRec->a * pRec->a3 );   /* d1=m1*scale1 */
	      d2 = fabs( 1000.0 *  pRec->b * pRec->b3 );   /* d2=m2*scale2 */
	      x1 = d1 * ( 1.0 - r) + d2 * r;
	      if ( x1 != 0.0 ) prcn = fabs( pRec->a2 / x1 );
	      else             prcn = 1.0;
	   }
	}
	else if (pRec->nla == 22.0) /* ------------------------- x2 speed changed */
	{
	   if ( pRec->b2  < 0.0 ) ifail = -1;              /* x,d speeds are always > 0 */
	   if ( pRec->b2 <= 0.0 ) prcn = 1.0;              /* if 0, then set to max */
	   else {
/* Speed of angle is the sum of the speeds of each motor divided by base */
	      d1 = fabs( 1000.0 *  pRec->a * pRec->a3 );   /* d1=m1*scale1 */
	      d2 = fabs( 1000.0 *  pRec->b * pRec->b3 );   /* d2=m2*scale2 */
	      x2 = FCDN*(d1+d2) / pRec->k;
	      if ( x2 != 0.0 ) prcn = fabs( pRec->b2 / x2 );
	      else             prcn = 1.0;
	   }
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
	d1 = fabs( 1000.0 *  m1 * pRec->a3 );            /* d1=m1*scale1 */
	d2 = fabs( 1000.0 *  m2 * pRec->b3 );            /* d2=m2*scale2 */
	x1 = fabs( d1 * (1.0 - r) + d2 * r );            /* x1=d1*(1-r)+d2*r */
	x2 = fabs( FCDN*(d1 + d2) / pRec->k );           /* x2=(d2+d1)/BaseLength */
	pRec->oa0 = m1;
	pRec->ob0 = m2;
	pRec->oa1 = d1;
	pRec->ob1 = d2;
	pRec->oa2 = x1;
	pRec->ob2 = x2;
  	if (tsubDNDebug > 1) printf ("tsubDNStSpeed: m1=%5.2f  m2=%5.2f\n",m1,m2);
	return (ifail);
}

/* ===========================================
 *               Names registration
 *  =========================================== */
static registryFunctionRef tsubDNStRef[] = {
    {"tsubDNSt",      (REGISTRYFUNCTION)tsubDNSt},
    {"tsubDNStSync",  (REGISTRYFUNCTION)tsubDNStSync},
    {"tsubDNStMtr",   (REGISTRYFUNCTION)tsubDNStMtr},
    {"tsubDNStDrv",   (REGISTRYFUNCTION)tsubDNStDrv},
    {"tsubDNStAxs",   (REGISTRYFUNCTION)tsubDNStAxs},
    {"tsubDNStSpeed", (REGISTRYFUNCTION)tsubDNStSpeed}
};

static void tsubDNStFunc(void) {				/* declare this via registrar in DBD */
    registryFunctionRefAdd(tsubDNStRef,NELEMENTS(tsubDNStRef));
}
epicsExportRegistrar(tsubDNStFunc);

