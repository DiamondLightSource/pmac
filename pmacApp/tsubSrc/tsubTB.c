/* @(#) tsubTBSt.c 2.0 2004/05/31 -- speed propagation */

/* tsubTBSt.c - Transformation Subroutines for a table supported by 3 motors
 *            - can also be used for the GM/CA CAT KB mirror  */

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

volatile int tsubTBDebug = 0;
#define TSUB_MESSAGE	logMsg
#define TSUB_TRACE(level,code)       { if ( (pRec->tpro == (level)) || (tsubTBDebug == (level)) ) { code } }

/* if x1=mm, x2,x3=mrad, base=m (BioCAT), then FCTB=1; if base=mm, then FCTB=1000. */
volatile double FCTB = 1.0;

/* ===========================================
 * tsubTBSt - Table support
 */
static long tsubTBSt (struct tsubRecord *pRec) {
        return (0);
}


/* ===========================================
 * tsubTBStSync
 *	oa = m1:RqsPos
 *	ob = m2:RqsPos
 *	oc = m2:RqsPos
 *	a  = m1:ActPos
 *	b  = m2:ActPos
 *	c  = m3:ActPos
 */
static long tsubTBStSync (struct tsubRecord *pRec) {
	pRec->oa = pRec->a;
	pRec->ob = pRec->b;
	pRec->oc = pRec->c;
	return (0);
}


/* ===========================================
 * tsubTBStMtr - Table support Motors
 *	oa = x1
 *	ob = x2
 *	oc = x3
 *	oa1 = d1
 *	ob1 = d2
 *	oc1 = d3
 *	a = m1
 *	b = m2
 *	c = m3
 *	m = Base1:23
 *	n = Base23
 *	a0 = d1:Offset
 *	a1 = d1:Scale
 *	b0 = d2:Offset
 *	b1 = d2:Scale
 *	c0 = d3:Offset
 *	c1 = d3:Scale
 *	nla = (0=w/Offsets)[abs,pos] (1=wo/Offsets)[rel,vel]
 */
static long tsubTBStMtr (struct tsubRecord *pRec) {
	if ( (pRec->m == 0.0) || (pRec->n == 0.0) )
	{
		return (-1);
	}
	if (pRec->nla == 0.0)
	{
		pRec->oa1 = pRec->a * pRec->a1 + pRec->a0;              /* d1=m1*scale1+offset1 */
		pRec->ob1 = pRec->b * pRec->b1 + pRec->b0;              /* d2=m2*scale2+offset2 */
		pRec->oc1 = pRec->c * pRec->c1 + pRec->c0;              /* d3=m3*scale3+offset3 */
	}
	else
	{
		pRec->oa1 = pRec->a * pRec->a1;                         /* d1=m1*scale1 */
		pRec->ob1 = pRec->b * pRec->b1;                         /* d2=m2*scale2 */
		pRec->oc1 = pRec->c * pRec->c1;                         /* d3=m3*scale3 */
	}

	pRec->oa  = (pRec->ob1 + pRec->oc1 + 2.0 * pRec->oa1) / 4.0;    /* x1=(d2+d3+2*d1)/4            */
	pRec->ob  = FCTB * (pRec->ob1 + pRec->oc1 - 2.0 * pRec->oa1)  /* x2=(d2+d3-2*d1)/(2*Base1:23) */
                  / (2.0 * pRec->m);
	pRec->oc  = FCTB * (pRec->ob1 - pRec->oc1) / pRec->n;         /* x3=(d2-d3)/Base23            */
	return (0);
}



/* ===========================================
 * tsubTBStDrv - Table support Drives
 *	oa = x1
 *	ob = x2
 *	oc = x3
 *	oa0 = m1
 *	ob0 = m2
 *	oc0 = m3
 *	a = d1
 *	b = d2
 *	c = d3
 *	m = Base1:23
 *	n = Base23
 *	a0 = d1:Offset
 *	a1 = d1:Scale
 *	b0 = d2:Offset
 *	b1 = d2:Scale
 *	c0 = d3:Offset
 *	c1 = d3:Scale
 *	nla = (0=w/Offsets)[abs,pos] (1=wo/Offsets)[rel,vel]
 */
static long tsubTBStDrv (struct tsubRecord *pRec) {
	if ( (pRec->m  == 0.0) || (pRec->n == 0.0) ||
	     (pRec->a1 == 0.0) ||
	     (pRec->b1 == 0.0) ||
	     (pRec->c1 == 0.0) )
	{
		return (-1);
	}
	if (pRec->nla == 0.0)
	{
		pRec->oa0 = (pRec->a - pRec->a0) / pRec->a1;            /* m1=(d1-offset1)/scale1 */
		pRec->ob0 = (pRec->b - pRec->b0) / pRec->b1;            /* m2=(d2-offset2)/scale2 */
		pRec->oc0 = (pRec->c - pRec->c0) / pRec->c1;            /* m3=(d3-offset3)/scale3 */
	}
	else
	{
		pRec->oa0 = (pRec->a) / pRec->a1;                       /* m1=d1/scale1 */
		pRec->ob0 = (pRec->b) / pRec->b1;                       /* m2=d2/scale2 */
		pRec->oc0 = (pRec->c) / pRec->c1;                       /* m3=d3/scale3 */
	}
	pRec->oa  = (pRec->b + pRec->c + 2.0 * pRec->a) / 4.0;          /* x1=(d2+d3+2*d1)/4            */
	pRec->ob  = FCTB * (pRec->b + pRec->c - 2.0 * pRec->a)        /* x2=(d2+d3-2*d1)/(2*Base1:23) */
                  / (2.0 * pRec->m);
	pRec->oc  = FCTB * (pRec->b - pRec->c) / pRec->n;             /* x3=(d2-d3)/Base23            */
	return (0);
}

/* ===========================================
 * tsubTBStAxs - Table support Axes
 *	oa0 = m1
 *	oa1 = d1
 *	ob0 = m2
 *	ob1 = d2
 *	oc0 = m3
 *	oc1 = d3
 *	a = x1
 *	b = x2
 *	c = x3
 *	m = Base1:23
 *	n = Base23
 *	a0 = d1:Offset
 *	a1 = d1:Scale
 *	b0 = d2:Offset
 *	b1 = d2:Scale
 *	c0 = d3:Offset
 *	c1 = d3:Scale
 *	nla = (0=w/Offsets)[abs,pos] (1=wo/Offsets)[rel,vel]
 */
static long tsubTBStAxs (struct tsubRecord *pRec) {
	if ( (pRec->m  == 0.0) ||
             (pRec->n  == 0.0) ||
	     (pRec->a1 == 0.0) ||
	     (pRec->b1 == 0.0) ||
	     (pRec->c1 == 0.0) )
	{
		return (-1);
	}
	pRec->oa1 = pRec->a - (pRec->b * pRec->m) / (2.0 * FCTB);                     /* d1=x1-(x2*Base1:23)/2            */
	pRec->ob1 = pRec->a + (pRec->b * pRec->m + pRec->c * pRec->n) / (2.0 * FCTB); /* d2=x1+(x2*Base1:23+x3*Bas23)/2   */
	pRec->oc1 = pRec->a + (pRec->b * pRec->m - pRec->c * pRec->n) / (2.0 * FCTB); /* d3=x1+(x2*Base1:23-x3*Bas23)/2   */
	if (pRec->nla == 0.0)
	{
		pRec->oa0 = (pRec->oa1 - pRec->a0) / pRec->a1;     /* m1=(d1-offset1)/scale1 */
		pRec->ob0 = (pRec->ob1 - pRec->b0) / pRec->b1;     /* m2=(d2-offset2)/scale2 */
		pRec->oc0 = (pRec->oc1 - pRec->c0) / pRec->c1;     /* m3=(d3-offset3)/scale3 */
	}
	else
	{
		pRec->oa0 = (pRec->oa1) / pRec->a1;                /* m1=d1/scale1 */
		pRec->ob0 = (pRec->ob1) / pRec->b1;                /* m2=d2/scale2 */
		pRec->oc0 = (pRec->oc1) / pRec->c1;                /* m3=d3/scale3 */
	}
	return (0);
}

/* ===========================================
 * tsubTBStSpeed - Speed propagation spreadsheet
 *	oa0 = m1
 *	ob0 = m2
 *	oc0 = m3
 *	oa1 = d1
 *	ob1 = d2
 *	oc1 = d3
 *	oa2 = x1
 *	ob2 = x2
 *	oc2 = x3
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
 *	a2 = x1
 *	b2 = x2
 *	c2 = x3
 *	a3 = d1:Scale
 *	b3 = d2:Scale
 *	c3 = d3:Scale
 *	m = Base1:23
 *	n = Base23
 *      nla = Index of input(m1=1, m2=2, m3=3, d1=11, d2=12, d3=13, x1=21, x2=22, x3=23)
 */
static long tsubTBStSpeed (struct tsubRecord *pRec) {
	double prcn = 0.0;
	double m1, m2, m3, d1, d2, d3, x1, x2, x3;
	long ifail = 0;
/* Disable next record processing in order to avoid infinite loop.
 * This actually points to a record linked to the SDIS field of the tsub.
 * The SDIS has to be re-enabled before next tsub record call */
   	pRec->oj = 1;                                    /* sdis=1 */

  	if (tsubTBDebug > 1) printf ("tsubTBStSpeed: called with n=%f \n",pRec->nla);

	if ( pRec->a  == 0.0 || pRec->b  == 0.0 || pRec->c  == 0.0 ||
             pRec->a3 == 0.0 || pRec->b3 == 0.0 || pRec->c3 == 0.0 ||
             pRec->m  == 0.0 || pRec->n  == 0.0) {
	   printf ("tsubTBStSpeed: exit on zero calc parameters\n");
	   printf ("tsubTBStSpeed: max1=%5.2f sca1=%g\n",    pRec->a, pRec->a3);
	   printf ("tsubTBStSpeed: max2=%5.2f sca2=%g\n",    pRec->b, pRec->b3);
	   printf ("tsubTBStSpeed: max3=%5.2f sca3=%g\n",    pRec->c, pRec->c3);
	   printf ("tsubTBStSpeed: Base1:23=%g Base23=%g\n", pRec->m, pRec->n);
           return (-1);
	}

	if      (pRec->nla ==  1.0)  /* ------------------------- m1 speed changed */
	{
	   if ( pRec->a0 == 0.0 ) prcn = 1.0;              /* if 0, then set to max */
	   else prcn = fabs(pRec->a0 / pRec->a);           /* prcn=m1/m1_max */
	}
	else if (pRec->nla ==  2.0)  /* ------------------------- m2 speed changed */
	{
	   if ( pRec->b0 == 0.0 ) prcn = 1.0;              /* if 0, then set to max */
	   else prcn = fabs(pRec->b0 / pRec->b);           /* prcn=m2/m2_max */
	}
	else if (pRec->nla ==  3.0)  /* ------------------------- m3 speed changed */
	{
	   if ( pRec->c0 == 0.0 ) prcn = 1.0;              /* if 0, then set to max */
	   else prcn = fabs(pRec->c0 / pRec->c);           /* prcn=m3/m3_max */
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
	else if (pRec->nla == 21.0) /* ------------------------- x1 speed changed */
	{
	   if ( pRec->a2  < 0.0 ) ifail = -1;              /* x,d speeds are always > 0 */
	   if ( pRec->a2 <= 0.0 ) prcn = 1.0;              /* if 0, then set to max */
	   else {
/* Speed of center is the average of the speeds of all drives */
	      d1 = fabs( 1000.0 *  pRec->a * pRec->a3 );   /* d1=m1*scale1 */
	      d2 = fabs( 1000.0 *  pRec->b * pRec->b3 );   /* d2=m2*scale2 */
	      d3 = fabs( 1000.0 *  pRec->c * pRec->c3 );   /* d3=m3*scale3 */
              prcn = fabs( pRec->a2 / ((d2 + d3 + 2.0*d1) / 4.0) );
	   }
	}
	else if (pRec->nla == 22.0) /* ------------------------- x2 speed changed */
	{
	   if ( pRec->b2  < 0.0 ) ifail = -1;              /* x,d speeds are always > 0 */
	   if ( pRec->b2 <= 0.0 ) prcn = 1.0;              /* if 0, then set to max */
	   else {
/* Speed of angle is twice the speed of each motor divided by base */
	      d1 = fabs( 1000.0 *  pRec->a * pRec->a3 );   /* d1=m1*scale1 */
	      d2 = fabs( 1000.0 *  pRec->b * pRec->b3 );   /* d2=m2*scale2 */
	      d3 = fabs( 1000.0 *  pRec->c * pRec->c3 );   /* d3=m3*scale3 */
              prcn = fabs( pRec->b2 / (FCTB*(d2 + d3 + 2.0*d1) / (2.0*pRec->m)) );
	   }
	}
	else if (pRec->nla == 23.0) /* ------------------------- x3 speed changed */
	{
	   if ( pRec->c2  < 0.0 ) ifail = -1;              /* x,d speeds are always > 0 */
	   if ( pRec->c2 <= 0.0 ) prcn = 1.0;              /* if 0, then set to max */
	   else {
/* Speed of angle is twice the speed of each motor divided by base */
	      d2 = fabs( 1000.0 *  pRec->b * pRec->b3 );   /* d2=m2*scale2 */
	      d3 = fabs( 1000.0 *  pRec->c * pRec->c3 );   /* d3=m3*scale3 */
              prcn = fabs( pRec->c2 / (FCTB*(d2 + d3) / (2.0*pRec->n)) );
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
	m1 = prcn * pRec->a;                                  /* m1=prcn*m1_max */
	m2 = prcn * pRec->b;                                  /* m2=prcn*m2_max */
	m3 = prcn * pRec->c;                                  /* m3=prcn*m3_max */
	d1 = fabs( 1000.0 *  m1 * pRec->a3 );                 /* d1=m1*scale1 */
	d2 = fabs( 1000.0 *  m2 * pRec->b3 );                 /* d2=m2*scale2 */
	d3 = fabs( 1000.0 *  m3 * pRec->c3 );                 /* d3=m3*scale3 */
	x1 = fabs( (d2 + d3 + 2.0*d1) / 4.0 );                /* x1=(d2+d3+2*d1)/4 */
	x2 = fabs( FCTB*(d2 + d3 + 2.0*d1) / (2.0*pRec->m) ); /* x2=(d2+d3-2*d1)/(2*Base1:23) */
	x3 = fabs( FCTB*(d2 + d3)          / (2.0*pRec->n) ); /* x2=(d2+d3)     /(2*Base23) */
	pRec->oa0 = m1;
	pRec->ob0 = m2;
	pRec->oc0 = m3;
	pRec->oa1 = d1;
	pRec->ob1 = d2;
	pRec->oc1 = d3;
	pRec->oa2 = x1;
	pRec->ob2 = x2;
	pRec->oc2 = x3;
  	if (tsubTBDebug > 1) printf ("tsubTBStSpeed: m1=%5.2f  m2=%5.2f  m3=%5.2f\n",m1,m2,m3);
	return (ifail);
}

/* ===========================================
 *               Names registration
 *  =========================================== */
static registryFunctionRef tsubTBStRef[] = {
    {"tsubTBSt",      (REGISTRYFUNCTION)tsubTBSt},
    {"tsubTBStSync",  (REGISTRYFUNCTION)tsubTBStSync},
    {"tsubTBStMtr",   (REGISTRYFUNCTION)tsubTBStMtr},
    {"tsubTBStDrv",   (REGISTRYFUNCTION)tsubTBStDrv},
    {"tsubTBStAxs",   (REGISTRYFUNCTION)tsubTBStAxs},
    {"tsubTBStSpeed", (REGISTRYFUNCTION)tsubTBStSpeed}
};

static void tsubTBStFunc(void) {				/* declare this via registrar in DBD */
    registryFunctionRefAdd(tsubTBStRef,NELEMENTS(tsubTBStRef));
}
epicsExportRegistrar(tsubTBStFunc);

