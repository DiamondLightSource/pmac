/* @(#) tsubCS.c 2.2 2004/05/31 -- speed propagation */

/* tsubCS.c - Transformation Subroutines For Collimator Slits */
/* These equations are for "piggy-back" slits, i.e. blade-2   */
/* runs "on the back" of blade-1                              */
/*                ** ACCEL SLITS CONFIGURATION **             */

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

volatile int tsubCSDebug = 0;
#define TSUB_MESSAGE	logMsg
#define TSUB_TRACE(level,code) { if ( (pRec->tpro == (level)) || (tsubCSDebug == (level)) ) { code } }

/* ===========================================
 * tsubCSAp - Aperture Initialization
 */
static long tsubCSAp (struct tsubRecord *pRec) {
 	return (0);
}


/* ===========================================
 * tsubCSApSync
 *	oa = m1:RqsPos
 *	ob = m2:RqsPos
 *	a  = m1:ActPos
 *	b  = m2:ActPos
 */
static long tsubCSApSync (struct tsubRecord *pRec) {
	pRec->oa = pRec->a;
	pRec->ob = pRec->b;
	return (0);
}


/* ===========================================
 * tsubCSApMtr - Aperture Motors To Drives           |  ACCEL CSAv      |  ACCEL CSAh      |   xxx (biocat?)   |
 *	oa = x1  (x_center)                          | d1=top,d2=piggy  | d1=out,f2=piggy  | d1=bottom,d2=piggy|
 *	ob = x2  (x_size)                            |    d2 < 0        |    d2 < 0        |   d2 > 0          |
 *	oa1 = d1 (d_out or d_top)    -- independent  | center=d1+0.5*d2 | center=d1+0.5*d2 | center=d1+0.5*d2  |
 *	ob1 = d2 (d_in  or d_bottom) -- piggy-backed |   size=-d2       |   size=-d2       |   size=d2         |
 *	a = m1   (m_position)                        |    top=d1        |    out=d1        |    top=d1+d2      |
 *	b = m2   (m_size)                            | bottom=d1+d2     |     in=d1+d2     | bottom=d1         |
 *      i = Invert (0 -> +1   1 -> -1, implemented as 1-2i)
 *	a0 = d1:Offset
 *	a1 = d1:Scale
 *	b0 = d2:Offset
 *	b1 = d2:Scale
 *	nla = (1=wo/Offsets)[rel,vel] (0=w/Offsets)[abs,pos]
 */
static long tsubCSApMtr (struct tsubRecord *pRec) {
	double d2_real;

	if (pRec->nla == 0.0)
	{
		pRec->oa1 = pRec->a * pRec->a1 + pRec->a0;         /* d1 = m1*scale1+offset1 */
		d2_real   = pRec->b * pRec->b1 + pRec->b0;         /* d2'= m2*scale2+offset2 */
	}
	else
	{
		pRec->oa1 = pRec->a * pRec->a1;                    /* d1 = m1*scale1 */
		d2_real   = pRec->b * pRec->b1;                    /* d2'= m2*scale2 */
	}
	pRec->ob1 = pRec->oa1 + d2_real;                           /* d2 = d1+d2' */

	pRec->oa =       0.5         * (pRec->ob1 + pRec->oa1);    /* x1=0.5*(d2+d1) */
	pRec->ob = (1 - 2 * pRec->i) * (pRec->ob1 - pRec->oa1);    /* x2=inv*(d2-d1) */
	return (0);
}

/* ===========================================
 * tsubCSApDrv - Horizontal Aperture Drives          |  ACCEL CSAv      |  ACCEL CSAh      |   xxx (biocat?)   |
 *	oa = x1  (x_center)                          | d1=top,d2=piggy  | d1=out,f2=piggy  | d1=bottom,d2=piggy|
 *	ob = x2  (x_size)                            |    d2 < 0        |    d2 < 0        |   d2 > 0          |
 *	oa0 = m1 (m_position)                        | center=d1+0.5*d2 | center=d1+0.5*d2 | center=d1+0.5*d2  |
 *	ob0 = m2 (m_size)                            |   size=-d2       |   size=-d2       |   size=d2         |
 *	a = d1   (d_out or d_top)    -- independent  |    top=d1        |    out=d1        |    top=d1+d2      |
 *	b = d2   (d_in  or d_bottom) -- piggy-backed | bottom=d1+d2     |     in=d1+d2     | bottom=d1         |
 *      i = Invert (0 -> +1   1 -> -1, implemented as 1-2i)
 *	a0 = d1:Offset
 *	a1 = d1:Scale
 *	b0 = d2:Offset
 *	b1 = d2:Scale
 *	nla = (1=wo/Offsets)[rel,vel] (0=w/Offsets)[abs,pos]
 */
static long tsubCSApDrv (struct tsubRecord *pRec) {
	double d2_real;

	if ( (pRec->a1 == 0.0) ||
	     (pRec->b1 == 0.0) )
	{
		return (-1);
	}

	d2_real = (pRec->b - pRec->a);                             /* d2'= d2-d1 */
	if (pRec->nla == 0.0)
	{
		pRec->oa0 = (pRec->a - pRec->a0) / pRec->a1;       /* m1=(d1 -offset1)/scale1 */
		pRec->ob0 = (d2_real - pRec->b0) / pRec->b1;       /* m2=(d2'-offset2)/scale2 */
	}
	else
	{
		pRec->oa0 = (pRec->a) / pRec->a1;                  /* m1=d1 /scale1 */
		pRec->ob0 = (d2_real) / pRec->b1;                  /* m2=d2'/scale2 */
	}

	pRec->oa =       0.5         * (pRec->b + pRec->a);        /* x1=0.5*(d2+d1) */
	pRec->ob = (1 - 2 * pRec->i) * (pRec->b - pRec->a);        /* x2=inv*(d2-d1) */
 	return (0);
}

/* ===========================================
 * tsubCSApAxs - Horizontal Aperture Axes            |  ACCEL CSAv      |  ACCEL CSAh      |   xxx (biocat?)   |
 *	oa0 = m1 (m_position)                        | d1=top,d2=piggy  | d1=out,f2=piggy  | d1=bottom,d2=piggy|
 *	oa1 = d1 (d_out or d_top)    -- independent  |    d2 < 0        |    d2 < 0        |   d2 > 0          |
 *	ob0 = m2 (m_size)                            | center=d1+0.5*d2 | center=d1+0.5*d2 | center=d1+0.5*d2  |
 *	ob1 = d2 (d_in  or d_bottom) -- piggy-backed |   size=-d2       |   size=-d2       |   size=d2         |
 *	a = x1   (x_center)                          |    top=d1        |    out=d1        |    top=d1+d2      |
 *	b = x2   (x_size)                            | bottom=d1+d2     |     in=d1+d2     | bottom=d1         |
 *      i = Invert (0 -> +1   1 -> -1, implemented as 1-2i)
 *	a0 = d1:Offset
 *	a1 = d1:Scale
 *	b0 = d2:Offset
 *	b1 = d2:Scale
 *	nla = (1=wo/Offsets)[rel,vel] (0=w/Offsets)[abs,pos]
 */
static long tsubCSApAxs (struct tsubRecord *pRec) {
	double d2_real;

	if ( (pRec->a1 == 0.0) || (pRec->b1 == 0.0) )
	{
		return (-1);
	}

	pRec->oa1 = pRec->a - (0.5 - pRec->i) * pRec->b;           /* d1=x1-0.5*inv*x2 */
	pRec->ob1 = pRec->a + (0.5 - pRec->i) * pRec->b;           /* d2=x1+0.5*inv*x2 */

	d2_real = (pRec->ob1 - pRec->oa1);                         /* d2'= d2-d1 */

	if (pRec->nla == 0.0)
	{
		pRec->oa0 = (pRec->oa1 - pRec->a0) / pRec->a1;     /* m1=(d1 -offset1)/scale1 */
		pRec->ob0 = ( d2_real  - pRec->b0) / pRec->b1;     /* m2=(d2'-offset2)/scale2 */
	}
	else
	{
		pRec->oa0 = (pRec->oa1) / pRec->a1;                /* m1=d1 /scale1 */
		pRec->ob0 =  (d2_real)  / pRec->b1;                /* m2=d2'/scale2 */
	}
	return (0);
}

/* ===========================================
 * tsubCSApSpeed - Speed propagation spreadsheet
 *	oa0 = m1    (m_position)                        |  ACCEL CSAv      |  ACCEL CSAh      |
 *	ob0 = m2    (m_size)                            | d1=top,d2=piggy  | d1=out,f2=piggy  |
 *	oa1 = d1    (d_out or d_top)    -- independent  |    d2 < 0        |    d2 < 0        |
 *	ob1 = d2    (d_in  or d_bottom) -- piggy-backed | center=d1+0.5*d2 | center=d1+0.5*d2 |
 *	oa2 = x1    (x_center)                          |   size=-d2       |   size=-d2       |
 *	ob2 = x2    (x_size)                            |    top=d1        |    out=d1        |
 *      oj  = sdis (sdis=1 -- disable record processing)| bottom=d1+d2     |     in=d1+d2     |
 *	a = m1_max
 *	b = m2_max
 *	a0 = m1     (m_position)
 *	b0 = m2     (m_size)
 *	a1 = d1     (d_out or d_top)    -- independent
 *	b1 = d2     (d_in  or d_bottom) -- piggy-backed
 *	a2 = x1     (x_center)
 *	b2 = x2     (x_size)
 *	a3 = d1:Scale
 *	b3 = d2:Scale
 *      nla = Index of input(m1=1, m2=2, d1=11, d2=12, x1=21, x2=22)
 */
static long tsubCSApSpeed (struct tsubRecord *pRec) {
	double prcn = 0.0;
	double m1, m2, d1, d2, x1, x2;
	long ifail = 0;
/* Disable next record processing in order to avoid infinite loop.
 * This actually points to a record linked to the SDIS field of the tsub.
 * The SDIS has to be re-enabled before next tsub record call */
   	pRec->oj = 1;                                    /* sdis=1 */

  	if (tsubCSDebug > 1) printf ("tsubCSApSpeed: called with n=%f \n",pRec->nla);

	if ( pRec->a  == 0.0 || pRec->b  == 0.0 ||
             pRec->a3 == 0.0 || pRec->b3 == 0.0 ) {
	   printf ("tsubCSApSpeed: exit on zero calc parameters\n");
	   printf ("tsubCSApSpeed: max1=%5.2f sca1=%g\n",pRec->a,pRec->a3);
	   printf ("tsubCSApSpeed: max2=%5.2f sca2=%g\n",pRec->b,pRec->b3);
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
/* In piggy-back slit the speed of center is the same as the speed of the first blade */
	      d1 = fabs( 1000.0 *  pRec->a * pRec->a3 );   /* d1=m1*scale1 */
              prcn = fabs( pRec->a2 / d1 );
	   }
	}
	else if (pRec->nla == 22.0) /* ------------------------- x2 speed changed */
	{
	   if ( pRec->b2  < 0.0 ) ifail = -1;              /* x,d speeds are always > 0 */
	   if ( pRec->b2 <= 0.0 ) prcn = 1.0;              /* if 0, then set to max */
	   else {
/* In piggy-back slit the speed of size is the same as the speed of the second blade */
	      d2 = fabs( 1000.0 *  pRec->b * pRec->b3 );   /* d2=m2*scale2 */
              prcn = fabs( pRec->b2 / d2 );
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
	x1 = d1;                                         /* center */
	x2 = d2;                                         /* size */
	pRec->oa0 = m1;
	pRec->ob0 = m2;
	pRec->oa1 = d1;
	pRec->ob1 = d2;
	pRec->oa2 = x1;
	pRec->ob2 = x2;
  	if (tsubCSDebug > 1) printf ("tsubCSApSpeed: m1=%5.2f  m2=%5.2f\n",m1,m2);
	return (ifail);
}

/* ===========================================
 *               Names registration
 *  =========================================== */
static registryFunctionRef tsubCSApRef[] = {
    {"tsubCSAp",      (REGISTRYFUNCTION)tsubCSAp},
    {"tsubCSApSync",  (REGISTRYFUNCTION)tsubCSApSync},
    {"tsubCSApMtr",   (REGISTRYFUNCTION)tsubCSApMtr},
    {"tsubCSApDrv",   (REGISTRYFUNCTION)tsubCSApDrv},
    {"tsubCSApAxs",   (REGISTRYFUNCTION)tsubCSApAxs},
    {"tsubCSApSpeed", (REGISTRYFUNCTION)tsubCSApSpeed}
};

static void tsubCSApFunc(void) {				/* declare this via registrar in DBD */
    registryFunctionRefAdd(tsubCSApRef,NELEMENTS(tsubCSApRef));
}
epicsExportRegistrar(tsubCSApFunc);

