/* @(#) tsubBD3.c, version 1.0 2010/09/28 */

/* tsubBD3St.c - Transformation Subroutines for GM/CA-CAT two-segment beam delivery pipe *
 *              supported by three in-line motors. For additional information see the    *
 *              files README, CalibdsSt_help.gif, and downstream3_equations.ppt in       *
 *              the bd3Db folder                                                         */

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

volatile int tsubBD3Debug = 0;
#define TSUB_MESSAGE	logMsg
#define TSUB_TRACE(level,code) { if ( (pRec->tpro == (level)) || (tsubBD3Debug == (level)) ) { code } }

/* Distances are measured in mm and the pipe angles is mrad */
/* if x1=mm, x2=mrad, base=m (BioCAT), then FCDS=1; */
/* if x1=mm, x2=mrad, base=mm (GMCA), then FCDS=1000. */
double FCDS = 1000.0;

/* ===========================================
 * tsubBD3St - BD3 support
 */
static long tsubBD3St (struct tsubRecord *pRec) {
        return (0);
}


/* ===========================================
 * tsubBD3StSync
 *	oa = m1:RqsPos
 *	ob = m2:RqsPos
 *	oc = m2:RqsPos
 *	a  = m1:ActPos
 *	b  = m2:ActPos
 *	c  = m3:ActPos
 */
static long tsubBD3StSync (struct tsubRecord *pRec) {
	pRec->oa = pRec->a;
	pRec->ob = pRec->b;
	pRec->oc = pRec->c;
	return (0);
}


/* ===========================================
 * tsubBD3StMtr - BD3 support Motors
 *	oa  = x1	(sample height)
 *	ob  = x2        (gonio support angle)
 *	oa1 = d1	(upstream support slide)
 *	ob1 = d2	(midstream support slide [upstream of gonio platform])
 *	oc1 = d3	(downstream support slide [downstream of gonio platform])
 *	oh  = ANGLE	(overall pipe angle from mirror to sample)
 *	a  = m1
 *	b  = m2
 *	c  = m3
 *	d  = Lm1
 *      e  = L12
 *      f  = L23
 *	g  = Ls3
 *	a0 = d1:Offset
 *	a1 = d1:Scale
 *	b0 = d2:Offset
 *	b1 = d2:Scale
 *	c0 = d3:Offset
 *	c1 = d3:Scale
 *	nla = (0=w/Offsets)[abs,pos] (1=wo/Offsets)[rel,vel]
 */
static long tsubBD3StMtr (struct tsubRecord *pRec) {
	if (pRec->nla == 0.0)  { /* ------- Absolute motion --------- */
		pRec->oa1 = pRec->a * pRec->a1 + pRec->a0;                    /* d1=m1*scale1+offset1 */
		pRec->ob1 = pRec->b * pRec->b1 + pRec->b0;                    /* d2=m2*scale2+offset2 */
		pRec->oc1 = pRec->c * pRec->c1 + pRec->c0;                    /* d3=m3*scale3+offset3 */
	}
	else {                  /* ------- Relative motion --------- */
		pRec->oa1 = pRec->a * pRec->a1;                               /* d1=m1*scale1 */
		pRec->ob1 = pRec->b * pRec->b1;                               /* d2=m2*scale2 */
		pRec->oc1 = pRec->c * pRec->c1;                               /* d3=m3*scale3 */
	}
	if ( pRec->f == 0.0) {
	   if (tsubBD3Debug > 0) printf("***tsub=%s (nla=%1.0f): L23=0 -- exit!\n",pRec->name,pRec->nla);
           return (-1);
	}
	pRec->ob = FCDS * (pRec->oc1 - pRec->ob1) / pRec->f;		      /* x2=(d3-d2)/L23 */
	pRec->oa = pRec->oc1 + pRec->ob * (pRec->g / FCDS);		      /* x1=d3+Ls3*x2 */

	if (pRec->nla == 0.0) {  /* ------- Absolute motion --------- */
	   pRec->oh = FCDS * pRec->oa / ( pRec->d +  pRec->e +  pRec->f +  pRec->g);  /* ANGLE = x1/(Lm1+L12+L23+Ls3) */
	}
	return (0);
}


/* ===========================================
 * tsubBD3StDrv - BD3 support Drives
 *	oa  = x1	(sample height)
 *	ob  = x2        (gonio support angle)
 *	oa0 = m1
 *	ob0 = m2
 *	oc0 = m3
 *	a = d1		(upstream support slide)
 *	b = d2          (midstream support slide [upstream of gonio platform])
 *	c = d3          (downstream support slide [downstream of gonio platform])
 *	d  = Lm1
 *      e  = L12
 *      f  = L23
 *	g  = Ls3
 *	a0 = d1:Offset
 *	a1 = d1:Scale
 *	b0 = d2:Offset
 *	b1 = d2:Scale
 *	c0 = d3:Offset
 *	c1 = d3:Scale
 *	nla = (0=w/Offsets)[abs,pos] (1=wo/Offsets)[rel,vel]
 */
static long tsubBD3StDrv (struct tsubRecord *pRec) {
	if (pRec->a1 == 0.0 || pRec->b1 == 0.0 || pRec->c1 == 0.0) return (-1);

	if (pRec->nla == 0.0) {  /* ------- Absolute motion --------- */
		pRec->oa0 = (pRec->a - pRec->a0) / pRec->a1;                  /* m1=(d1-offset1)/scale1 */
		pRec->ob0 = (pRec->b - pRec->b0) / pRec->b1;                  /* m2=(d2-offset2)/scale2 */
		pRec->oc0 = (pRec->c - pRec->c0) / pRec->c1;                  /* m3=(d3-offset3)/scale3 */
	}
	else {                   /* ------- Relative motion --------- */
		pRec->oa0 = (pRec->a) / pRec->a1;                             /* m1=d1/scale1 */
		pRec->ob0 = (pRec->b) / pRec->b1;                             /* m2=d2/scale2 */
		pRec->oc0 = (pRec->c) / pRec->c1;                             /* m3=d3/scale3 */
	}
	if ( pRec->f == 0.0) {
	   if (tsubBD3Debug > 0) printf("***tsub=%s (nla=%1.0f): L23=0 -- exit!\n",pRec->name,pRec->nla);
           return (-1);
	}
	pRec->ob = FCDS * (pRec->c - pRec->b) / pRec->f;		      /* x2=(d3-d2)/L23 */
	pRec->oa = pRec->c + pRec->ob * (pRec->g / FCDS);		      /* x1=d3+Ls3*x2 */
	return (0);
}


/* ===========================================
 * tsubBD3StAxs - BD3 support Axes
 *	oa0 = m1
 *	ob0 = m2
 *	oc0 = m3
 *	oa1 = d1        (upstream support slide)
 *	ob1 = d2	(midstream support slide [upstream of gonio platform])
 *	oc1 = d3	(downstream support slide [downstream of gonio platform])
 *	a = x1		(sample height)
 *	b = x2          (gonio support angle)
 *	d  = Lm1
 *      e  = L12
 *      f  = L23
 *	g  = Ls3
 *	a0 = d1:Offset
 *	a1 = d1:Scale
 *	b0 = d2:Offset
 *	b1 = d2:Scale
 *	c0 = d3:Offset
 *	c1 = d3:Scale
 *	nla = (0=w/Offsets)[abs,pos] (1=wo/Offsets)[rel,vel]
 */
static long tsubBD3StAxs (struct tsubRecord *pRec) {
	if (pRec->a1 == 0.0 || pRec->b1 == 0.0 || pRec->c1 == 0.0) return (-1);

	pRec->oc1 = pRec->a - pRec->b * (pRec->g)           / FCDS;	      /* d3=x1-x2*Ls3       */
	pRec->ob1 = pRec->a - pRec->b * (pRec->g + pRec->f) / FCDS;	      /* d2=x1-x2*(Ls3+L23) */
	pRec->oa1 = pRec->a * pRec->d / (pRec->d + pRec->e + pRec->f + pRec->g); /* d1= x1*Lm1/(Lm1+L12+L23+Ls3) */

	if (pRec->nla == 0.0) {  /* ------- Absolute motion --------- */
		pRec->oa0 = (pRec->oa1 - pRec->a0) / pRec->a1;                /* m1=(d1-offset1)/scale1 */
		pRec->ob0 = (pRec->ob1 - pRec->b0) / pRec->b1;                /* m2=(d2-offset2)/scale2 */
		pRec->oc0 = (pRec->oc1 - pRec->c0) / pRec->c1;                /* m3=(d3-offset3)/scale3 */
	}
	else {                   /* ------- Relative motion --------- */
		pRec->oa0 = (pRec->oa1) / pRec->a1;                           /* m1=d1/scale1 */
		pRec->ob0 = (pRec->ob1) / pRec->b1;                           /* m2=d2/scale2 */
		pRec->oc0 = (pRec->oc1) / pRec->c1;                           /* m3=d3/scale3 */
	}
	return (0);
}


/* ===========================================
 * tsubBD3StSpeed - BD3 Support speed propagation spreadsheet
 *	oa0 = m1
 *	ob0 = m2
 *	oc0 = m3
 *	oa1 = d1
 *	ob1 = d2
 *	oc1 = d3
 *	oa2 = x1
 *	ob2 = x2
 *      oj  = sdis (sdis=1 -- disable record processing)
 *	a = m1_max
 *	b = m2_max
 *	c = m3_max
 *	d  = Lm1
 *      e  = L12
 *      f  = L23
 *	g  = Ls3
 *	a0 = m1
 *	b0 = m2
 *	c0 = m3
 *	a1 = d1
 *	b1 = d2
 *	c1 = d3
 *	a2 = x1
 *	b2 = x2
 *	a3 = d1:Scale
 *	b3 = d2:Scale
 *	c3 = d3:Scale
 *      nla = Index of input(m1=1, m2=2, m3=3, d1=11, d2=12, d3=13, x1=21, x2=22)
 */
static long tsubBD3StSpeed (struct tsubRecord *pRec) {
	double prcn = 0.0;
	double m1, m2, m3, d1, d2, d3, x1, x2;
	long ifail = 0;
/* Disable next record processing in order to avoid infinite loop.
 * This actually points to a record linked to the SDIS field of the tsub.
 * The SDIS has to be re-enabled before next tsub record call */
   	pRec->oj = 1;                                    /* sdis=1 */

  	if (tsubBD3Debug > 1) printf ("+++tsub=%s: called with n=%1.0f \n",pRec->name,pRec->nla);

	if ( pRec->a  == 0.0 || pRec->b  == 0.0 || pRec->c  == 0.0 ||
             pRec->a3 == 0.0 || pRec->b3 == 0.0 || pRec->c3 == 0.0 ) {
	   if (tsubBD3Debug > 0) {
	      printf ("***tsub=%s: exit on zero calc parameters\n", pRec->name);
	      printf ("***tsub=%s: max1=%5.2f sca1=%g\n", pRec->name,pRec->a,pRec->a3);
	      printf ("***tsub=%s: max2=%5.2f sca2=%g\n", pRec->name,pRec->b,pRec->b3);
	      printf ("***tsub=%s: max3=%5.2f sca3=%g\n", pRec->name,pRec->c,pRec->c3);
	   }
           return (-1);
	}
	if      (pRec->nla ==  1.0)  /* -------------------- m1 speed changed */
	{
	   if ( pRec->a0 == 0.0 ) prcn = 1.0;                                 /* if 0, then set to max */
	   else prcn = fabs(pRec->a0 / pRec->a);                              /* prcn=m1/m1_max */
	}
	else if (pRec->nla ==  2.0)  /* -------------------- m2 speed changed */
	{
	   if ( pRec->b0 == 0.0 ) prcn = 1.0;                                 /* if 0, then set to max */
	   else prcn = fabs(pRec->b0 / pRec->b);                              /* prcn=m2/m2_max */
	}
	else if (pRec->nla ==  3.0)  /* -------------------- m3 speed changed */
	{
	   if ( pRec->c0 == 0.0 ) prcn = 1.0;                                 /* if 0, then set to max */
	   else prcn = fabs(pRec->c0 / pRec->c);                              /* prcn=m3/m3_max */
	}
	else if (pRec->nla == 11.0) /* --------------------- d1 speed changed */
	{
	   if ( pRec->a1  < 0.0 ) ifail = -1;                                 /* x,d speeds are always > 0 */
	   if ( pRec->a1 <= 0.0 ) prcn = 1.0;                                 /* if 0, then set to max */
/* m speed must have same sign as m_max */
	   else prcn = fabs( pRec->a1 / (1000.0 * pRec->a * pRec->a3) );
	}
	else if (pRec->nla == 12.0) /* --------------------- d2 speed changed */
	{
	   if ( pRec->b1  < 0.0 ) ifail = -1;                                 /* x,d speeds are always > 0 */
	   if ( pRec->b1 <= 0.0 ) prcn = 1.0;                                 /* if 0, then set to max */
/* m speed must have same sign as m_max */
	   else prcn = fabs( pRec->b1 / (1000.0 * pRec->b * pRec->b3) );
	}
	else if (pRec->nla == 13.0) /* --------------------- d3 speed changed */
	{
	   if ( pRec->c1  < 0.0 ) ifail = -1;                                 /* x,d speeds are always > 0 */
	   if ( pRec->c1 <= 0.0 ) prcn = 1.0;                                 /* if 0, then set to max */
/* m speed must have same sign as m_max */
	   else prcn = fabs( pRec->c1 / (1000.0 * pRec->c * pRec->c3) );
	}
	else if (pRec->nla == 21.0) /* --------------------- x1 speed changed */
	{
	   if ( pRec->a2  < 0.0 ) ifail = -1;                                 /* x,d speeds are always > 0 */
	   if ( pRec->a2 <= 0.0 ) prcn = 1.0;                                 /* if 0, then set to max */
	   else {
	      d2 = 1000.0 *  pRec->b * pRec->b3;                              /* d2=m2*scale2 */
	      d3 = 1000.0 *  pRec->c * pRec->c3;                              /* d3=m3*scale3 */
	      x2 = (d3-d2)/pRec->f;                                           /* x2=(d3-d2)/L23 */
	      x1 = d3+x2*pRec->g;                                             /* x1=d3+x2*Ls3 */
	      if (x1 != 0.0) {
                  prcn = fabs( pRec->a2 / x1 );
	      }
	      else {
	          if (tsubBD3Debug > 0) printf ("***tsub=%s(nla=%1.0f): x1=0\n",
                                                          pRec->name,pRec->nla);
		  prcn = 1.0;
	      }
	   }
	}
	else if (pRec->nla == 22.0) /* --------------------- x2 speed changed */
	{
	   if ( pRec->b2  < 0.0 ) ifail = -1;                                 /* x,d speeds are always > 0 */
	   if ( pRec->b2 <= 0.0 ) prcn = 1.0;                                 /* if 0, then set to max */
	   else {
	      d2 = 1000.0 *  pRec->b * pRec->b3;                              /* d2=m2*scale2 */
	      d3 = 1000.0 *  pRec->c * pRec->c3;                              /* d3=m3*scale3 */
	      x2 = (d3+d2)/pRec->f;                                           /* x2=(d3-d2)/L23 */
	      if (x2 != 0.0) {
                  prcn = fabs( pRec->b2 / x2 );
	      }
	      else {
	          if (tsubBD3Debug > 0) printf ("***tsub=%s(nla=%1.0f): x2=0\n",
                                                          pRec->name,pRec->nla);
		  prcn = 1.0;
	      }
	   }
	}
	else
	{
           if (tsubBD3Debug > 0) printf ("***tsub=%s: incorrect nla=%1.0f -- exit!\n",
                                                          pRec->name,pRec->nla);
	   return (-1);
	}

	if ( prcn < 0.0 || prcn > 1.0 ) {
/* If **anything** is wrong set speed to normal */
          if (tsubBD3Debug > 0) printf ("***tsub=%s(nla=%1.0f): prcn=%g not in 0-1 range\n",
                                                          pRec->name,pRec->nla,prcn);
          ifail = -1;
	  prcn = 1.0;
	}
	m1 = prcn * pRec->a;                                                  /* m1=prcn*m1_max */
	m2 = prcn * pRec->b;                                                  /* m2=prcn*m2_max */
	m3 = prcn * pRec->c;                                                  /* m3=prcn*m3_max */
	d1 = 1000.0*m1*(pRec->a3);                                            /* d1=m1*scale1   */
	d2 = 1000.0*m2*(pRec->b3);                                            /* d2=m2*scale2   */
	d3 = 1000.0*m3*(pRec->c3);                                            /* d3=m3*scale3   */
	x2 = (d3+d2)/pRec->f;                                                 /* x2=(d3-d2)/L23 */
	x1 = d3+x2*pRec->g;                                                   /* x1=d3+x2*Ls3 */
	x2 = FCDS*x2;                                                         /* rad -> mrad */
	pRec->oa0 = m1;
	pRec->ob0 = m2;
	pRec->oc0 = m3;
	pRec->oa1 = fabs(d1);
	pRec->ob1 = fabs(d2);
	pRec->oc1 = fabs(d3);
	pRec->oa2 = fabs(x1);
	pRec->ob2 = fabs(x2);
        if (tsubBD3Debug > 1) printf ("+++tsub=%s(nla=%1.0f):  m1=%5.2f  m2=%5.2f  m3=%5.2f\n",
                                                          pRec->name,pRec->nla,m1,m2,m3);
	return (ifail);
}

/* ===========================================
 *               Names registration
 *  =========================================== */
static registryFunctionRef tsubBD3StRef[] = {
    {"tsubBD3St",      (REGISTRYFUNCTION)tsubBD3St},
    {"tsubBD3StSync",  (REGISTRYFUNCTION)tsubBD3StSync},
    {"tsubBD3StMtr",   (REGISTRYFUNCTION)tsubBD3StMtr},
    {"tsubBD3StDrv",   (REGISTRYFUNCTION)tsubBD3StDrv},
    {"tsubBD3StAxs",   (REGISTRYFUNCTION)tsubBD3StAxs},
    {"tsubBD3StSpeed", (REGISTRYFUNCTION)tsubBD3StSpeed}
};

static void tsubBD3StFunc(void) {				/* declare this via registrar in DBD */
    registryFunctionRefAdd(tsubBD3StRef,NELEMENTS(tsubBD3StRef));
}
epicsExportRegistrar(tsubBD3StFunc);

