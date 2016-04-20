/* @(#) tsubMO.c 2.0 2004/05/31 -- speed propagation */

/* tsubMO.c - Transformation Subroutines For Monochromator Energy */
/*            This is for Accel monochromator (no trolley X)      */

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

#define   	PI2360 	     ((double) 1.7453292519943295769236907684886e-2)	/* = 2*pi/360 */
#define   	DEG2RAD(deg) ((deg) * PI2360)
#define   	RAD2DEG(rad) ((rad) / PI2360)
#define		SIND(deg)      sin(DEG2RAD(deg))
#define		COSD(deg)      cos(DEG2RAD(deg))

volatile int tsubMODebug = 0;
#define TSUB_MESSAGE	logMsg
#define TSUB_TRACE(level,code) { if ( (pRec->tpro == (level)) || (tsubMODebug == (level)) ) { code } }

double coQB, siQB;                      /* added by Sergey 2001/04/20 */

/*####################################################################*/
/* =========================================== - Energy Initialization
 * tsubMOEn - Energy Initialization
 */
static long tsubMOEn (struct tsubRecord *pRec) {
 	return (0);
}


/* ===========================================
 * tsubMOEnSync
 *	oa = m1:RqsPos
 *	ob = m2:RqsPos
 *	a  = m1:ActPos
 *	b  = m2:ActPos
 */
static long tsubMOEnSync (struct tsubRecord *pRec) {
	pRec->oa = pRec->a;
	pRec->ob = pRec->b;
	return (0);
}


/* =========================================== - Energy Motors
 * tsubMOEnMtr - Energy Motors
 *	oa = x1  (=E)      a = m1
 *	ob = x2  (=L)      b = m2
 *	oa1 = d1 (=QB)     g = ESinTheta
 *	ob1 = d2 (=t2)     h = EvLambda
 *	oa2 = m1_int       i = BeamOffset
 *	oa3 = m1_fra       a0 = d1:Offset
 *	                   a1 = d1:Scale
 *	                   b0 = d2:Offset
 *	                   b1 = d2:Scale
 *                         m = d1:ActPos (QB:ActPos)
 *                         p = x1:ActPos (E:ActPos)
 *	                   nla = (0=w/Offsets)[abs,pos] (1=wo/Offsets)[rel,vel]
 */
static long tsubMOEnMtr (struct tsubRecord *pRec) {
	if (pRec->nla == 0.0) {		/* ----------Absolute Motion--------- */
		pRec->oa1 = pRec->a * pRec->a1 + pRec->a0;		      /* d1=m1*scale1+offset1 */
		pRec->ob1 = pRec->b * pRec->b1 + pRec->b0;		      /* d2=m2*scale2+offset2 */
		siQB = SIND(pRec->oa1);                                       /* sin(QB)              */
		if (siQB != 0.0)     pRec->oa = pRec->g / siQB;               /* E=EsinTheta/sin(QB)  */
		if (pRec->oa != 0.0) pRec->ob = pRec->h / pRec->oa;           /* L=EvLambda/E         */
                pRec->oa2 = (long)(pRec->a);
		pRec->oa3 = pRec->a - pRec->oa2;
	}
	else {				/* ----------Relative Motion--------- */
		pRec->oa1 = pRec->a * pRec->a1;                               /* d1=m1*scale1         */
		pRec->ob1 = pRec->b * pRec->b1;                               /* d2=m2*scale2         */
		siQB = SIND(pRec->m);                                         /* sin(QB)              */
		coQB = COSD(pRec->m);                                         /* cos(QB)              */
		if (siQB != 0.0) {
			pRec->oa = -1.0 * pRec->g * coQB * DEG2RAD(pRec->oa1) /* dE=-EsinTheta*cos(QB)*dQ/sin^2(QB) */
			         / (siQB*siQB);
		} else {
			pRec->oa = 0.0;                                        /* dE=0.               */
		}
		if (pRec->p != 0.0) {                                          /* if E#0              */
			pRec->ob = -1.0 * pRec->h * pRec->oa                   /* dL=-EvLambda*dE/E^2 */
				 / (pRec->p * pRec->p);
		} else {
			pRec->ob = 0.0;                                        /* dL=0.               */
		}
	}
	return (0);
}

/* =========================================== - Energy Drives
 * tsubMOEnDrv - Energy Drives
 *	oa = x1  (=E)      a = d1   (=QB)
 *	ob = x2  (=L)      b = d2   (=t2)
 *	oa0 = m1           g = ESinTheta
 *	ob0 = m2           h = EvLambda
 *	oa2 = m1_int       i = BeamOffset
 *      oa3 = m1_fra       a0 = d1:Offset
 *                         a1 = d1:Scale
 *                         b0 = d2:Offset
 *	                   b1 = d2:Scale
 *                         m = d1:ActPos (QB:ActPos)
 *                         p = x1:ActPos (E:ActPos)
 *	                   nla = (0=w/Offsets)[abs,pos] (1=wo/Offsets)[rel,vel]
 */
static long tsubMOEnDrv (struct tsubRecord *pRec) {
	if ((pRec->a1 == 0.0) || (pRec->b1 == 0.0)) return (-1);

	if (pRec->nla == 0.0) {		/* ----------Absolute Motion--------- */
		pRec->oa0 = (pRec->a - pRec->a0) / pRec->a1;                  /* m1=(d1-offset1)/scale1 */
		pRec->ob0 = (pRec->b - pRec->b0) / pRec->b1;                  /* m2=(d2-offset2)/scale2 */
		siQB = SIND(pRec->a);                                         /* sin(QB)                */
		if (siQB != 0.0)     pRec->oa = pRec->g / siQB;               /* E=EsinTheta/sin(QB)    */
		if (pRec->oa != 0.0) pRec->ob = pRec->h / pRec->oa;           /* L=EvLambda/E           */
                pRec->oa2 = (long)(pRec->oa0);
		pRec->oa3 = pRec->oa0 - pRec->oa2;
	}
	else {				/* ----------Relative Motion--------- */
		pRec->oa0 = (pRec->a) / pRec->a1;                             /* m1=d1/scale1           */
		pRec->ob0 = (pRec->b) / pRec->b1;                             /* m2=d2/scale2           */
		siQB = SIND(pRec->m);                                         /* sin(QB)                */
		coQB = COSD(pRec->m);                                         /* cos(QB)                */
		if (siQB != 0.0) {
			pRec->oa = -1.0 * pRec->g * coQB * DEG2RAD(pRec->a)   /* dE=-EsinTheta*cos(QB)*dQ/sin^2(QB) */
			         / (siQB*siQB);
		} else {
			pRec->oa = 0.0;                                       /* dE=0.                  */
		}
		if (pRec->p != 0.0) {                                         /* if E#0                 */
			pRec->ob = -1.0 * pRec->h * pRec->oa                  /* dL=-EvLambda*dE/E^2    */
			         / (pRec->p * pRec->p);
		} else {
			pRec->ob = 0.0;                                       /* dL=0.                  */
		}
	}
	return (0);
}

/* =========================================== - Energy Axes
 * tsubMOEnAxs - Energy Axes
 *	oa0 = m1           a = x1
 *	oa1 = d1 (=QB)     b = x2
 *	ob0 = m2           f = TrolleyFlag
 *	ob1 = d2 (=t2)     g = ESinTheta
 *	oa2 = m1_int       h = EvLambda
 *	oa3 = m1_fra       i = BeamOffset
 *	                   a0 = d1:Offset
 *	                   a1 = d1:Scale
 *	                   b0 = d2:Offset
 *	                   b1 = d2:Scale
 *                         m = d1:ActPos     (QB:ActPos)
 *                         n = d2:RqsPos     (T2:RqsPos)
 *                         p = x1:ActPos     (E:ActPos)
 *                         nla = (0=w/Offsets)[abs,pos] (1=wo/Offsets)[rel,vel]
 */
static long tsubMOEnAxs (struct tsubRecord *pRec) {
	if ((pRec->a1 == 0.0) || (pRec->b1 == 0.0)) return (-1);

	if (pRec->nla == 0.0) { /* ------------Absolute Motion--------------- */
		if (pRec->a != 0.0) {                                         /* if E#0                 */
		        siQB = pRec->g / pRec->a;                             /* sin(QB)                */
                        if      (siQB > 1.0)  pRec->oa1 = 90.0;
                        else if (siQB < -1.0) pRec->oa1 = -90.0;
			else                  pRec->oa1 = RAD2DEG (asin(siQB)); /* QB=asin(EsinTheta/E)   */
		}
		pRec->oa0 = (pRec->oa1 - pRec->a0) / pRec->a1;                /* m1=(d1-offset1)/scale1 */
                pRec->oa2 = (long)(pRec->oa0);
		pRec->oa3 = pRec->oa0 - pRec->oa2;
		siQB = SIND(pRec->oa1);                                       /* sin(QB)                */
		coQB = COSD(pRec->oa1);                                       /* cos(QB)                */

                /* Trolley-Y (Crystal2 T2): */
		if (pRec->f != 0 && coQB != 0.0) {                            /* if TrolleyFlag # 1     */
		   	pRec->ob1 = pRec->i / (2.0 * coQB);                   /* d2=t2=BeamOffset/[2*cos(QB)] */
		} else {
		   	pRec->ob1 = pRec->n;		                     /* d2=d2 */
		}
		pRec->ob0 = (pRec->ob1 - pRec->b0) / pRec->b1;	             /* m2=(d2-offset2)/scale2       */
	}
	else {	/* --------------------Relative Motion----------------------- */
		if (pRec->p != 0.0) {                                         /* if E#0                */
			siQB = pRec->g / pRec->p;                             /* sin(QB)=(EsinTheta/E) */
			if      (siQB > 1.0)  {siQB = 1.0;  coQB = 0.0;}
			else if (siQB < -1.0) {siQB = -1.0; coQB = 0.0;}
			else                  {coQB = sqrt( 1.0 - siQB*siQB );}
			if (coQB != 0.0) {
				pRec->oa1 = -1.0 * pRec->g * RAD2DEG(pRec->a) /* d1=d(QB)=-EsinTheta*dE/[E^2*cos(QB)]    */
					  / ( pRec->p * pRec->p * coQB );
			}
			else {
				pRec->oa1 = 0.0;                              /* d1=d(QB)=0.         */
			}
		} else {                                                      /* if E=0              */
			siQB = SIND(pRec->m);                                 /* take QB=QB_actual   */
			coQB = COSD(pRec->m);                                 /* take QB=QB_actual   */
		  	pRec->oa1 = 0.0;                                      /* d1=d(QB)=0.         */
		}
		pRec->oa0 = pRec->oa1 / pRec->a1;                             /* m1 = d1 / d1:Scale  */


                /* Trolley-Y (Crystal2 T2): */
		if (pRec->f != 0 && coQB != 0.0) {                            /* if TrolleyFlag # 0  */
			pRec->ob1 = pRec->i * siQB * DEG2RAD(pRec->oa1)       /* d2=d(t2)=BeamOffset*sin(QB)*dQ  */
				  / (2.0 * coQB * coQB);                      /*         /(2*cos^2(QB))          */
		} else {
			pRec->ob1 = 0.0;                                      /* d2=d(t2)=0.         */
		}
		pRec->ob0 = pRec->ob1 / pRec->b1;                             /* m2=d2/scale2        */
	}
	return (0);
}

/* =========================================== - Axes Energy To Lambda
 * tsubMOEnAxs1 - Axes Energy To Lambda
 *	oa = L               h = EvLambda
 *	a = E                p = x1:ActPos
 *	nla = (0=w/Offsets)[abs,pos] (1=wo/Offsets)[rel,vel]
 */
static long tsubMOEnAxs1 (struct tsubRecord *pRec) {
	if (pRec->nla == 0.0) {		/* ----------Absolute Motion--------- */
		if (pRec->a != 0.0) pRec->oa = pRec->h / pRec->a;             /* L=EvLambda/E        */
	} else {			/* ---------Relative Motion---------- */
		if (pRec->p != 0.0) pRec->oa = -1.0 * pRec->h * pRec->a       /* dL=-EvLambda*dE/E^2 */
					     / (pRec->p * pRec->p);
	}
	return (0);
}

/* =========================================== - Axes Lamdba To Energy
 * tsubMOEnAx2 - Axes Lamdba To Energy
 *	oa = E               h = EvLambda
 *	a = L                q = x2:ActPos
 *	nla = (0=w/Offsets)[abs,pos] (1=wo/Offsets)[rel,vel]
 */
static long tsubMOEnAxs2 (struct tsubRecord *pRec) {
	if (pRec->nla == 0.0) {		/* ----------Absolute Motion--------- */
		if (pRec->a != 0.0) pRec->oa = pRec->h / pRec->a;             /* E=EvLambda/L        */
	} else {			/* ----------Relative Motion--------- */
		if (pRec->q != 0.0) pRec->oa = -1.0 * pRec->h * pRec->a       /* dE=-EvLambda*dL/L^2 */
					     / (pRec->q * pRec->q);
	}
	return (0);
}

/* ===========================================
 * tsubMOEnSpeed - Speed propagation spreadsheet
 *	oa0 = m1   (=rotary)
 *	ob0 = m2   (=translation_2)
 *	oa1 = d1   (=QB)
 *	ob1 = d2   (=t2)
 *	oa2 = x1   (=E)
 *	ob2 = x2   (=L)
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
 *	g = ESinTheta
 *	h = EvLambda
 *      m = d1:ActPos (QB:ActPos)
 *      p = x1:ActPos (E:ActPos)
 *      nla = Index of input (m1=1, m2=2, d1=11, d2=12, x1=21, x2=22)
 */
static long tsubMOEnSpeed (struct tsubRecord *	pRec) {
	double prcn = 0.0;
	double m1, m2, d1, d2, x1, x2;
	long ifail = 0;
/* Disable next record processing in order to avoid infinite loop.
 * This actually points to a record linked to the SDIS field of the tsub.
 * The SDIS has to be re-enabled before next tsub record call */
   	pRec->oj = 1;                                      /* sdis=1 */

  	if (tsubMODebug > 1) printf ("tsubMOEnSpeed: called with n=%f \n",pRec->nla);

	siQB = SIND(pRec->m);                              /* sin(QB) */
	coQB = COSD(pRec->m);                              /* cos(QB) */

	if ( pRec->a  == 0.0 || pRec->b  == 0.0 ||
             pRec->a3 == 0.0 || pRec->b3 == 0.0 ||
             pRec->g  == 0.0 || pRec->h  == 0.0 ||
             siQB     == 0.0 || coQB     == 0.0 ) {
	   printf ("tsubMOEnSpeed: exit on zero calc parameters\n");
	   printf ("tsubMOEnSpeed: max1=%5.2f sca1=%g\n",pRec->a,pRec->a3);
	   printf ("tsubMOEnSpeed: max2=%5.2f sca2=%g\n",pRec->b,pRec->b3);
	   printf ("tsubMOEnSpeed: ESinTheta=%g EvLambda=%g\n",pRec->g,pRec->h);
	   printf ("tsubMOEnSpeed: sin(QB)=%g cos(QB)=%g\n",siQB,coQB);
           return (-1);
	}

	if      (pRec->nla ==  1.0) {    /* --------------------- m1 speed changed */
	   if ( pRec->a0 == 0.0 ) prcn = 1.0;              /* if 0, then set to max */
	   else prcn = fabs( pRec->a0 / pRec->a );         /* prcn=m1/m1_max */
	}
	else if (pRec->nla ==  2.0) {    /* --------------------- m2 speed changed */
	   if (pRec->b0 == 0.0) prcn = 1.0;                /* if 0, then set to max */
	   else prcn = fabs( pRec->b0 / pRec->b );         /* prcn=m2/m2_max */
	}
	else if (pRec->nla == 11.0) {    /* --------------------- d1 speed changed */
	   if (pRec->a1  < 0.0) ifail = -1;                /* x,d speeds are always > 0 */
	   if (pRec->a1 <= 0.0) prcn = 1.0;                /* if 0, then set to max */
/* m speed must have same sign as m_max */
	   else prcn = fabs( pRec->a1 / (1000.0 * pRec->a * pRec->a3) );
	}
	else if (pRec->nla == 12.0) {    /* --------------------- d2 speed changed */
	   if (pRec->b1  < 0.0) ifail = -1;                /* x,d speeds are always > 0 */
	   if (pRec->b1 <= 0.0) prcn = 1.0;                /* if 0, then set to max */
/* m speed must have same sign as m_max */
	   else prcn = fabs( pRec->b1 / (1000.0 * pRec->b * pRec->b3) );
	}
	else if (pRec->nla == 21.0) {    /* --------------------- x1 speed changed */
	   if (pRec->a2  < 0.0) ifail = -1;                /* x,d speeds are always > 0 */
	   if (pRec->a2 <= 0.0) prcn = 1.0;                /* if 0, then set to max */
	   else {
/* d1=dQ/dt=m1*scale1 */
	     d1 = fabs( 1000.0 * pRec->a * pRec->a3 );
/* maximum speed: max(x1)=max(dE/dt)=-(EsinTheta*cos(QB)/sin^2(QB))*max(dQ/dt) */
	     x1 = -(pRec->g * coQB/(siQB*siQB)) * DEG2RAD(d1);
             prcn = fabs( pRec->a2 / x1 );
	   }
	}
	else if (pRec->nla == 22.0) {    /* --------------------- x2 speed changed */
	   if (pRec->b2  < 0.0) ifail = -1;                /* x,d speeds are always > 0 */
	   if (pRec->b2 <= 0.0) prcn = 1.0;                /* if 0, then set to max */
	   else {
/* d1=dQ/dt=m1*scale1 */
	     d1 = fabs( 1000.0 * pRec->a * pRec->a3 );
/* maximum speed: max(x1)=max(dE/dt)=-(EsinTheta*cos(QB)/sin^2(QB))*max(dQ/dt) */
	     x1 = -(pRec->g * coQB/(siQB*siQB)) * DEG2RAD(d1);
/* maximum speed: max(x2)=max(dL/dt)=-(EvLambda/E^2)*max(dE/dt) */
	     x2 = -(pRec->h / (pRec->p*pRec->p)) * x1;
             prcn = fabs( pRec->b2 / x2 );
	   }
	}
	else {
	   return (-1);
	}

	if (prcn < 0.0 || prcn > 1.0) {
/* If **anything** is wrong set speed to normal */
          ifail = -1;
	  prcn = 1.0;
	}
/* If we are scanning trolley, we should not reduce the speed of rotary,
 * since the rotary is generally slower than trolley and it may limit
 * the scan speed of trolley */
	if ( pRec->nla == 2.0 ||
             pRec->nla == 12.0 ) m1 = pRec->a;                    /* m1=m1_max */
	else                     m1 = prcn * pRec->a;             /* m1=prcn*m1_max */
	m1 = prcn * pRec->a;                                      /* m1=prcn*m1_max */
	m2 = prcn * pRec->b;                                      /* m2=prcn*m2_max */
	d1 = fabs( 1000.0 *  m1 * pRec->a3 );                     /* d1=dQ/dt=m1*scale1 */
	d2 = fabs( 1000.0 *  m2 * pRec->b3 );                     /* d2=dt2/dt=m2*scale2 */
	x1 = fabs( pRec->g * DEG2RAD(d1) * coQB / (siQB*siQB) );  /* x1=dE/dt=-(EsinTheta*cos(QB)/sin^2(QB))*dQ/dt */
	x2 = fabs( pRec->h * x1 / (pRec->p*pRec->p) );            /* x2=dL/dt=-(EvLambda/E^2)*dE/dt */
	pRec->oa0 = m1;
	pRec->ob0 = m2;
	pRec->oa1 = d1;
	pRec->ob1 = d2;
	pRec->oa2 = x1;
	pRec->ob2 = x2;
  	if (tsubMODebug > 1) printf ("tsubMOEnSpeed: m1=%5.2f  m2=%5.2f\n",m1,m2);
	return (ifail);
}

/* ===========================================
 *               Names registration
 *  =========================================== */
static registryFunctionRef tsubMOEnRef[] = {
    {"tsubMOEn",      (REGISTRYFUNCTION)tsubMOEn},
    {"tsubMOEnSync",  (REGISTRYFUNCTION)tsubMOEnSync},
    {"tsubMOEnMtr",   (REGISTRYFUNCTION)tsubMOEnMtr},
    {"tsubMOEnDrv",   (REGISTRYFUNCTION)tsubMOEnDrv},
    {"tsubMOEnAxs",   (REGISTRYFUNCTION)tsubMOEnAxs},
    {"tsubMOEnAxs1",  (REGISTRYFUNCTION)tsubMOEnAxs1},
    {"tsubMOEnAxs2",  (REGISTRYFUNCTION)tsubMOEnAxs2},
    {"tsubMOEnSpeed", (REGISTRYFUNCTION)tsubMOEnSpeed}
};

static void tsubMOEnFunc(void) {				/* declare this via registrar in DBD */
    registryFunctionRefAdd(tsubMOEnRef,NELEMENTS(tsubMOEnRef));
}
epicsExportRegistrar(tsubMOEnFunc);

