/* @(#) tsubCCD.c, version 2.0 2004/04/27 -- speed propagation; support axis sign and start changed */

/* tsubCCDSt.c - Transformation Subroutines for CCD detector 2*Theta and distance      *
 *               (GM/CA CAT detector mounted on A-frame and supported by 3 motors)     *
 * tsubCCDLp.c - Transformation Subroutines for CCD detector lateral position          *
 *		 with respect to x-ray beam deflected by Horiz.Focusing Mirror (HFM)   *
 *               (GM/CA CAT detector mounted on A-frame with 1 motor for latera shift) */

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

#define   	PI2360       ((double) 1.7453292e-2)	/* = 2*pi/360 */
#define   	DEG2RAD(deg) ((deg) * PI2360)
#define   	RAD2DEG(rad) ((rad) / PI2360)
#define		SIND(deg)      sin(DEG2RAD(deg))
#define		COSD(deg)      cos(DEG2RAD(deg))
#define		TAND(deg)      tan(DEG2RAD(deg))

volatile int tsubCCDDebug = 0;
#define TSUB_MESSAGE	logMsg
#define TSUB_TRACE(level,code) { if ( (pRec->tpro == (level)) || (tsubCCDDebug == (level)) ) { code } }

/* Distances are measured in mm, detector angle is degr., and mirror angles are in mrad */


/* ===========================================
 * tsubCCDSt - CCD support
 */
static long tsubCCDSt (struct tsubRecord *pRec) {
        return (0);
}


/* ===========================================
 * tsubCCDStSync
 *	oa = m1:RqsPos
 *	ob = m2:RqsPos
 *	oc = m2:RqsPos
 *	a  = m1:ActPos
 *	b  = m2:ActPos
 *	c  = m3:ActPos
 */
static long tsubCCDStSync (struct tsubRecord *pRec) {
	pRec->oa = pRec->a;
	pRec->ob = pRec->b;
	pRec->oc = pRec->c;
	return (0);
}


/* ===========================================
 * tsubCCDStMtr - CCD support Motors
 *	oa  = x1	(detector distance)
 *	ob  = x2        (detector 2*theta)
 *	oa1 = d1	(DD  - detector X-slide)
 *	ob1 = d2	(VUS - detector vertical Upstream slide)
 *	oc1 = d3	(VDS - detector vertical Downstream slide)
 *	a  = m1
 *	b  = m2
 *	c  = m3
 *	j  = Lv
 *	k  = Lh
 *	l  = L
 *	p  = d1_ActPos	(X-slide actual)
 *	q  = d2_ActPos	(VUS actual)
 *	r  = x1_ActPos	(Distance actual)
 *	t  = x2_ActPos	(2*Theta actual)
 *	a0 = d1:Offset
 *	a1 = d1:Scale
 *	b0 = d2:Offset
 *	b1 = d2:Scale
 *	c0 = d3:Offset
 *	c1 = d3:Scale
 *	nla = (0=w/Offsets)[abs,pos] (1=wo/Offsets)[rel,vel]
 */
static long tsubCCDStMtr (struct tsubRecord *pRec) {
	double Du=0.0,
	       D=0.0,
	       Si=0.0,
	       Co=0.0;

	if (pRec->nla == 0.0)   /* ------- Absolute motion --------- */
	{
		pRec->oa1 = pRec->a * pRec->a1 + pRec->a0;                    /* d1=m1*scale1+offset1 */
		pRec->ob1 = pRec->b * pRec->b1 + pRec->b0;                    /* d2=m2*scale2+offset2 */
		pRec->oc1 = pRec->c * pRec->c1 + pRec->c0;                    /* d3=m3*scale3+offset3 */

		Du = (pRec->oa1+pRec->k)*(pRec->oa1+pRec->k)                  /* Du^2=(X+Lh)^2+(Hu+Lv)^2-Lv^2 */
                   + (pRec->ob1+pRec->j)*(pRec->ob1+pRec->j)
                   - (pRec->j)*(pRec->j);
	        if ( Du <= 0.0 ) {
		     if (tsubCCDDebug > 0) printf("***tsub=%s(nla=%1.0f): (D+Lh)^2=%g <= 0 -- exit!\n",
                                                                          pRec->name,pRec->nla,Du);
                     return (-1);
		}
		Du = sqrt(Du);                                                /* Du is: Du=D+Lh */
		D  = Du - pRec->k;	                                      /*  D = Du-Lh */
  	        if (tsubCCDDebug > 2) printf("+++tsub=%s(nla=%1.0f): D+Lh=%7.2f, D=%7.2f\n",
                                                            pRec->name,pRec->nla,Du,D);
		Si = ((pRec->ob1)*Du+(pRec->j)*(D-pRec->oa1))	              /* sin(2Q)=[Hu*Du+Lv(D-X)]/[Du^2+Lv^2] */
		   / (Du*Du+(pRec->j)*(pRec->j));
	        if ( fabs(Si) > 1.0 ) {
		     if (tsubCCDDebug > 0) printf("***tsub=%s(nla=%1.0f): |sin(2Q)|=%g > 1 -- exit!\n",
                                                                              pRec->name,pRec->nla,Si);
                     return (-1);
		}
		if (tsubCCDDebug > 2) printf("+++tsub=%s(nla=%1.0f): sin(2Q)=%g\n",
                                                          pRec->name,pRec->nla,Si);
		pRec->oa = D;		                                      /*  D = Du-Lh */
		pRec->ob = RAD2DEG(asin(Si));                                 /* 2Q = asin(sin(2Q)) */
	}
	else                    /* ------- Relative motion --------- */
	{
		pRec->oa1 = pRec->a * pRec->a1;                               /* d1=m1*scale1 */
		pRec->ob1 = pRec->b * pRec->b1;                               /* d2=m2*scale2 */
		pRec->oc1 = pRec->c * pRec->c1;                               /* d3=m3*scale3 */

		/* First calculate absolute positions based on actual X and Hu */
		Du = (pRec->p+pRec->k)*(pRec->p+pRec->k)                      /* Du^2=(X+Lh)^2+(Hu+Lv)^2-Lv^2 */
                   + (pRec->q+pRec->j)*(pRec->q+pRec->j)
                   - (pRec->j)*(pRec->j);
	        if ( Du <= 0.0 ) {
		     if (tsubCCDDebug > 0) printf("***tsub=%s(nla=%1.0f): (D+Lh)^2=%g <= 0 -- exit!\n",
                                                                          pRec->name,pRec->nla,Du);
                     return (-1);
		}
		Du = sqrt(Du);                                                /* Du is: Du=D+Lh */
		D  = Du - pRec->k;	                                      /*  D = Du-Lh */
  	        if (tsubCCDDebug > 2) printf("+++tsub=%s(nla=%1.0f): D+Lh=%7.2f, D=%7.2f\n",
                                                            pRec->name,pRec->nla,Du,D);
		Si = ((pRec->q)*Du+(pRec->j)*(D-pRec->p))                     /* sin(2Q)=[Hu*Du+Lv(D-X)]/[Du^2+Lv^2] */
		   / (Du*Du+(pRec->j)*(pRec->j));
	        if ( fabs(Si) > 1.0 ) {
		     if (tsubCCDDebug > 0) printf("***tsub=%s(nla=%1.0f): |sin(2Q)|=%g > 1 -- exit!\n",
                                                                              pRec->name,pRec->nla,Si);
                     return (-1);
		}
		if (tsubCCDDebug > 2) printf("+++tsub=%s(nla=%1.0f): sin(2Q)=%g\n",
                                                          pRec->name,pRec->nla,Si);
		Co = sqrt(1.0-Si*Si);
		if (tsubCCDDebug > 2) printf("+++tsub=%s(nla=%1.0f): cos(2Q)=%g\n",
                                                          pRec->name,pRec->nla,Co);
		pRec->oa =((pRec->oa1)*(pRec->p+pRec->k)                      /* d(D) =[dX*(X+Lh)+dHu*(Hu+Lv)]/Du */
                         + (pRec->ob1)*(pRec->q+pRec->j))
                         / Du;
		pRec->ob = RAD2DEG(((pRec->oa)*Co-pRec->oa1)                  /* d(2Q)=[d(D)*cos(2Q)-dX]/(Hu+Lv) */
                                   /(pRec->q+pRec->j));
	}
	return (0);
}


/* ===========================================
 * tsubCCDStDrv - CCD support Drives
 *	oa = x1		(detector distance)
 *	ob = x2         (detector 2*theta)
 *	oa0 = m1
 *	ob0 = m2
 *	oc0 = m3
 *	a = d1		(DD  - detector X-slide)
 *	b = d2          (VUS - detector vertical Upstream slide)
 *	c = d3          (VDS - detector vertical Downstream slide)
 *	j  = Lv
 *	k  = Lh
 *	l  = L
 *	p  = d1_ActPos	(X-slide actual)
 *	q  = d2_ActPos	(VUS actual)
 *	r  = x1_ActPos	(Distance actual)
 *	t  = x2_ActPos	(2*Theta actual)
 *	a0 = d1:Offset
 *	a1 = d1:Scale
 *	b0 = d2:Offset
 *	b1 = d2:Scale
 *	c0 = d3:Offset
 *	c1 = d3:Scale
 *	nla = (0=w/Offsets)[abs,pos] (1=wo/Offsets)[rel,vel]
 */
static long tsubCCDStDrv (struct tsubRecord *pRec) {
	double Du=0.0,
	       D=0.0,
	       Si=0.0,
	       Co=0.0;

	if ( (pRec->a1 == 0.0) || (pRec->b1 == 0.0) || (pRec->c1 == 0.0) ) {
		if (tsubCCDDebug > 0) printf("***tsub=%s: scale=0 -- exit!\n",pRec->name);
                return (-1);
	}

	if (pRec->nla == 0.0)   /* ------- Absolute motion --------- */
	{
		pRec->oa0 = (pRec->a - pRec->a0) / pRec->a1;                  /* m1=(d1-offset1)/scale1 */
		pRec->ob0 = (pRec->b - pRec->b0) / pRec->b1;                  /* m2=(d2-offset2)/scale2 */
		pRec->oc0 = (pRec->c - pRec->c0) / pRec->c1;                  /* m3=(d3-offset3)/scale3 */

		Du = (pRec->a+pRec->k)*(pRec->a+pRec->k)                      /* Du^2=(X+Lh)^2+(Hu+Lv)^2-Lv^2 */
                   + (pRec->b+pRec->j)*(pRec->b+pRec->j)
                   - (pRec->j)*(pRec->j);
	        if ( Du <= 0.0 ) {
		     if (tsubCCDDebug > 0) printf("***tsub=%s(nla=%1.0f): (D+Lh)^2=%g <= 0 -- exit!\n",
                                                                          pRec->name,pRec->nla,Du);
                     return (-1);
		}
		Du = sqrt(Du);                                                /* Du is: Du=D+Lh */
		D  = Du - pRec->k;	                                      /*  D = Du-Lh */
  	        if (tsubCCDDebug > 2) printf("+++tsub=%s(nla=%1.0f): D+Lh=%7.2f, D=%7.2f\n",
                                                            pRec->name,pRec->nla,Du,D);
		Si = ((pRec->b)*Du+(pRec->j)*(D-pRec->a))	              /* sin(2Q)=[Hu*Du+Lv(D-X)]/[Du^2+Lv^2] */
		   / (Du*Du+(pRec->j)*(pRec->j));
	        if ( fabs(Si) > 1.0 ) {
		     if (tsubCCDDebug > 0) printf("***tsub=%s(nla=%1.0f): |sin(2Q)|=%g > 1 -- exit!\n",
                                                                              pRec->name,pRec->nla,Si);
                     return (-1);
		}
		if (tsubCCDDebug > 2) printf("+++tsub=%s(nla=%1.0f): sin(2Q)=%g\n",
                                                          pRec->name,pRec->nla,Si);
		pRec->oa = D;		                                      /*  D = Du-Lh */
		pRec->ob = RAD2DEG(asin(Si));                                 /* 2Q = asin(sin(2Q)) */
	}
	else                    /* ------- Relative motion --------- */
	{
		pRec->oa0 = (pRec->a) / pRec->a1;                             /* m1=d1/scale1 */
		pRec->ob0 = (pRec->b) / pRec->b1;                             /* m2=d2/scale2 */
		pRec->oc0 = (pRec->c) / pRec->c1;                             /* m3=d3/scale3 */

		/* First calculate absolute positions based on actual X and Hu */
		Du = (pRec->p+pRec->k)*(pRec->p+pRec->k)                      /* Du^2=(X+Lh)^2+(Hu+Lv)^2-Lv^2 */
                   + (pRec->q+pRec->j)*(pRec->q+pRec->j)
                   - (pRec->j)*(pRec->j);
	        if ( Du <= 0.0 ) {
		     if (tsubCCDDebug > 0) printf("***tsub=%s(nla=%1.0f): (D+Lh)^2=%g <= 0 -- exit!\n",
                                                                          pRec->name,pRec->nla,Du);
                     return (-1);
		}
		Du = sqrt(Du);                                                /* Du is: Du=D+Lh */
		D  = Du - pRec->k;	                                      /*  D = Du-Lh */
  	        if (tsubCCDDebug > 2) printf("+++tsub=%s(nla=%1.0f): D+Lh=%7.2f, D=%7.2f\n",
                                                            pRec->name,pRec->nla,Du,D);
		Si = ((pRec->q)*Du+(pRec->j)*(D-pRec->p))                     /* sin(2Q)=[Hu*Du+Lv(D-X)]/[Du^2+Lv^2] */
		   / (Du*Du+(pRec->j)*(pRec->j));
	        if ( fabs(Si) > 1.0 ) {
		     if (tsubCCDDebug > 0) printf("***tsub=%s(nla=%1.0f): |sin(2Q)|=%g > 1 -- exit!\n",
                                                                              pRec->name,pRec->nla,Si);
                     return (-1);
		}
		if (tsubCCDDebug > 2) printf("+++tsub=%s(nla=%1.0f): sin(2Q)=%g\n",
                                                          pRec->name,pRec->nla,Si);
		Co = sqrt(1-Si*Si);
		if (tsubCCDDebug > 2) printf("+++tsub=%s(nla=%1.0f): cos(2Q)=%g\n",
                                                          pRec->name,pRec->nla,Co);
		pRec->oa =((pRec->a)*(pRec->p+pRec->k)                        /* d(D) =[dX*(X+Lh)+dHu*(Hu+Lv)]/Du */
                         + (pRec->b)*(pRec->q+pRec->j))
                         / Du;
		pRec->ob = RAD2DEG(((pRec->oa)*Co-pRec->a)                   /* d(2Q)=[d(D)*cos(2Q)-dX]/(Hu+Lv) */
                                   /(pRec->q+pRec->j));
	}
	return (0);
}


/* ===========================================
 * tsubCCDStAxs - CCD support Axes
 *	oa0 = m1
 *	ob0 = m2
 *	oc0 = m3
 *	oa1 = d1        (DD  - detector X-slide)
 *	ob1 = d2	(VUS - detector vertical Upstream slide)
 *	oc1 = d3	(VDS - detector vertical Downstream slide)
 *	a = x1		(detector distance)
 *	b = x2		(detector 2*theta)
 *	j  = Lv
 *	k  = Lh
 *	l  = L
 *      o  = dH (offset between Hu and Hd)
 *	p  = d1_ActPos	(X-slide actual)
 *	q  = d2_ActPos	(VUS actual)
 *	r  = x1_ActPos	(Distance actual)
 *	t  = x2_ActPos	(2*Theta actual)
 *	a0 = d1:Offset
 *	a1 = d1:Scale
 *	b0 = d2:Offset
 *	b1 = d2:Scale
 *	c0 = d3:Offset
 *	c1 = d3:Scale
 *	nla = (0=w/Offsets)[abs,pos] (1=wo/Offsets)[rel,vel]
 */
static long tsubCCDStAxs (struct tsubRecord *pRec) {
	double Du=0.0,
	       Si=0.0,
	       Co=0.0,
               Tg=0.0;

	if ( (pRec->a1 == 0.0) || (pRec->b1 == 0.0) || (pRec->c1 == 0.0) ) {
		if (tsubCCDDebug > 0) printf("***tsub=%s: scale=0 -- exit!\n",pRec->name);
		return (-1);
	}

	if (pRec->nla == 0.0)   /* ------- Absolute motion --------- */
	{
		Du  = pRec->a + pRec->k;                                      /* Du=D+Lh */
		Si = SIND(pRec->b);
		Co = COSD(pRec->b);
		Tg = TAND(pRec->b);
                if (tsubCCDDebug > 2) {
                	printf("+++tsub=%s: D+Lh=%g\n",pRec->name,Du);
                	printf("+++tsub=%s(nla=%1.0f): 2Q=%g degr\n" ,pRec->name,pRec->nla,pRec->b);
                	printf("+++tsub=%s(nla=%1.0f): sin(2Q)=%g\n",pRec->name,pRec->nla,Si);
                	printf("+++tsub=%s(nla=%1.0f): cos(2Q)=%g\n",pRec->name,pRec->nla,Co);
		}

		pRec->oa1 = Du*Co - (pRec->j)*Si - pRec->k;                   /* X =(D+Lh)*cos(2Q)-Lv*sin(2Q)-Lh */
		pRec->ob1 = Du*Si + (pRec->j)*Co - pRec->j;                   /* Hu=(D+Lh)*sin(2Q)+Lv*cos(2Q)-Lv */
		pRec->oc1 = pRec->ob1 + (pRec->l)*Tg + pRec->o/Co - pRec->o;  /* Hd=Hu+L*tan(2Q)+dH/cos(2Q)-dH */

		pRec->oa0 = (pRec->oa1 - pRec->a0) / pRec->a1;                /* m1=(d1-offset1)/scale1 */
		pRec->ob0 = (pRec->ob1 - pRec->b0) / pRec->b1;                /* m2=(d2-offset2)/scale2 */
		pRec->oc0 = (pRec->oc1 - pRec->c0) / pRec->c1;                /* m3=(d3-offset3)/scale3 */
	}
	else                    /* ------- Relative motion --------- */
	{
		Du = pRec->r + pRec->k;                                       /* Du=D+Lh */
		Si = SIND(pRec->t);
		Co = COSD(pRec->t);
		if ( Co == 0.0 ) {
			if (tsubCCDDebug > 0) printf("***tsub=%s(nla=%1.0f): cos(2Q)=%g=0 -- exit!\n",
                                                                                  pRec->name,pRec->nla,Co);
        		return (-1);
		}
                if (tsubCCDDebug > 2) {
                	printf("+++tsub=%s: D+Lh=%g\n",pRec->name,Du);
                	printf("+++tsub=%s(nla=%1.0f): 2Q=%g rad\n" ,pRec->name,pRec->nla,pRec->t);
                	printf("+++tsub=%s(nla=%1.0f): sin(2Q)=%g\n",pRec->name,pRec->nla,Si);
                	printf("+++tsub=%s(nla=%1.0f): cos(2Q)=%g\n",pRec->name,pRec->nla,Co);
		}

		pRec->oa1 = (pRec->a)*Co-DEG2RAD(pRec->b)*(Du*Si+(pRec->j)*Co);       /*  d(X)=d(D)*cos(2Q)-d(2Q)*(Du*sin(2Q)+Lv*sin(2Q)) */
		pRec->ob1 = (pRec->a)*Si+DEG2RAD(pRec->b)*(Du*Co-(pRec->j)*Si);       /* d(Hu)=d(D)*sin(2Q)+d(2Q)(Du*cos(2Q)-Lv*sin(2Q)) */
		pRec->oc1 = pRec->ob1+DEG2RAD(pRec->b)*(pRec->l+pRec->o*Si)/(Co*Co);  /* d(Hd)=d(Hu)+d(2Q)*[L+dH*sin(2Q)]/cos^2(2Q) */

		pRec->oa0 = (pRec->oa1) / pRec->a1;                           /* m1=d1/scale1 */
		pRec->ob0 = (pRec->ob1) / pRec->b1;                           /* m2=d2/scale2 */
		pRec->oc0 = (pRec->oc1) / pRec->c1;                           /* m3=d3/scale3 */
	}
	return (0);
}


/* ===========================================
 * tsubCCDStSpeed - CCD Support speed propagation spreadsheet
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
 *	j  = Lv
 *	k  = Lh
 *	l  = L
 *      o  = dH (offset between Hu and Hd)
 *	p  = d1_ActPos (X-slide actual)
 *	q  = d2_ActPos (VUS actual)
 *	r  = x1_ActPos	(Distance actual)
 *	t  = x2_ActPos	(2*Theta actual)
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
static long tsubCCDStSpeed (struct tsubRecord *pRec) {
	double prcn = 0.0;
	double m1, m2, m3, d1, d2, d3, x1, x2;
	double Du=0.0,
	       Si=0.0,
	       Co=0.0;
	long ifail = 0;
/* Disable next record processing in order to avoid infinite loop.
 * This actually points to a record linked to the SDIS field of the tsub.
 * The SDIS has to be re-enabled before next tsub record call */
   	pRec->oj = 1;                                    /* sdis=1 */

  	if (tsubCCDDebug > 1) printf ("+++tsub=%s: called with n=%1.0f \n",pRec->name,pRec->nla);

	if ( pRec->a  == 0.0 || pRec->b  == 0.0 || pRec->c  == 0.0 ||
             pRec->a3 == 0.0 || pRec->b3 == 0.0 || pRec->c3 == 0.0 ) {
	   if (tsubCCDDebug > 0) {
	      printf ("***tsub=%s: exit on zero calc parameters\n", pRec->name);
	      printf ("***tsub=%s: max1=%5.2f sca1=%g\n", pRec->name,pRec->a,pRec->a3);
	      printf ("***tsub=%s: max2=%5.2f sca2=%g\n", pRec->name,pRec->b,pRec->b3);
	      printf ("***tsub=%s: max3=%5.2f sca3=%g\n", pRec->name,pRec->c,pRec->c3);
	   }
           return (-1);
	}

	Du  = pRec->r + pRec->k;                                              /* Du=D+Lh */
	if ( Du <= 0.0 ) {
		if (tsubCCDDebug > 0) printf("***tsub=%s(nla=%2.0f): D+Lh=%g <= 0 -- exit!\n",
                                                                pRec->name,pRec->nla,Du);
        	return (-1);
	}
	Si = SIND(pRec->t);
	Co = COSD(pRec->t);
	if ( Co == 0.0 ) {
		if (tsubCCDDebug > 0) printf("***tsub=%s(nla=%1.0f): cos(2Q+2VFM)=%g=0 -- exit!\n",
                                                                          pRec->name,pRec->nla,Co);
        	return (-1);
	}
        if (tsubCCDDebug > 2) {
             	printf("+++tsub=%s: D+Lh=%g\n",pRec->name,Du);
               	printf("+++tsub=%s(nla=%1.0f): 2Q=%g degr\n" ,pRec->name,pRec->nla,pRec->t);
               	printf("+++tsub=%s(nla=%1.0f): sin(2Q)=%g\n",pRec->name,pRec->nla,Si);
               	printf("+++tsub=%s(nla=%1.0f): cos(2Q)=%g\n",pRec->name,pRec->nla,Co);
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
	      d1 = 1000.0 *  pRec->a * pRec->a3;                              /* d1=m1*scale1 */
	      d2 = 1000.0 *  pRec->b * pRec->b3;                              /* d2=m2*scale2 */
	      d3 = 1000.0 *  pRec->c * pRec->c3;                              /* d3=m3*scale3 */
	      x1 = (d1*(pRec->p+pRec->l)+d2*(pRec->q+pRec->j))/Du;            /* d(D) =[dX*(X+Lh)+dHu*(Hu+Lv)]/Du */
	      if ( x1 != 0.0) {
                  prcn = fabs( pRec->a2 / x1 );
	      }
	      else {
	          if (tsubCCDDebug > 0) printf ("***tsub=%s(nla=%1.0f): x1=0\n",pRec->name,pRec->nla);
		  prcn = 1.0;
	      }
	   }
	}
	else if (pRec->nla == 22.0) /* --------------------- x2 speed changed */
	{
	   if ( pRec->b2  < 0.0 ) ifail = -1;                                 /* x,d speeds are always > 0 */
	   if ( pRec->b2 <= 0.0 ) prcn = 1.0;                                 /* if 0, then set to max */
	   else {
	      d1 = 1000.0 *  pRec->a * pRec->a3;                              /* d1=m1*scale1 */
	      d2 = 1000.0 *  pRec->b * pRec->b3;                              /* d2=m2*scale2 */
	      d3 = 1000.0 *  pRec->c * pRec->c3;                              /* d3=m3*scale3 */
	      x1 = (d1*(pRec->p+pRec->l)+d2*(pRec->q+pRec->j))/Du;            /* d(D) =[dX*(X+Lh)+dHu*(Hu+Lv)]/Du */
	      x2 = RAD2DEG((x1*Co-d1)/(pRec->q+pRec->j));                     /* d(2Q)=(d(D)*cos(2Q)-dX)/(Hu+Lv) */
	      if ( x2 != 0.0) {
                  prcn = fabs( pRec->b2 / x2 );
	      }
	      else {
	          if (tsubCCDDebug > 0) printf ("***tsub=%s(nla=%1.0f): x2=0\n",pRec->name,pRec->nla);
		  prcn = 1.0;
	      }
	   }
	}
	else
	{
           if (tsubCCDDebug > 0) printf ("***tsub=%s: incorrect nla=%1.0f -- exit!\n",pRec->name,pRec->nla);
	   return (-1);
	}

	if ( prcn < 0.0 || prcn > 1.0 ) {
/* If **anything** is wrong set speed to normal */
          if (tsubCCDDebug > 0) printf ("***tsub=%s(nla=%1.0f): prcn=%g not in 0-1 range\n",
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
	x1 = (d1*(pRec->p+pRec->l)+d2*(pRec->q+pRec->j))/Du;                  /* d(D) =[dX*(X+Lh)+dHu*(Hu+Lv)]/Du */
	x2 = RAD2DEG((x1*Co-d1)/(pRec->q+pRec->j));                           /* d(2Q)=(d(D)*cos(2Q)-dX)/(Hu+Lv) */
	pRec->oa0 = m1;
	pRec->ob0 = m2;
	pRec->oc0 = m3;
	pRec->oa1 = fabs(d1);
	pRec->ob1 = fabs(d2);
	pRec->oc1 = fabs(d3);
	pRec->oa2 = fabs(x1);
	pRec->ob2 = fabs(x2);
        if (tsubCCDDebug > 1) printf ("+++tsub=%s(nla=%1.0f):  m1=%5.2f  m2=%5.2f  m3=%5.2f\n",
                                                                pRec->name,pRec->nla,m1,m2,m3);
	return (ifail);
}

/* ===========================================
 *               Names registration
 *  =========================================== */
static registryFunctionRef tsubCCDStRef[] = {
    {"tsubCCDSt",      (REGISTRYFUNCTION)tsubCCDSt},
    {"tsubCCDStSync",  (REGISTRYFUNCTION)tsubCCDStSync},
    {"tsubCCDStMtr",   (REGISTRYFUNCTION)tsubCCDStMtr},
    {"tsubCCDStDrv",   (REGISTRYFUNCTION)tsubCCDStDrv},
    {"tsubCCDStAxs",   (REGISTRYFUNCTION)tsubCCDStAxs},
    {"tsubCCDStSpeed", (REGISTRYFUNCTION)tsubCCDStSpeed}
};

static void tsubCCDStFunc(void) {				/* declare this via registrar in DBD */
    registryFunctionRefAdd(tsubCCDStRef,NELEMENTS(tsubCCDStRef));
}
epicsExportRegistrar(tsubCCDStFunc);
