/* @(#) tsubCollimator.c 2011/07/01 */

/* tsubCollimator.c - Save/Upload collimator presets on the event of changing collimator  */
/* This tsub is associated with the $(BL)bi:copyColPresets record in the bluiceConfig.db */
/* that is processed on the change of $(BL)bi:currentCollimatorSet */

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

volatile int tsubCollDebug = 0;
#define TSUB_MESSAGE	logMsg
#define TSUB_TRACE(level,code) { if ( (pRec->tpro == (level)) || (tsubCollDebug == (level)) ) { code } }

/* ===========================================
 * tsubColl - Initialization
 */
static long tsubColl (struct tsubRecord *pRec) {
/*	printf ("+++tsub=%s:  initialization\n",pRec->name); */
	return (0);
}

/* ===========================================
 * tsubCollSR - save current presets and upload new
 *	a = requested set (1=set1/0=set0)
 *	oa =  memorized index of currently loaded set (1=set1/0=set0)
 *	oa0 ... oa9 = set-0
 *	ob0 ... ob9 = set-1
 *	oc0...oc9 = output presets (new presets to be set)
 *	c0 ... c9 = input preset (previous presets to be backed up)
 */
static long tsubCollSR (struct tsubRecord *pRec) {
        /* printf ("+++tsub=%s:  set=%5.2f\n",pRec->name,pRec->a); */
	if (pRec->a == 0) {				/* set-0 is requested */
	   if (pRec->oa != pRec->a) {			/* choice changed! */

		pRec->oa = pRec->a;			/* remember choice */

		pRec->ob0 = pRec->c0;			/* memorize presets of set-1 */
		pRec->ob1 = pRec->c1;
		pRec->ob2 = pRec->c2;
		pRec->ob3 = pRec->c3;
		pRec->ob4 = pRec->c4;
		pRec->ob5 = pRec->c5;
		pRec->ob6 = pRec->c6;
		pRec->ob7 = pRec->c7;
		pRec->ob8 = pRec->c8;
		pRec->ob9 = pRec->c9;

		pRec->oc0 = pRec->oa0;			/* load set-0 presets */
		pRec->oc1 = pRec->oa1;
		pRec->oc2 = pRec->oa2;
		pRec->oc3 = pRec->oa3;
		pRec->oc4 = pRec->oa4;
		pRec->oc5 = pRec->oa5;
		pRec->oc6 = pRec->oa6;
		pRec->oc7 = pRec->oa7;
		pRec->oc8 = pRec->oa8;
		pRec->oc9 = pRec->oa9;
	   }
	} else {                                        /* set-1 is requested */
	   if (pRec->oa != pRec->a) {			/* choice changed! */

		pRec->oa = pRec->a;			/* remember choice */

		pRec->oa0 = pRec->c0;                   /* memorize presets of set-0 */
		pRec->oa1 = pRec->c1;
		pRec->oa2 = pRec->c2;
		pRec->oa3 = pRec->c3;
		pRec->oa4 = pRec->c4;
		pRec->oa5 = pRec->c5;
		pRec->oa6 = pRec->c6;
		pRec->oa7 = pRec->c7;
		pRec->oa8 = pRec->c8;
		pRec->oa9 = pRec->c9;

		pRec->oc0 = pRec->ob0;                  /* load set-1 presets */
		pRec->oc1 = pRec->ob1;
		pRec->oc2 = pRec->ob2;
		pRec->oc3 = pRec->ob3;
		pRec->oc4 = pRec->ob4;
		pRec->oc5 = pRec->ob5;
		pRec->oc6 = pRec->ob6;
		pRec->oc7 = pRec->ob7;
		pRec->oc8 = pRec->ob8;
		pRec->oc9 = pRec->ob9;
	   }
	}
	return (0);
}

/* ===========================================
 *               Names registration
 *  =========================================== */
static registryFunctionRef tsubCollRef[] = {
    {"tsubColl",  (REGISTRYFUNCTION)tsubColl},
    {"tsubCollSR",(REGISTRYFUNCTION)tsubCollSR}
};

static void tsubCollFunc(void) {				/* declare this via registrar in DBD */
    registryFunctionRefAdd(tsubCollRef,NELEMENTS(tsubCollRef));
}
epicsExportRegistrar(tsubCollFunc);

