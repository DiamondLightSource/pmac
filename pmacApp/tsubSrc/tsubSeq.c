/* @(#) tsubSeq.c 1.1 97/05/22 */

/* tsubSeq.c - Transformation Subroutines For Sequence Function */

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

#define TSUB_DIAGNOSTIC tsubSeqDebug

#if TSUB_DIAGNOSTIC
volatile int TSUB_DIAGNOSTIC = 0;
#define TSUB_MESSAGE	logMsg
#define TSUB_TRACE(level,code)  {if ((pRec->tpro == (level)) || (TSUB_DIAGNOSTIC == (level))) {code}}
#else
#define TSUB_TRACE(level,code);
#endif

/* ===========================================
 * tsubSeqInit - Initialization
 */
static long tsubSeqInit (struct tsubRecord *pRec) {
	return (0);
}

/* ===========================================
 * tsubSeqProc - Process
 */
static long tsubSeqProc (struct tsubRecord *pRec) {
	pRec->oa = pRec->a;
	pRec->ob = pRec->b;
	pRec->oc = pRec->c;
	pRec->od = pRec->d;
	pRec->oe = pRec->e;
	pRec->of = pRec->f;
	pRec->og = pRec->g;
	pRec->oh = pRec->h;
	pRec->oi = pRec->i;
	pRec->oj = pRec->j;

	pRec->oa0 = pRec->a0;
	pRec->oa1 = pRec->a1;
	pRec->oa2 = pRec->a2;
	pRec->oa3 = pRec->a3;
	pRec->oa4 = pRec->a4;
	pRec->oa5 = pRec->a5;
	pRec->oa6 = pRec->a6;
	pRec->oa7 = pRec->a7;
	pRec->oa8 = pRec->a8;
	pRec->oa9 = pRec->a9;

	pRec->ob0 = pRec->b0;
	pRec->ob1 = pRec->b1;
	pRec->ob2 = pRec->b2;
	pRec->ob3 = pRec->b3;
	pRec->ob4 = pRec->b4;
	pRec->ob5 = pRec->b5;
	pRec->ob6 = pRec->b6;
	pRec->ob7 = pRec->b7;
	pRec->ob8 = pRec->b8;
	pRec->ob9 = pRec->b9;

	pRec->oc0 = pRec->c0;
	pRec->oc1 = pRec->c1;
	pRec->oc2 = pRec->c2;
	pRec->oc3 = pRec->c3;
	pRec->oc4 = pRec->c4;
	pRec->oc5 = pRec->c5;
	pRec->oc6 = pRec->c6;
	pRec->oc7 = pRec->c7;
	pRec->oc8 = pRec->c8;
	pRec->oc9 = pRec->c9;

	pRec->od0 = pRec->d0;
	pRec->od1 = pRec->d1;
	pRec->od2 = pRec->d2;
	pRec->od3 = pRec->d3;
	pRec->od4 = pRec->d4;
	pRec->od5 = pRec->d5;
	pRec->od6 = pRec->d6;
	pRec->od7 = pRec->d7;
	pRec->od8 = pRec->d8;
	pRec->od9 = pRec->d9;

	pRec->oe0 = pRec->e0;
	pRec->oe1 = pRec->e1;
	pRec->oe2 = pRec->e2;
	pRec->oe3 = pRec->e3;
	pRec->oe4 = pRec->e4;
	pRec->oe5 = pRec->e5;
	pRec->oe6 = pRec->e6;
	pRec->oe7 = pRec->e7;
	pRec->oe8 = pRec->e8;
	pRec->oe9 = pRec->e9;

	pRec->of0 = pRec->f0;
	pRec->of1 = pRec->f1;
	pRec->of2 = pRec->f2;
	pRec->of3 = pRec->f3;
	pRec->of4 = pRec->f4;
	pRec->of5 = pRec->f5;
	pRec->of6 = pRec->f6;
	pRec->of7 = pRec->f7;
	pRec->of8 = pRec->f8;
	pRec->of9 = pRec->f9;

	return (0);
}

/* ===========================================
 *               Names registration
 *  =========================================== */
static registryFunctionRef tsubSeqRef[] = {
    {"tsubSeqInit",  (REGISTRYFUNCTION)tsubSeqInit},
    {"tsubSeqProc",  (REGISTRYFUNCTION)tsubSeqProc}
};

static void tsubSeqFunc(void) {				/* declare this via registrar in DBD */
    registryFunctionRefAdd(tsubSeqRef,NELEMENTS(tsubSeqRef));
}
epicsExportRegistrar(tsubSeqFunc);

