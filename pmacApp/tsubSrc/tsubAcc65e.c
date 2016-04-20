/* @(#) tsubACC65E.c 102 2005/03/26 */

/* tsubACC65E.c - Transformation Subroutines for PMAC Accessory-65E */
/*                Digital Output -- Sergey Stepanov    */

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

#define TSUB_DIAGNOSTIC tsubAcc65eDebug

#if TSUB_DIAGNOSTIC
volatile int TSUB_DIAGNOSTIC = 0;
#define TSUB_MESSAGE	logMsg
#define TSUB_TRACE(level,code)       { if ( (pRec->tpro == (level)) || (TSUB_DIAGNOSTIC == (level)) ) { code } }
#else
#define TSUB_TRACE(level,code)      ;
#endif

/*===========================================
 * tsubAcc65eIni - Accessory-65E Initialization
 */
static long tsubAcc65eIni (struct tsubRecord *pRec) {
	return (0);
}

/*===========================================
 * tsubAcc65eOut - Accessory-65E Digital Out word composition
 *	a0,...a9,b0,..b9,c1,...c3 = inputs
 *	oa = output
 */
static long tsubAcc65eOut (struct tsubRecord *	pRec) {
	long acc65e_out_long = 0;

	if ( pRec->a0 ) acc65e_out_long += 0x00000001;      /* IPA0 = bit0  */
	if ( pRec->a1 ) acc65e_out_long += 0x00000002;      /* IPA1 = bit1  */
	if ( pRec->a2 ) acc65e_out_long += 0x00000004;      /* IPA2 = bit2  */
	if ( pRec->a3 ) acc65e_out_long += 0x00000008;      /* IPA3 = bit3  */
	if ( pRec->a4 ) acc65e_out_long += 0x00000010;      /* IPA4 = bit4  */
	if ( pRec->a5 ) acc65e_out_long += 0x00000020;      /* IPA5 = bit5  */
	if ( pRec->a6 ) acc65e_out_long += 0x00000040;      /* IPA6 = bit6  */
	if ( pRec->a7 ) acc65e_out_long += 0x00000080;      /* IPA7 = bit7  */
	if ( pRec->a8 ) acc65e_out_long += 0x00000100;      /* IPA8 = bit8  */
	if ( pRec->a9 ) acc65e_out_long += 0x00000200;      /* IPA9 = bit9  */
	if ( pRec->b0 ) acc65e_out_long += 0x00000400;      /* IPB0 = bit10 */
	if ( pRec->b1 ) acc65e_out_long += 0x00000800;      /* IPB1 = bit11 */
	if ( pRec->b2 ) acc65e_out_long += 0x00001000;      /* IPB2 = bit12 */
	if ( pRec->b3 ) acc65e_out_long += 0x00002000;      /* IPB3 = bit13 */
	if ( pRec->b4 ) acc65e_out_long += 0x00004000;      /* IPB4 = bit14 */
	if ( pRec->b5 ) acc65e_out_long += 0x00008000;      /* IPB5 = bit15 */
	if ( pRec->b6 ) acc65e_out_long += 0x00010000;      /* IPB6 = bit16 */
	if ( pRec->b7 ) acc65e_out_long += 0x00020000;      /* IPB7 = bit17 */
	if ( pRec->b8 ) acc65e_out_long += 0x00040000;      /* IPB8 = bit18 */
	if ( pRec->b9 ) acc65e_out_long += 0x00080000;      /* IPB9 = bit19 */
	if ( pRec->c0 ) acc65e_out_long += 0x00100000;      /* IPC0 = bit20 */
	if ( pRec->c1 ) acc65e_out_long += 0x00200000;      /* IPC1 = bit21 */
	if ( pRec->c2 ) acc65e_out_long += 0x00400000;      /* IPC2 = bit22 */
	if ( pRec->c3 ) acc65e_out_long += 0x00800000;      /* IPC3 = bit23 */
	pRec->oa = acc65e_out_long;
	return (0);
}

/*===========================================
 * tsubAcc65eRbk - Accessory-65E Digital Out word decomposition
 *	a = input
 *	oa0,...oa9,ob0,..ob9,oc1,...oc3 = outputs
 */
static long tsubAcc65eRbk (struct tsubRecord *pRec) {
	long          acc65e_rbk_long = 0;
	unsigned long tmp = 0;
	char 	      binstr[32];
	short         i=0;

	acc65e_rbk_long = (long)pRec->a;

	for(i=31; i>=0; i--) {
/*
 *  "<<n" Left shift n places,
 *  ">>n" Right shift n places
*/
          tmp = acc65e_rbk_long/(1<<i);  		/* tmp=value/pow(2,i)*/
	  if(tmp > 0) {
   	    binstr[i] = '1';
            acc65e_rbk_long = acc65e_rbk_long%(1<<i);	/* value=value%pow(2,i) */
  	  } else {
   	    binstr[i] = '0';
	  }
 	}
	if ( binstr[0]  == '1' ) {pRec->oa0=1;} else {pRec->oa0=0;}      /* bit0  = OTA0 */
	if ( binstr[1]  == '1' ) {pRec->oa1=1;} else {pRec->oa1=0;}      /* bit1  = OTA1 */
	if ( binstr[2]  == '1' ) {pRec->oa2=1;} else {pRec->oa2=0;}      /* bit2  = OTA2 */
	if ( binstr[3]  == '1' ) {pRec->oa3=1;} else {pRec->oa3=0;}      /* bit3  = OTA3 */
	if ( binstr[4]  == '1' ) {pRec->oa4=1;} else {pRec->oa4=0;}      /* bit4  = OTA4 */
	if ( binstr[5]  == '1' ) {pRec->oa5=1;} else {pRec->oa5=0;}      /* bit5  = OTA5 */
	if ( binstr[6]  == '1' ) {pRec->oa6=1;} else {pRec->oa6=0;}      /* bit6  = OTA6 */
	if ( binstr[7]  == '1' ) {pRec->oa7=1;} else {pRec->oa7=0;}      /* bit7  = OTA7 */
	if ( binstr[8]  == '1' ) {pRec->oa8=1;} else {pRec->oa8=0;}      /* bit8  = OTA8 */
	if ( binstr[9]  == '1' ) {pRec->oa9=1;} else {pRec->oa9=0;}      /* bit9  = OTA9 */
	if ( binstr[10] == '1' ) {pRec->ob0=1;} else {pRec->ob0=0;}      /* bit10 = OTB0 */
	if ( binstr[11] == '1' ) {pRec->ob1=1;} else {pRec->ob1=0;}      /* bit11 = OTB1 */
	if ( binstr[12] == '1' ) {pRec->ob2=1;} else {pRec->ob2=0;}      /* bit12 = OTB2 */
	if ( binstr[13] == '1' ) {pRec->ob3=1;} else {pRec->ob3=0;}      /* bit13 = OTB3 */
	if ( binstr[14] == '1' ) {pRec->ob4=1;} else {pRec->ob4=0;}      /* bit14 = OTB4 */
	if ( binstr[15] == '1' ) {pRec->ob5=1;} else {pRec->ob5=0;}      /* bit15 = OTB5 */
	if ( binstr[16] == '1' ) {pRec->ob6=1;} else {pRec->ob6=0;}      /* bit16 = OTB6 */
	if ( binstr[17] == '1' ) {pRec->ob7=1;} else {pRec->ob7=0;}      /* bit17 = OTB7 */
	if ( binstr[18] == '1' ) {pRec->ob8=1;} else {pRec->ob8=0;}      /* bit18 = OTB8 */
	if ( binstr[19] == '1' ) {pRec->ob9=1;} else {pRec->ob9=0;}      /* bit19 = OTB9 */
	if ( binstr[20] == '1' ) {pRec->oc0=1;} else {pRec->oc0=0;}      /* bit20 = OTC0 */
	if ( binstr[21] == '1' ) {pRec->oc1=1;} else {pRec->oc1=0;}      /* bit21 = OTC1 */
	if ( binstr[22] == '1' ) {pRec->oc2=1;} else {pRec->oc2=0;}      /* bit22 = OTC2 */
	if ( binstr[23] == '1' ) {pRec->oc3=1;} else {pRec->oc3=0;}      /* bit23 = OTC3 */
	return (0);
}

/* ===========================================
 *               Names registration
 *  =========================================== */
static registryFunctionRef tsubAcc65eRef[] = {
    {"tsubAcc65eIni", (REGISTRYFUNCTION)tsubAcc65eIni},
    {"tsubAcc65eOut", (REGISTRYFUNCTION)tsubAcc65eOut},
    {"tsubAcc65eRbk", (REGISTRYFUNCTION)tsubAcc65eRbk}
};

static void tsubAcc65eFunc(void) {				/* declare this via registrar in DBD */
    registryFunctionRefAdd(tsubAcc65eRef,NELEMENTS(tsubAcc65eRef));
}
epicsExportRegistrar(tsubAcc65eFunc);

