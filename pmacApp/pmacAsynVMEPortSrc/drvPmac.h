/* drvPmac.h -  EPICS Device Driver Support for Turbo PMAC2-VME Ultralite
 * Author       Oleg A. Makarov
 *              Thomas A. Coleman's PMAC-VME driver was used as a prototype 
 * Date:        2003/08/19
 *
 *      Experimental Physics and Industrial Control System (EPICS)
 */

/*
*****************************************************************
                          COPYRIGHT NOTIFICATION
*****************************************************************

THE FOLLOWING IS A NOTICE OF COPYRIGHT, AVAILABILITY OF THE CODE,
AND DISCLAIMER WHICH MUST BE INCLUDED IN THE PROLOGUE OF THE CODE
AND IN ALL SOURCE LISTINGS OF THE CODE.

(C)  COPYRIGHT 1995 UNIVERSITY OF CHICAGO

Argonne National Laboratory (ANL), with facilities in the States of
Illinois and Idaho, is owned by the United States Government, and
operated by the University of Chicago under provision of a contract
with the Department of Energy.

Portions of this material resulted from work developed under a U.S.
Government contract and are subject to the following license:  For
a period of five years from March 30, 1993, the Government is
granted for itself and others acting on its behalf a paid-up,
nonexclusive, irrevocable worldwide license in this computer
software to reproduce, prepare derivative works, and perform
publicly and display publicly.  With the approval of DOE, this
period may be renewed for two additional five year periods.
Following the expiration of this period or periods, the Government
is granted for itself and others acting on its behalf, a paid-up,
nonexclusive, irrevocable worldwide license in this computer
software to reproduce, prepare derivative works, distribute copies
to the public, perform publicly and display publicly, and to permit
others to do so.

*****************************************************************
                                DISCLAIMER
*****************************************************************

NEITHER THE UNITED STATES GOVERNMENT NOR ANY AGENCY THEREOF, NOR
THE UNIVERSITY OF CHICAGO, NOR ANY OF THEIR EMPLOYEES OR OFFICERS,
MAKES ANY WARRANTY, EXPRESS OR IMPLIED, OR ASSUMES ANY LEGAL
LIABILITY OR RESPONSIBILITY FOR THE ACCURACY, COMPLETENESS, OR
USEFULNESS OF ANY INFORMATION, APPARATUS, PRODUCT, OR PROCESS
DISCLOSED, OR REPRESENTS THAT ITS USE WOULD NOT INFRINGE PRIVATELY
OWNED RIGHTS.

*****************************************************************
LICENSING INQUIRIES MAY BE DIRECTED TO THE INDUSTRIAL TECHNOLOGY
DEVELOPMENT CENTER AT ARGONNE NATIONAL LABORATORY (708-252-2000).
*/

/*
 * Modification History:
 * ---------------------
 * .01  6-7-95        tac     initial
 */

#ifndef __INCdrvPmacH
#define __INCdrvPmacH

#include <pmacVme.h>
#include <pmacError.h>
#include <waveformRecord.h>

typedef struct  /* PMAC_MBX_IO */
{
	struct dbCommon *	pRec;
	int			card;
	long			terminator;
	char			command[PMAC_MBX_OUT_BUFLEN];
	char			response[PMAC_MBX_IN_BUFLEN];
	char			errmsg[PMAC_MBX_ERR_BUFLEN];
	CALLBACK		callback;
} PMAC_MBX_IO;

typedef struct  /* PMAC_RAM_IO */
{
	int		memType;
	int		pmacAdr;
	int		hostOfs;
  	PMAC_DPRAM *	pAddress;
	long		valLong;
	double		valDouble;
 	VOIDFUNCPTR	pFunc;
  	void *		pParm;
} PMAC_RAM_IO;

/* ---- Sergey ----- */
long pmacDrvConfig (int	cardNumber, int	scanMtrRate, int scanBkgRate, int scanVarRate,
#ifdef PMAC_ASYN
	char *		asynMbxPort
#else
	int		disableMbx
#endif
);
PMAC_LOCAL long drvPmac_report (int level);
PMAC_LOCAL long drvPmac_init (void);
PMAC_LOCAL long drvPmacStartup (void);
long drvPmacMemSpecParse (char *pmacAdrSpec, int *memType, int *pmacAdr);
long drvPmacVarSetup ( int card);
long drvPmacMtrRead ( int card);
long drvPmacBkgRead ( int card);
long drvPmacVarRead ( int card);
int pmacMtrShow ( int card, int	index);
int pmacBkgShow ( int card, int index);
int pmacVarShow ( int card, int	index);
int pmacOpnShow ( int card, int index);
char drvPmacMbxWriteRead ( int card, char *writebuf, char *readbuf, char *errmsg);
PMAC_LOCAL void drvPmacMbxScanInit(int card);
int mbxProcessTask(int card);
int mtrProcessTask(int card);
int bkgProcessTask(int card);
int varProcessTask(int card);
PMAC_LOCAL void drvPmacMtrScanInit(int card);
PMAC_LOCAL void drvPmacBkgScanInit(int card);
PMAC_LOCAL void drvPmacVarScanInit(int card);
long drvPmacDpramRequest (short card, short pmacAdrOfs, char *pmacAdrSpec, void (*pFunc)(void *), void *pParm, PMAC_RAM_IO **ppRamIo);
PMAC_LOCAL long drvPmacRamGetData(PMAC_RAM_IO *pRamIo);
PMAC_LOCAL long drvPmacRamPutData(PMAC_RAM_IO *pRamIo);
void drvPmacMbxScan (PMAC_MBX_IO *pMbxIo);
void drvPmacStrErr (waveformRecord *pRec);

#endif /* __INCdrvPmacH */
