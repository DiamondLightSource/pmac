/* @(#) pmacVme.h 1.5 97/05/06 */

/* pmacVme.h -  Turbo PMAC2-VME Ultralite Device Driver */

/*
 * Author:      Oleg A. Makarov
 * 		adapted from PMAC-VME device driver written by Thomas A. Coleman
 * Date:        2003/09/17
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

#ifndef __INCpmacVmeH
#define __INCpmacVmeH

/*
 * DEFINES
 */

#define PMAC_PRIVATE FALSE

#if PMAC_PRIVATE
#define PMAC_LOCAL LOCAL
#else
#define PMAC_LOCAL
#endif

/* PMAC Hardware Constants */

#define PMAC_TERM_ACK   0x06
#define PMAC_TERM_BELL  0x07
#define PMAC_TERM_CR    0x0D

#define PMAC_MAX_CTLRS	(8)			/* Maximum # of PMAC controller cards */

#define PMAC_MBX_OUT_BUFLEN	(80)
#define PMAC_MBX_IN_BUFLEN	(80)
#define PMAC_MBX_ERR_BUFLEN	(10)

/* #define WAIT_TIMEOUT	60 */		/* timeout in ticks for semTake */
#define WAIT_TIMEOUT	   3600		/* timeout in ticks for semTake -- changed 2005-05-09 */
#define PMAC_BASE_MBX_REGS (16)
#define PMAC_STRLEN_FWVER  (31)

/*
 * TYPEDEFS
 */

typedef volatile char	PMAC_DPRAM;
typedef volatile char	PMAC_DPRAM_BASE;

typedef struct  /* PMAC_MBX_BASE */
{
	struct pmacBaseMbxStruct
	{
		struct { unsigned char unused; unsigned char data; } MB[PMAC_BASE_MBX_REGS];
		
	} mailbox;
} PMAC_MBX_BASE;

typedef struct  /* PMAC_CTLR */
{
	int		ctlr;
	int		configured;
	int		present;
	int		enabled;
	int		active;
	int		presentBase;
	int		enabledBase;
	int		activeBase;
	int		presentDpram;
	int		enabledDpram;
	int		activeDpram;
	int		enabledGather;
	int		activeGather;
	PMAC_MBX_BASE   *pBase;
	PMAC_DPRAM_BASE *pDpramBase;
	unsigned	irqVector;
	unsigned	irqLevel;
	unsigned long	vmebusBase;
	unsigned long	vmebusDpram;
	SEM_ID		ioMbxLockSem;
	SEM_ID		ioMbxReceiptSem;
	SEM_ID		ioMbxReadmeSem;
	char		firmwareVersion[PMAC_STRLEN_FWVER];
} PMAC_CTLR;


/*
 * FORWARD DECLARATIONS
 */


void pmacVmeReport( int, int );
           long pmacVmeConfig( int, unsigned long, unsigned long, unsigned int, unsigned int );
PMAC_LOCAL long pmacVmeInit (void);
PMAC_LOCAL char pmacMbxWrite(int, char *);
PMAC_LOCAL char pmacMbxRead(int, char *, char *);
PMAC_LOCAL void pmacMbxReceiptISR (PMAC_CTLR * pPmacCtlr);
PMAC_LOCAL void pmacMbxReadmeISR (PMAC_CTLR * pPmacCtlr);

long pmacMbxLock (int ctlr);
long pmacMbxUnlock (int	ctlr);

char pmacVmeWriteC (char * addr, char val);
char pmacVmeReadC (char * addr);

PMAC_DPRAM * pmacRamAddr (int ctlr, int offset);

PMAC_LOCAL long pmacRamGet16 (PMAC_DPRAM * pRam, long * pVal);
PMAC_LOCAL long pmacRamPut16 (PMAC_DPRAM * pRam, long val);
PMAC_LOCAL long pmacRamGet24U (PMAC_DPRAM * pRam, long * pVal);
PMAC_LOCAL long pmacRamGet24 (PMAC_DPRAM * pRam, long * pVal);
PMAC_LOCAL long pmacRamPut32 (PMAC_DPRAM * pRam, long val);
PMAC_LOCAL long pmacRamGetF (PMAC_DPRAM * pRam, double * pVal);
PMAC_LOCAL long pmacRamPutF (PMAC_DPRAM * pRam, double val);
PMAC_LOCAL long pmacRamGetD (PMAC_DPRAM * pRam, double * pVal);
PMAC_LOCAL long pmacRamGetL (PMAC_DPRAM * pRam, double * pVal);

PMAC_LOCAL void pmacGatBufferSem (int ctlr);

#endif /* __INCpmacVmeH */
