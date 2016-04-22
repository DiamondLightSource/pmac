/* @(#) pmacSerial.c 1.5 96/07/19 */

/* pmacSerial.c -  PMAC Interactive via Serial Port */

/*
 * Author:      Thomas A. Coleman
 * Date:        96/07/19
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
 * .01  06-23-95        tac     initial
 */

/*
 * DESCRIPTION:
 * ------------
 * This module provides interactive access to PMAC via a serial port.
 *
 * INCLUDE FILES:
 *	
 */

/*
 * INCLUDES
 */

/* VxWorks Includes */

#include	<vxWorks.h>
#include 	<stdlib.h>	/* Sergey */
#include	<vxLib.h>
#include	<ioLib.h>
#include	<stdio.h>
#include	<string.h>
#include	<selectLib.h>

/* local includes */

/*
 * DEFINES
 */

#define PMAC_DIAGNOSTICS TRUE
#define PMAC_PRIVATE FALSE

#if PMAC_PRIVATE
#define PMAC_LOCAL LOCAL
#else
#define PMAC_LOCAL
#endif

#if PMAC_DIAGNOSTICS
#define PMAC_MESSAGE	printf
#define PMAC_DEBUG(level,code)       { if (pmacSerialDebug >= (level)) { code } }
#else
#define PMAC_DEBUG(level,code)      ;
#endif

#define NO_ERR_STATUS	(-1)

#define PMAC_DEFAULT_TTY	"/tyCo/1"
#define PMAC_DEFAULT_BAUD	38400

#define PMAC_MAX_COMMAND	(255)
#define PMAC_MAX_RESPONSE	(1024)

#define ACK	0x06
#define BELL	0x07
#define CR	0x0D
#define CONTROL_X	0x18
#define CONTROL_Z	0x1A

/*
 * TYPEDEFS
 */


/*
 * FORWARD DECLARATIONS
 */


/*
 * GLOBALS
 */

char * pmacSerialVersion = "@(#) pmacSerial.c 1.5 96/07/19";

#if PMAC_DIAGNOSTICS
volatile int pmacSerialDebug = 0;
#endif

/*
 * LOCALS
 */


/*******************************************************************************
 *
 * pmacSerial - provide interative access to PMAC via serial port
 *
 */
int pmacSerial
(
	char *	ttyDevice,
	int	ttyBaud
)
{
	char *	MyName	= "pmacSerial";
	char *	ttyDefault = PMAC_DEFAULT_TTY;
	int	i;
	int	status;
	int	bytesRead;
	int	bytesUnread;
	int	maxBytes;
	int	fdPmac;
	fd_set	fdset;
	int	valid;
	int	invalid;
	int	errnum;
	int	exitNow;
	FILE *	fpInput;
	FILE *	fpOutput;
	char *	commandBuffer;
	char *	responseBuffer;
	
	/* Open Standard Input */
	fpInput = fdopen (0, "r");
	if (fpInput == (FILE *) NULL)
	{
		printErr ("%s: Unable to open standard input\n", MyName);
		exit (ERROR);
	}

	/* Open Standard Output */
	fpOutput = fdopen (1, "a");
	if (fpOutput == (FILE *) NULL)
	{
		printErr ("%s: Unable to open standard output\n", MyName);
		exit (ERROR);
	}

	/* Open User-Specified or Default Serial Device */
	if (ttyDevice != (char *) NULL)
	{
		fdPmac = open (ttyDevice, O_RDWR, (int)NULL);
		if (fdPmac == ERROR)
		{
			printErr ("%s: Unable to open device %s\n",
					MyName, ttyDevice);
			exit (ERROR);
		}
	}
	else
	{
		fdPmac = open (ttyDefault, O_RDWR, (int)NULL);
		if (fdPmac == ERROR)
		{
			printErr ("%s: Unable to open default device %s\n",
					MyName, ttyDefault);
			exit (ERROR);
		}
	}
	
	/* Set Serial TTY Options */
	status = ioctl (fdPmac, FIOSETOPTIONS, OPT_RAW);
	if (status == ERROR)
	{
		printErr ("%s: Unable to set device options\n", MyName);
		exit (status);
	}

	/* Set User-Specified or Default Baud Rate */
	if (ttyBaud != 0)
	{
		status = ioctl (fdPmac, FIOBAUDRATE, ttyBaud);
		if (status == ERROR)
		{
			printErr ("%s: Unable to set baudrate %d\n",
					MyName, ttyBaud);
			exit (status);
		}
	}
	else
	{
		status = ioctl (fdPmac, FIOBAUDRATE, PMAC_DEFAULT_BAUD);
		if (status == ERROR)
		{
			printErr ("%s: Unable to set default baudrate\n",
					MyName, PMAC_DEFAULT_BAUD);
			exit (status);
		}
	}
	
	/* Flush Input And Output Buffers */
	status = ioctl (fdPmac, FIOFLUSH, 0);
	if (status == ERROR)
	{
		printErr ("%s: Unable to flush buffers\n", MyName);
		exit (status);
	}

	/* Allocate String Buffers */
	commandBuffer = (char *) malloc (PMAC_MAX_COMMAND);
	responseBuffer = (char *) malloc (PMAC_MAX_RESPONSE);
	
	exitNow = FALSE;

	/* Get Command String */
	status = (int) fgets (commandBuffer, PMAC_MAX_COMMAND, fpInput);
	if (status == (int)NULL)
	{
		exitNow = TRUE;
	}

	/* Loop Until Exit */
	while ( !exitNow )
	{

		/* Write String To PMAC Prefixed By Control-Z */
		status = fdprintf (fdPmac, "%c%s\r", CONTROL_Z, commandBuffer);
		if (status == ERROR)
		{
			printErr ("%s: Error during ouptut\n", MyName);
			return (status);
		}
		
		/* Loop Until Response Is Acknowledged */
		valid = FALSE;
		invalid = FALSE;
		errnum = FALSE;
		while ( !valid && !(invalid && errnum) )
		{
		
			/* Clear File Descriptor Set */
			FD_ZERO (&fdset);
		
			/* Loop Until We Have Data Available */
			while ( !FD_ISSET (fdPmac, &fdset) )
			{

				/* Pend Until Device Has Data */
				FD_SET (fdPmac, &fdset);
				status = select (FD_SETSIZE, &fdset, NULL, NULL, NULL);
		
				if (status == ERROR)
				{
					printErr ("%s: Error occured during select\n", MyName);
					return (status);
				}
				else if (status == 0)
				{
					printErr ("%s: Timed out\n", MyName);
				}
			}

			/* Determine Number Of Unread Bytes */
			status = ioctl (fdPmac, FIONREAD, (int) &bytesUnread);
			if (status == ERROR)
			{
				printErr ("%s: Error determining number of bytes\n",
					MyName);
			}
		
			/* Read Bytes */
			maxBytes = min (bytesUnread, PMAC_MAX_RESPONSE);
			if (maxBytes == 0)
			{
				printErr ("%s: maxBytes = %d\n", MyName, maxBytes);
			}
			bytesRead = read (fdPmac, responseBuffer, maxBytes);
			if (bytesRead == ERROR)
			{
				printErr ("%s: Error occured during read\n", MyName);
				return (bytesRead);
			}
			else if (bytesRead == 0)
			{
				printErr ("%s: End of File\n", MyName);
			}

			/* Write Response To Standard Out */
			for (i=0; i<bytesRead; i++)
			{
				if ( responseBuffer[i] == ACK )
				{
					valid = TRUE;
				}
				else if ( responseBuffer[i] == BELL )
				{
					fprintf (fpOutput, "[");
					invalid = TRUE;
				}
				else if ( responseBuffer[i] == CR )
				{
					if (invalid)
					{
						fprintf (fpOutput, "]\n");
						errnum = TRUE;
					}
					else
					{
						fprintf (fpOutput, "\n");
					}
				}
				else
				{
					fprintf (fpOutput, "%c", responseBuffer[i]);
				}
			}
		}
		
		/* Command Has Been Acknowledged */

		/* Get Next Command */
		status = (int) fgets (commandBuffer, PMAC_MAX_COMMAND, fpInput);
		if (status == (int)NULL)
		{
			exitNow = TRUE;
		}

	}
	
	/* Stop On End Of File */
	
	status = fclose (fpInput);
	status = fclose (fpOutput);
	status = close (fdPmac);
	if (status == ERROR)
	{
		printErr ("%s: Unable to close device\n", MyName);
		exit (status);
	}

	return (0);
}
