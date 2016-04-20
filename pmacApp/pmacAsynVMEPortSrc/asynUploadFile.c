#include <stdio.h>
#include <string.h>
#include "asynDriver.h"
#include "asynOctetSyncIO.h"
#include <epicsExport.h>
#include <iocsh.h>

#define OK 0

#define TIMEOUT 6.0

int asynUploadFile( char * filename, char * asynPort, char * out_term, char * inp_term )
{
    FILE * fd;
    asynStatus status;
    asynUser * pasynUser;
    int linenum=0;
    char readbuf[255], writebuf[255];

    fd = fopen( filename, "r" );
    if (fd == NULL)
    {
        fprintf( stderr, "asynUploadFile: Cannot open file: %s\n", filename );
        return ERROR;
    }
 
    status = pasynOctetSyncIO->connect( asynPort, 0, &pasynUser, NULL);
    if (status) {
        printf( "asynUploadFile: unable to connect to asynPort %s\n", asynPort );
        fclose( fd );
        return ERROR;
    }

    status = pasynOctetSyncIO->setInputEos( pasynUser, inp_term, strlen(inp_term) );
    if (status) {
        printf( "pmacDrvConfig: unable to set input EOS on %s: %s\n", 
                asynPort, pasynUser->errorMessage);
        pasynOctetSyncIO->disconnect(pasynUser);
        fclose( fd );
        return ERROR;
    }

    status = pasynOctetSyncIO->setOutputEos( pasynUser, out_term, strlen(out_term) );
    if (status) {
        printf( "pmacDrvConfig: unable to set output EOS on %s: %s\n", 
                asynPort, pasynUser->errorMessage);
        pasynOctetSyncIO->disconnect(pasynUser);
        fclose( fd );
        return ERROR;
    }

    while ( fgets( writebuf, sizeof(writebuf), fd ) != NULL )
    {
        const double timeout=5.0;
        size_t nwrite, nread;
        int eomReason;
        size_t ls = strlen(writebuf);
        linenum++;

        if ( writebuf[ls-1] == '\n' ) ls--;

        status = pasynOctetSyncIO->writeRead( pasynUser,
                                              writebuf, ls,
                                              readbuf, sizeof(readbuf),
                                              timeout,
                                              &nwrite, &nread, &eomReason );

        if (status)
        {
            asynPrint( pasynUser,
                       ASYN_TRACE_ERROR,
                       "Error writing line %d to asyn port %s: command %s\n Status=%d, Error=%s\n",
                       linenum, asynPort, writebuf, status, pasynUser->errorMessage);
            fclose( fd );
            pasynOctetSyncIO->disconnect( pasynUser );
            return ERROR;
        }

        if ( nread != 0 )
        {
            readbuf[nread]=0;
            printf( "Received non-null response writing line %d to asyn port %s: \nCommand: %s\nResponse: %s\n",
                    linenum, asynPort, writebuf, readbuf );
        }
    }

    fclose( fd );
    pasynOctetSyncIO->disconnect( pasynUser );
    return OK;
}

static const iocshArg asynUploadFileArg0 = {"File name", iocshArgString};
static const iocshArg asynUploadFileArg1 = {"Asyn port", iocshArgString};
static const iocshArg asynUploadFileArg2 = {"Output terminator", iocshArgString};
static const iocshArg asynUploadFileArg3 = {"Input terminator",  iocshArgString};
static const iocshArg * const asynUploadFileArgs[] = {&asynUploadFileArg0,
                                                      &asynUploadFileArg1,
                                                      &asynUploadFileArg2,
                                                      &asynUploadFileArg3};
 
static const iocshFuncDef asynUploadFileDef = {"asynUploadFile", 4, asynUploadFileArgs};

static void asynUploadFileCallFunc(const iocshArgBuf *args)
{
    asynUploadFile(args[0].sval, args[1].sval, args[2].sval, args[3].sval );
}


static void asynUploadFileRegister(void)
{
    iocshRegister(&asynUploadFileDef,  asynUploadFileCallFunc);
}

epicsExportRegistrar(asynUploadFileRegister);

