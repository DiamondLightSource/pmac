#ifndef DRVPMACETHERNET_H
  #define DRVPMACETHERNET_H

  #define PMACETH_VERSION "1.0.7"

  #include <callback.h>
  #include <pmacRam.h>
  #include <pmacError.h>
  #include <waveformRecord.h>

  #define PMAC_MBX_OUT_BUFLEN	(80)
  #define PMAC_MBX_IN_BUFLEN	(80)
  #define PMAC_MBX_ERR_BUFLEN	(10)

  #define PMAC_ASC_OUT_BUFLEN	(160)
  #define PMAC_ASC_IN_BUFLEN	(256)

  #define PMAC_RECONNECT_TIME	2

  /* PMAC Hardware Constants */

  #define PMAC_TERM_ACK   0x06
  #define PMAC_TERM_BELL  0x07
  #define PMAC_TERM_CR    0x0D

  typedef struct {  /* PMAC_MBX_IO */
    struct dbCommon *pRec;
    int		    card;
    int		    terminator;
    int 	    status;
    int 	    init;
    char	    command[PMAC_MBX_OUT_BUFLEN];
    char	    response[PMAC_MBX_IN_BUFLEN];
    char	    errmsg[PMAC_MBX_ERR_BUFLEN];
    CALLBACK	    callback;
  } PMAC_MBX_IO;

  #ifndef vxWorks
    #define ERROR -1
    typedef void    (*VOIDFUNCPTR)(void *);
  #endif


  typedef struct {  /* PMAC_RAM_IO */
    int	        memType;
    int	        pmacAdr;
    int	        hostOfs;
    PMAC_DPRAM  *pAddress;
    int         valInt;
    double      valDouble;
    VOIDFUNCPTR pFunc;
    void        *pParm;
  } PMAC_RAM_IO;

  int drvPmacEthConfig (int card, char *pmacAddress);
  int drvPmacMbxScan (PMAC_MBX_IO *pAscIO);
  int drvPmacSendBuffer(int pmacCard, char *buffer, int bufferLength);
  int drvPmacEthInit();
  int drvPmacStartup (void);
  long drvPmacMemSpecParse (char *pmacAdrSpec, int *memType, int *pmacAdr);
  long drvPmacDpramRequest (short card, short pmacAdrOfs, char *pmacAdrSpec, void (*pFunc)(void *), void *pParm, PMAC_RAM_IO **ppRamIo);
  PMAC_LOCAL long drvPmacRamGetData(PMAC_RAM_IO *pRamIo);
  PMAC_LOCAL long drvPmacRamPutData(int Card, PMAC_RAM_IO *pRamIo);
  void drvPmacStrErr (waveformRecord *pRec);
#endif   /* DRVPMACETHERNET_H  */           

