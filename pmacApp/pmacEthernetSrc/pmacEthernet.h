#ifndef PMACETHERNET_H
#define PMACETHERNET_H

#include <netdb.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>               
#include <fcntl.h>


#define PORT 1025 /* the port client will be connecting to */

#define MAXDATASIZE 100 /* max number of bytes we can get at once */



//----------------------------------------------------------------------------- 
// Constants Defined by USB 1.1 Spec 
//----------------------------------------------------------------------------- 
#define VR_DOWNLOAD      0x40 // command sent to device 
#define VR_UPLOAD        0xC0 // command sent to host 

//----------------------------------------------------------------------------- 
// Constants Defined by Quad 1 for implementing PMACUSB expansion card 
//----------------------------------------------------------------------------- 
#define VR_PMAC_SENDLINE     0xB0  // Device specific vendor commands
#define VR_PMAC_GETLINE      0xB1
#define VR_PMAC_FLUSH        0xB3
#define VR_PMAC_GETMEM       0xB4
#define VR_PMAC_SETMEM       0xB5
#define VR_PMAC_SENDCTRLCHAR 0xB6 
#define VR_PMAC_ENARTDATAGAT 0xB7 
#define VR_PMAC_INIT         0xB8 
#define VR_EEPROM_VIDDID     0xB9 
#define VR_PMAC_SETBIT       0xBA
#define VR_PMAC_SETBITS      0xBB
#define VR_PMAC_DATAPORT     0xBC 
#define VR_PMAC_STATUSPORT   0xBD 
#define VR_PMAC_PORT         0xBE
#define VR_PMAC_GETRESPONSE  0xBF

#define VR_PMAC_SETSERCHAR   0xC0 
#define VR_PMAC_GETSERCHAR   0xC1 
#define VR_PMAC_READREADY    0xC2
#define VR_PMAC_GETLINE_B    0xC3 
#define VR_CTRL_RESPONSE     0xC4
#define VR_PMAC_GETBUFFER    0xC5
#define VR_PMAC_WRITEBUFFER  0xC6
#define VR_PMAC_WRITEERROR   0xC7 /* no description in the "UMAC Turbo CPU/Communications Board Hardware Manual" */
#define VR_PMAC_BINROTPUT    0xC8 
#define VR_PMAC_BINROTERROR  0xC9 
#define VR_PMAC_BINROTINIT   0xCA 
#define VR_FWDOWNLOAD        0xCB

#define VR_PMAC_CLOSE        0xD0 

#define VR_IPADDRESS         0xE0
#define VR_MACADDRESS        0xE1 
#define VR_DATE              0xE2 
#define VR_TIME              0xE3 
#define VR_VERSION           0xE4 

#define ETH_COMM_TIMEOUT_SEC  5
#define ETH_COMM_TIMEOUT_USEC 0

typedef unsigned char BYTE;
typedef unsigned short WORD;


typedef struct tagEthernetCmd {
  BYTE RequestType;
  BYTE Request;
  WORD wValue;
  WORD wIndex;
  WORD wLength;
  BYTE bData[1492];
} ETHERNETCMD, *PETHERNETCMD;
      


#define FLUSH_TIMEOUT 10
#define ETH_CMD_SIZE   8


int  pmacSockFlush	 (int  sock,    fd_set *readfds, fd_set *writefds);
int  pmacSockSendLine	 (int  sock,    fd_set *readfds, fd_set *writefds, char *outstr);
int  pmacSockSetMem	 (int  sock,    fd_set *readfds, fd_set *writefds, char *outstr, short offset, short length);
int  pmacSockGetLine	 (int  sock,    fd_set *readfds, fd_set *writefds, char *instr);
int  pmacSockGetBuffer   (int  sock,    fd_set *readfds, fd_set *writefds, char *instr);
int  pmacSockGetMem	 (int  sock,    fd_set *readfds, fd_set *writefds, char *instr,  short offset, short length);
int  pmacSockSetBit	 (int  sock,    fd_set *readfds, fd_set *writefds, short offset, short on, short bitno);
int  pmacSockSetBits	 (int  sock,    fd_set *readfds, fd_set *writefds, short offset, int bts, int btc);
int  pmacSockPortGet	 (int  sock,    fd_set *readfds, fd_set *writefds, short offset, char *inch);
int  pmacSockPortPut	 (int  sock,    fd_set *readfds, fd_set *writefds, short offset, char outch);
int  pmacSockCtrlRsp	 (int  sock,    fd_set *readfds, fd_set *writefds, char  *outch, char *instr);
int  pmacSockFwDown	 (int  sock,    fd_set *readfds, fd_set *writefds, char bRestart, char *data, int len);
int  pmacSockReadReady   (int  sock,    fd_set *readfds, fd_set *writefds);
int  pmacSockGetIPAddress(int  sock,    fd_set *readfds, fd_set *writefds, unsigned char *ip);
int  pmacSockGetResponse (int  sock,    fd_set *readfds, fd_set *writefds, char *outstr, char *response);
int  pmacSockWriteBuffer (int  sock,    fd_set *readfds, fd_set *writefds, char *outBuffer, int bufferLength, char *response);
int  pmacSockOpen	 (char *ipaddr, char   *hostname, fd_set *readfds, fd_set *writefds);
void pmacSockClose       (int  sock);

#endif

