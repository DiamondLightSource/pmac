#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <netdb.h>
#include <netinet/in.h>
#include <sys/socket.h>

#include <pthread.h>

#include "pmacEthernet.h"

/*
 * Define external references.
 */
 
extern int errno;

pthread_mutex_t  ethMutex;

/*
Sends ctrl-x character to PMAC which flushes the PMAC communication buffer. 

Returns: -1 on failure
	  1 on success

VR_PMAC_FLUSH packet issues a ^X to the UMAC Turbo CPU/Communications Board and waits up to 10 msec for
UMAC Turbo CPU/Communications Board to respond with a ^X. command. Set up the packet that is
sent as follows. One byte is returned upon successful completion of the command. 
*/
int pmacSockFlush (int sock, fd_set *readfds, fd_set *writefds) {
  ETHERNETCMD EthCmd;
  int sv, rv;
  struct timeval tv;

  tv.tv_sec = ETH_COMM_TIMEOUT_SEC;
  tv.tv_usec = ETH_COMM_TIMEOUT_USEC;
      
  EthCmd.RequestType = VR_DOWNLOAD;
  EthCmd.Request = VR_PMAC_FLUSH;
  EthCmd.wValue = htons (FLUSH_TIMEOUT);
  EthCmd.wIndex = 0;
  EthCmd.wLength = 0;
  pthread_mutex_lock (&ethMutex);
  select (sock + 1, NULL, writefds, NULL, &tv);
  if (FD_ISSET (sock, writefds)) {
    sv = send (sock, (char *) &EthCmd, ETH_CMD_SIZE, 0);
    if (sv == -1) {
      pthread_mutex_unlock (&ethMutex);
      printf ("pmacSockFlush: Send error!\n");
      return -1;
    }
  } else {
    pthread_mutex_unlock (&ethMutex);
    printf ("Timeout while waiting for eth to send\n");
    return -1;
  }
  select (sock + 1, readfds, NULL, NULL, &tv);
  if (FD_ISSET (sock, readfds)) {
    rv = recv (sock, (char *) &EthCmd, 200, 0);
    pthread_mutex_unlock (&ethMutex);
    if (rv == -1) {
      printf ("pmacSockFlush: Receive error!\n");
    }
    return rv;
  } else {
    pthread_mutex_unlock (&ethMutex);
    printf ("Timeout while waiting for response from pmac\n");
    return -1;
  }
}


/*
  Sends a null-terminated string to PMAC. The string should not be terminated with \r
  
  Returns: -1 on failure
  	    1 on success

VR_PMAC_SENDLINE packet causes the string NULL terminated in EthCmd.bData to be sent to the UMAC Turbo
CPU/Communications Board. The string should not be terminated with a carriage return as this is done
by the firmware. One byte will be returned upon successful completion of the command.
*/
int pmacSockSendLine (int sock, fd_set *readfds, fd_set *writefds, char *outstr) {
  ETHERNETCMD EthCmd;
  int sv, rv;
  struct timeval tv;
  
  tv.tv_sec = ETH_COMM_TIMEOUT_SEC;
  tv.tv_usec = ETH_COMM_TIMEOUT_USEC;
      
  EthCmd.RequestType = VR_DOWNLOAD;
  EthCmd.Request = VR_PMAC_SENDLINE;
  EthCmd.wValue = 0;
  EthCmd.wIndex = 0;
  EthCmd.wLength = htons ((WORD) strlen (outstr));
  memcpy (EthCmd.bData, outstr, strlen (outstr));
  pthread_mutex_lock (&ethMutex);
  select (sock + 1, NULL, writefds, NULL, &tv);
  if (FD_ISSET (sock, writefds)) {
    sv = send (sock, (char *) &EthCmd,  ETH_CMD_SIZE + strlen (outstr), 0);
    if (sv == -1) {
      pthread_mutex_unlock (&ethMutex);
      printf ("pmacSockSendLine: Send error\n");
      return -1;
    }
  } else {
    pthread_mutex_unlock (&ethMutex);
    printf ("Timeout while waiting for eth to send\n");
    return -1;
  }

  select (sock + 1, readfds, NULL, NULL, &tv);
  if (FD_ISSET (sock, readfds)) {
    rv = recv (sock, (char *) &EthCmd, 1, 0);
    pthread_mutex_unlock (&ethMutex);
  
    if (rv == -1) {
      printf ("pmacSockSendLine: Receive error\n");
    }
    return rv;
  } else {
    pthread_mutex_unlock (&ethMutex);
    /* timeout*/
    printf ("Timeout while waiting for response from pmac\n");
    return -1;
  }
}      

/*
VR_PMAC_SETMEM packet causes the Ethernet connection to write data to the DPRAM shared between the UMAC
Turbo CPU/Communications Board and the host port. Up to 1400 bytes may be written in a single
packet. The wValue field contains the byte offset to write the data to while the wLength parameter
indicates how many bytes to write. After sending the packet, the programmer must wait to receive one
byte via the Recv function before continuing. The data received is irrelevant; its purpose is to ensure that
the sender's command was received.
*/
int pmacSockSetMem (int sock, fd_set *readfds, fd_set *writefds, char *outstr, short int offset, short int length)
{
  ETHERNETCMD EthCmd;
  int sv, rv;
  struct timeval tv;
    
  tv.tv_sec = ETH_COMM_TIMEOUT_SEC;
  tv.tv_usec = ETH_COMM_TIMEOUT_USEC;
      
  EthCmd.RequestType = VR_UPLOAD;
/*EthCmd.RequestType = VR_DOWNLOAD;*/
  EthCmd.Request = VR_PMAC_SETMEM;
  EthCmd.wValue = htons (offset);
  EthCmd.wIndex = 0;
  EthCmd.wLength = htons((WORD) length);

  memcpy (EthCmd.bData, outstr, length);
  pthread_mutex_lock (&ethMutex);
  select (sock + 1, NULL, writefds, NULL, &tv);
  if (FD_ISSET (sock, writefds)) {
    sv = send (sock, (char *) &EthCmd,  ETH_CMD_SIZE + length, 0);
    if (sv == -1) {
      pthread_mutex_unlock (&ethMutex);
      printf ("pmacSockSetMem: Send error\n");
      return -1;
    }
  } else {
    pthread_mutex_unlock (&ethMutex);
    printf ("Timeout while waiting for eth to send\n");
    return -1;
  }

  select (sock + 1, readfds, NULL, NULL, &tv);
  if (FD_ISSET (sock, readfds)) {
    rv = recv (sock, (char *) &EthCmd, 1, 0);
    pthread_mutex_unlock (&ethMutex);
  
    if (rv == -1) {
      printf ("pmacSockSetMem: Receive error\n");
    }
    return rv;
  } else {
    pthread_mutex_unlock (&ethMutex);
    /* timeout*/
    printf ("Timeout while waiting for response from pmac\n");
    return -1;
  }
}

/*
	Fetches any available string residing in PMAC. This function will only
	retreive one line. To retreive multiple lines at once use pmacSockGetBuffer.
	
	Returns: -1 on failure
		 number of characters retreived if it was successfull

VR_PMAC_GETLINE packet causes the Ethernet connection to return any available string that may be residing in the
UMAC Turbo CPU/Communications Board. All characters up to a <CR>, <ACK> or <LF> are returned.
The available string in UMAC Turbo CPU/Communications Board is returned and captured via an
Ethernet recv command. Do not use this function. Use VR_PMAC_GETBUFFER instead, as this
function will retrieve multiple lines and enhance performance instead of using multiple calls of
VR_PMAC_GETLINE.
*/

int pmacSockGetLine (int sock, fd_set *readfds, fd_set *writefds, char *instr) {
  ETHERNETCMD EthCmd;
  int sv, rv;
  struct timeval tv;
    
  tv.tv_sec = ETH_COMM_TIMEOUT_SEC;
  tv.tv_usec = ETH_COMM_TIMEOUT_USEC;
  
  EthCmd.RequestType = VR_UPLOAD;
  EthCmd.Request = VR_PMAC_GETLINE;
  EthCmd.wValue = 0;
  EthCmd.wIndex = 0;
  pthread_mutex_lock (&ethMutex);
  select (sock + 1, NULL, writefds, NULL, &tv);
  if (FD_ISSET (sock, writefds)) {
    sv = send (sock, (char *) &EthCmd, ETH_CMD_SIZE, 0);
    if (sv == -1) {
      pthread_mutex_unlock (&ethMutex);
      printf ("pmacSockGetLine: Send error!\n");
      return -1;
    }
  } else {
    pthread_mutex_unlock (&ethMutex);
    printf ("Timeout while waiting for eth to send\n");
    return -1;
  }
  
  select (sock + 1, readfds, NULL, NULL, &tv);
  if (FD_ISSET (sock, readfds)) {
    rv = recv (sock, instr, 255, 0);
    pthread_mutex_unlock (&ethMutex);
    if (rv == -1) {
      printf ("pmacSockGetLine: Receive error!\n");
    }
    return rv;
  } else {
    pthread_mutex_unlock (&ethMutex);
    printf ("Timeout while waiting for response from pmac\n");
    return -1;
  }
}

/*
	Fetches any available string from PMAC. All characters up to an <ACK> or <LF> will be returned.
	Maximum number of characters retreived at once is 1400. The caller is responsible to determine
	if there are more characters to return (check for termination characters in retreived data)
	
	Returns: -1 on failure
		 number of characters retreived on success

VR_PMAC_GETBUFFER packet causes the Ethernet connection to return any available string that may be residing in the
PMAC. All characters up to an <ACK> or <LF> are returned. If a <BEL> or <STX> character is
detected, only the data up to the next <CR> is returned. The maximum amount of data returned is 1400
Bytes. It is the caller\u2019s responsibility to determine if there is more data to follow and to call
VR_PMAC_GETBUFFER again to retrieve all of the data available.
*/
int pmacSockGetBuffer (int sock, fd_set *readfds, fd_set *writefds, char *instr) {
  ETHERNETCMD EthCmd;
  int sv, rv;
  struct timeval tv;
    
  tv.tv_sec = ETH_COMM_TIMEOUT_SEC;
  tv.tv_usec = ETH_COMM_TIMEOUT_USEC;
        
        
  EthCmd.RequestType = VR_UPLOAD;
  EthCmd.Request = VR_PMAC_GETBUFFER;
  EthCmd.wValue = 0;
  EthCmd.wIndex = 0;
  pthread_mutex_lock (&ethMutex);
  select (sock + 1, NULL, writefds, NULL, &tv);
  if (FD_ISSET (sock, writefds)) {
    sv = send (sock, (char *) &EthCmd, ETH_CMD_SIZE, 0);
    if (sv == -1) {
      pthread_mutex_unlock (&ethMutex);
      printf ("pmacSockGetBuffer: Send error!\n");
      return -1;
    }
  } else {
    pthread_mutex_unlock (&ethMutex);
    printf ("Timeout while waiting for eth to send\n");
    return -1;
  }

  select (sock + 1, readfds, NULL, NULL, &tv);
  if (FD_ISSET (sock, readfds)) {
    rv = recv (sock, instr, 1400, 0);
    pthread_mutex_unlock (&ethMutex);
    if  (rv == -1) {
      printf ("pmacSockGetBuffer: Receive error!\n");
      return -1;
    }
    return rv;
  } else {
    pthread_mutex_unlock (&ethMutex);
    printf ("Timeout while waiting for response from pmac\n");
    return -1;
  }
    
}

/*
VR_PMAC_GETMEM packet causes the Ethernet connection to retrieve DPRAM data from the UMAC Turbo
CPU/Communications Board. Up to 1400 bytes may be received in a single packet. The wValue field
contains the byte offset to retrieve the data from, while the wLength parameter indicates how many bytes
to receive.
*/
int pmacSockGetMem (int sock, fd_set *readfds, fd_set *writefds, char *instr, short offset, short length)
{
  ETHERNETCMD EthCmd;
  int sv, rv;
  struct timeval tv;
    
  tv.tv_sec = ETH_COMM_TIMEOUT_SEC;
  tv.tv_usec = ETH_COMM_TIMEOUT_USEC;
        
  EthCmd.RequestType = VR_UPLOAD;
  EthCmd.Request = VR_PMAC_GETMEM;
  EthCmd.wValue = htons(offset);
  EthCmd.wIndex = 0;
  EthCmd.wLength = htons(length);

  pthread_mutex_lock (&ethMutex);
  select (sock + 1, NULL, writefds, NULL, &tv);
  if (FD_ISSET (sock, writefds)) {
    sv = send (sock, (char *) &EthCmd, ETH_CMD_SIZE, 0);
    if (sv == -1) {
      pthread_mutex_unlock (&ethMutex);
      printf ("pmacSockGetMem: Send error!\n");
      return -1;
    }
  } else {
    pthread_mutex_unlock (&ethMutex);
    printf ("Timeout while waiting for eth to send\n");
    return -1;
  }

  select (sock + 1, readfds, NULL, NULL, &tv);
  if (FD_ISSET (sock, readfds)) {
    rv = recv (sock, instr, 1400, 0);
    pthread_mutex_unlock (&ethMutex);
    if  (rv == -1) {
      printf ("pmacSockGetMem: Receive error!\n");
      return -1;
    }
    return rv;
  } else {
    pthread_mutex_unlock (&ethMutex);
    printf ("Timeout while waiting for response from pmac\n");
    return -1;
  }
    
}

/*
VR_PMAC_SETBIT packet causes the Ethernet connection to perform a write to the DPRAM shared between the PMAC
and the PMAC that either sets bits in a 32-bit word or clears bits in a 32-bit word. If the wIndex
parameter is supplied with a 1, a logical or is performed that sets bits. If it is 0, a logical AND is
performed, which clears bits. It is the programmer's responsibility to use the appropriate mask for setting
or clearing bits. The wValue field contains the byte offset to retrieve the data. After sending the packet,
the programmer must wait to receive one byte via the Recv function before continuing. The data received
is irrelevant; its purpose is to ensure that the sender's command was received.
*/
int pmacSockSetBit(int sock, fd_set *readfds, fd_set *writefds, short offset, short on, short bitno)
{
  ETHERNETCMD EthCmd;
  int rv, sv;
  struct timeval tv;
  int mask = 0x00000001;
  
  tv.tv_sec = ETH_COMM_TIMEOUT_SEC;
  tv.tv_usec = ETH_COMM_TIMEOUT_USEC;
  EthCmd.RequestType = VR_DOWNLOAD;
  EthCmd.Request = VR_PMAC_SETBIT;
  EthCmd.wValue = htons(offset);
  EthCmd.wIndex = htons(on);
  EthCmd.wLength = htons(4);
// generate the mask
  mask <<= bitno; // zero based
// If clearing a bit compliment mask to prepare the firmware for AND
  if(!on)
    mask = ~mask;
  memcpy(EthCmd.bData,&mask,4);
// Send command request
  pthread_mutex_lock (&ethMutex);
  select (sock + 1, NULL, writefds, NULL, &tv);
  if (FD_ISSET (sock, writefds)) {
    sv = send (sock, (char *) &EthCmd, ETH_CMD_SIZE + 4, 0);
    if (sv == -1) {
      pthread_mutex_unlock (&ethMutex);
      printf ("pmacSockSetBit: Send error!\n");
      return -1;
    }
  } else {
    pthread_mutex_unlock (&ethMutex);
    printf ("Timeout while waiting for eth to send\n");
    return -1;
  }

  select (sock + 1, readfds, NULL, NULL, &tv);
  if (FD_ISSET (sock, readfds)) {
    rv = recv (sock, (char *) &EthCmd, 1, 0);
    pthread_mutex_unlock (&ethMutex);
    if  (rv == -1) {
      printf ("pmacSockSetBit: Receive error!\n");
      return -1;
    }
    return rv;
  } else {
    pthread_mutex_unlock (&ethMutex);
    printf ("Timeout while waiting for response from pmac\n");
    return -1;
  }
}

/*
VR_PMAC_SETBITS packet causes the Ethernet connection to perform a write to the DPRAM shared between the PMAC
and the PMAC that sets bits in a 32-bit word to a new value. The wValue field contains the byte offset to
retrieve the data. The bData field of the Ethernet command packet must be stuffed with a mask
indicating which bits to set in four bytes followed by four bytes that indicate the bits to clear in a 32-bit
word. After sending the packet, the programmer must wait to receive one byte via the Recv function
before continuing. The data received is irrelevant; its purpose is to ensure that the sender's command was
received.
*/
int pmacSockSetBits(int sock, fd_set *readfds, fd_set *writefds, short offset, int bts, int btc)
{
  ETHERNETCMD EthCmd;
  int rv, sv;
  struct timeval tv;
  
  tv.tv_sec = ETH_COMM_TIMEOUT_SEC;
  tv.tv_usec = ETH_COMM_TIMEOUT_USEC;
  EthCmd.RequestType = VR_DOWNLOAD;
  EthCmd.Request = VR_PMAC_SETBITS;
  EthCmd.wValue = htons(offset);
  EthCmd.wIndex = 0;
  EthCmd.wLength = htons(8);
  memcpy(EthCmd.bData,&bts,4);
  memcpy(EthCmd.bData + 4,&btc,4);
// Send command request
  pthread_mutex_lock (&ethMutex);
  select (sock + 1, NULL, writefds, NULL, &tv);
  if (FD_ISSET (sock, writefds)) {
    sv = send (sock, (char *) &EthCmd, ETH_CMD_SIZE + 8, 0);
    if (sv == -1) {
      pthread_mutex_unlock (&ethMutex);
      printf ("pmacSockSetBits: Send error!\n");
      return -1;
    }
  } else {
    pthread_mutex_unlock (&ethMutex);
    printf ("Timeout while waiting for eth to send\n");
    return -1;
  }

  select (sock + 1, readfds, NULL, NULL, &tv);
  if (FD_ISSET (sock, readfds)) {
    rv = recv (sock, (char *) &EthCmd, 1, 0);
    pthread_mutex_unlock (&ethMutex);
    if  (rv == -1) {
      printf ("pmacSockSetBits: Receive error!\n");
      return -1;
    }
    return rv;
  } else {
    pthread_mutex_unlock (&ethMutex);
    printf ("Timeout while waiting for response from pmac\n");
    return -1;
  }
}

/*
VR_PMAC_PORT packet sends or receives a single byte to or from the UMAC Turbo CPU/Communications Board.
To send data to the host port, set the packet as follows. After sending the packet, the programmer must
wait to receive one byte via the Recv function before continuing. The data received is irrelevant; its
purpose is to ensure that the sender's command was received.
*/
int pmacSockPortPut (int sock, fd_set *readfds, fd_set *writefds, short offset, char outch)
{
  ETHERNETCMD EthCmd;
  int rv, sv;
  struct timeval tv;
  
  tv.tv_sec = ETH_COMM_TIMEOUT_SEC;
  tv.tv_usec = ETH_COMM_TIMEOUT_USEC;
  EthCmd.RequestType = VR_DOWNLOAD;
  EthCmd.Request = VR_PMAC_PORT;
  EthCmd.wValue = htons (offset);
  EthCmd.wIndex = htons ((short) outch);
  EthCmd.wLength = 0;
// Send command request
  pthread_mutex_lock (&ethMutex);
  select (sock + 1, NULL, writefds, NULL, &tv);
  if (FD_ISSET (sock, writefds)) {
    sv = send (sock, (char *) &EthCmd, ETH_CMD_SIZE, 0);
    if (sv == -1) {
      pthread_mutex_unlock (&ethMutex);
      printf ("pmacSockPortPut: Send error!\n");
      return -1;
    }
  } else {
    pthread_mutex_unlock (&ethMutex);
    printf ("Timeout while waiting for eth to send\n");
    return -1;
  }

  select (sock + 1, readfds, NULL, NULL, &tv);
  if (FD_ISSET (sock, readfds)) {
    rv = recv (sock, (char *) &EthCmd, 1, 0);
    pthread_mutex_unlock (&ethMutex);
    if  (rv == -1) {
      printf ("pmacSockPortPut: Receive error!\n");
      return -1;
    }
    return rv;
  } else {
    pthread_mutex_unlock (&ethMutex);
    printf ("Timeout while waiting for response from pmac\n");
    return -1;
  }
}

/*
To receive data from the host port, set the packet as follows. After sending the packet, the programmer
shall receive one byte which is the value the UMAC Turbo CPU/Communications Board read from the
host port.
*/
int pmacSockPortGet (int sock, fd_set *readfds, fd_set *writefds, short offset, char *inch)
{
  ETHERNETCMD EthCmd;
  int rv, sv;
  struct timeval tv;
  
  tv.tv_sec = ETH_COMM_TIMEOUT_SEC;
  tv.tv_usec = ETH_COMM_TIMEOUT_USEC;
  EthCmd.RequestType = VR_UPLOAD;
  EthCmd.Request = VR_PMAC_PORT;
  EthCmd.wValue = htons (offset);
  EthCmd.wIndex = 0;
  EthCmd.wLength = 0;
// Send command request
  pthread_mutex_lock (&ethMutex);
  select (sock + 1, NULL, writefds, NULL, &tv);
  if (FD_ISSET (sock, writefds)) {
    sv = send(sock, (char *) &EthCmd, ETH_CMD_SIZE, 0);
    if (sv == -1) {
      pthread_mutex_unlock (&ethMutex);
      printf ("pmacSockPortGet: Send error!\n");
      return -1;
    }
  } else {
    pthread_mutex_unlock (&ethMutex);
    printf ("Timeout while waiting for eth to send\n");
    return -1;
  }

  select (sock + 1, readfds, NULL, NULL, &tv);
  if (FD_ISSET (sock, readfds)) {
    rv = recv(sock, inch, 1, 0);
    pthread_mutex_unlock (&ethMutex);
    if  (rv == -1) {
      printf ("pmacSockPortGet: Receive error!\n");
      return -1;
    }
    return rv;
  } else {
    pthread_mutex_unlock (&ethMutex);
    printf ("Timeout while waiting for response from pmac\n");
    return -1;
  }
}

/*
VR_CTRL_REPONSE After sending a control character, this packet obtains the response. Set up the packet as follows. The
received data is the response to the sent control character. Meaningful data is returned for the following
control characters ^B, ^C, ^F, ^G, ^P and ^V. All other data control characters do not return meaningful
data.
*/
int pmacSockCtrlRsp (int sock, fd_set *readfds, fd_set *writefds, char *outch, char *instr)
{
  ETHERNETCMD EthCmd;
  int rv, sv;
  struct timeval tv;
  
  EthCmd.RequestType = VR_UPLOAD;
  EthCmd.Request = VR_CTRL_RESPONSE;
  EthCmd.wValue = htons ((short) *outch);//outch=ctrl char to send out
  EthCmd.wIndex = 0;
  EthCmd.wLength = 0;
  pthread_mutex_lock (&ethMutex);
  select (sock + 1, NULL, writefds, NULL, &tv);
  if (FD_ISSET (sock, writefds)) {
    sv = send (sock, ((char *) &EthCmd), ETH_CMD_SIZE, 0);
    if (sv == -1) {
      pthread_mutex_unlock (&ethMutex);
      printf ("pmacSockCtrlRsp: Send error!\n");
      return -1;
    }
  } else {
    pthread_mutex_unlock (&ethMutex);
    printf ("Timeout while waiting for eth to send\n");
    return -1;
  }

  select (sock + 1, readfds, NULL, NULL, &tv);
  if (FD_ISSET (sock, readfds)) {
    rv = recv (sock, instr, 1400, 0); // returned data appears
    pthread_mutex_unlock (&ethMutex);
    if  (rv == -1) {
      printf ("pmacSockCtrlRsp: Receive error!\n");
      return -1;
    }
    return rv;
  } else {
    pthread_mutex_unlock (&ethMutex);
    printf ("Timeout while waiting for response from pmac\n");
    return -1;
  }
}

/*
VR_FWDOWNLOAD packet writes raw data to the UMAC Turbo CPU/Communications Board host port for firmware
download. The firmware takes the stream of data, and then writes to the UMAC Turbo
CPU/Communications Board host port at address 5, 6 and 7. The packet includes the wValue parameter
that commands the start the download at host port address 5. This packet writes multiple lines to the
UMAC Turbo CPU/Communications Board with just one packet. The packet is set up as follows. The
received data is the response to the sent control character. Usually, it is used for downloading a file. Data
should have each line separated by null byte. After sending the packet, the programmer must wait to
receive one byte via the recv function before continuing. The data received is irrelevant; its purpose is to
ensure that the sender's command was received.
*/
int pmacSockFwDown (int sock, fd_set *readfds, fd_set *writefds, char bRestart, char *data, int len)
{
  ETHERNETCMD EthCmd;
  int rv, sv;
  struct timeval tv;
  
  EthCmd.RequestType = VR_DOWNLOAD;
  EthCmd.Request = VR_FWDOWNLOAD;
  EthCmd.wValue = htons((WORD)bRestart); //bRestart = 1 on start
  EthCmd.wIndex = 0;
  EthCmd.wLength = htons((WORD)len) ;
  memcpy(EthCmd.bData, data, len);
  pthread_mutex_lock (&ethMutex);
  select (sock + 1, NULL, writefds, NULL, &tv);
  if (FD_ISSET (sock, writefds)) {
    sv = send (sock, (char *) &EthCmd, ETH_CMD_SIZE + len, 0);
    if (sv == -1) {
      pthread_mutex_unlock (&ethMutex);
      printf ("pmacSockCtrlRsp: Send error!\n");
      return -1;
    }
  } else {
    pthread_mutex_unlock (&ethMutex);
    printf ("Timeout while waiting for eth to send\n");
    return -1;
  }

  select (sock + 1, readfds, NULL, NULL, &tv);
  if (FD_ISSET (sock, readfds)) {
    rv = recv (sock, (char *) &EthCmd, 1, 0);
    pthread_mutex_unlock (&ethMutex);
    if  (rv == -1) {
      printf ("pmacSockCtrlRsp: Receive error!\n");
      return -1;
    }
    return rv;
  } else {
    pthread_mutex_unlock (&ethMutex);
    printf ("Timeout while waiting for response from pmac\n");
    return -1;
  }
}

/*
	Use this function to check if there's data on PMAC to be read.
	
	Returns: -1 if there is no data
		  1 if there is data to be read

VR_PMAC_READREADY packet determines if there is data on the Turbo PMAC 2 CPU ready to be read.
Two bytes are returned. The first byte if non-zero indicates there is data to be read; if zero, there is no
data to be read.
*/
int pmacSockReadReady (int sock, fd_set *readfds, fd_set *writefds) {
  ETHERNETCMD EthCmd;
  unsigned char data[2];
  int rv, sv;
  struct timeval tv;
  
  tv.tv_sec = ETH_COMM_TIMEOUT_SEC;
  tv.tv_usec = ETH_COMM_TIMEOUT_USEC;
      
  EthCmd.RequestType = VR_UPLOAD;
  EthCmd.Request = VR_PMAC_READREADY;
  EthCmd.wValue = 0;
  EthCmd.wIndex = 0;
  EthCmd.wLength = htons (2);
  pthread_mutex_lock (&ethMutex);
  select (sock + 1, NULL, writefds, NULL, &tv);
  if (FD_ISSET (sock, writefds)) {
    sv = send (sock, (char *) &EthCmd, ETH_CMD_SIZE, 0);
    if (sv == -1) {
      pthread_mutex_unlock (&ethMutex);
      printf ("pmacSockReadReady: Send error!\n");
      return -1;
    }
  } else {
    pthread_mutex_unlock (&ethMutex);
    printf ("Timeout while waiting for eth to send\n");
    return -1;
  }

  select (sock + 1, readfds, NULL, NULL, &tv);
  if (FD_ISSET (sock, readfds)) {
    rv = recv (sock, data, 2, 0);
    pthread_mutex_unlock (&ethMutex);
    if (rv == -1) {
      printf ("pmacSockReadReady: Receive error!\n");
      return -1;
    }

    if (data[0] == 0) return -1;
    return 1;
  } else {
    pthread_mutex_unlock (&ethMutex);
    printf ("Timeout while waiting for response from pmac\n");
    return -1;
  }
}

/*
	Get current IP address of the PMAC.
	
	Returns: -1 on failure
		  4 on success

VR_ IPADDRESS packet permits either setting or retrieval of the current IP address in the UMAC Turbo
CPU/Communications Board. When setting the IP address to a new value, it is required that the UMAC
Turbo CPU/Communications Board be powered down for the new address to take effect.
*/
int pmacSockSetIPAddress (int sock, fd_set *readfds, fd_set *writefds, int ip) {
  ETHERNETCMD EthCmd;
  int rv, sv;
  struct timeval tv;
  
  tv.tv_sec = ETH_COMM_TIMEOUT_SEC;
  tv.tv_usec = ETH_COMM_TIMEOUT_USEC;
      
  EthCmd.RequestType = VR_DOWNLOAD;      
  EthCmd.Request = VR_IPADDRESS;
  EthCmd.wValue = 0;
  EthCmd.wIndex = 0;
  EthCmd.wLength = htons (4);
  memcpy(EthCmd.bData, &ip, 4);
  pthread_mutex_lock (&ethMutex);
  select (sock + 1, NULL, writefds, NULL, &tv);
  if (FD_ISSET (sock, writefds)) {
    sv = send (sock, (char *) &EthCmd, ETH_CMD_SIZE + 4, 0);
  } else {
    pthread_mutex_unlock (&ethMutex);
    printf ("Timeout while waiting for eth to send\n");
    return -1;
  }

  select (sock + 1, readfds, NULL, NULL, &tv);
  if (FD_ISSET (sock, readfds)) {
    rv = recv (sock, &ip, 4, 0);
    pthread_mutex_unlock (&ethMutex);
    return rv;
  } else {
    pthread_mutex_unlock (&ethMutex);
    printf ("Timeout while waiting for response from pmac\n");
    return -1;
  }
}

int pmacSockGetIPAddress (int sock, fd_set *readfds, fd_set *writefds, unsigned char *ip) {
  ETHERNETCMD EthCmd;
  int rv, sv;
  struct timeval tv;
  
  tv.tv_sec = ETH_COMM_TIMEOUT_SEC;
  tv.tv_usec = ETH_COMM_TIMEOUT_USEC;
      
  EthCmd.RequestType = VR_UPLOAD;      
  EthCmd.Request = VR_IPADDRESS;
  EthCmd.wValue = 0;
  EthCmd.wIndex = 0;
  EthCmd.wLength = 0;
  pthread_mutex_lock (&ethMutex);
  select (sock + 1, NULL, writefds, NULL, &tv);
  if (FD_ISSET (sock, writefds)) {
    sv = send (sock, (char *) &EthCmd, ETH_CMD_SIZE, 0);
  } else {
    pthread_mutex_unlock (&ethMutex);
    printf ("Timeout while waiting for eth to send\n");
    return -1;
  }

  select (sock + 1, readfds, NULL, NULL, &tv);
  if (FD_ISSET (sock, readfds)) {
    rv = recv (sock, ip, 4, 0);
    pthread_mutex_unlock (&ethMutex);
    return rv;
  } else {
    pthread_mutex_unlock (&ethMutex);
    printf ("Timeout while waiting for response from pmac\n");
    return -1;
  }
}

/*
Write multiple lines to PMAC. The packet maximum size is 1024 bytes. Anything bigger must be 
separated into multiple calls of this function. Each line of program code should be terminated with null 
byte \0. 
PMAC will return 4 bytes upon successfull reception. If Byte 3 contains 0 there was no error, if it contains
0x80 there was an error. Byte 2 contains error code (Check error code list under I6 variable) and Line number
in byte 1 (MSB) and byte 0 (LSB)

Procedure returns 0 on no communication error or -1 on communication error. Response buffer should be checked
if 0 is returned.
*/
int pmacSockWriteBuffer (int sock, fd_set *readfds, fd_set *writefds, char *outBuffer, int bufferLength, char *response) {
  ETHERNETCMD EthCmd;
  int sv, rv;
  struct timeval tv;
  
  tv.tv_sec = ETH_COMM_TIMEOUT_SEC;
  tv.tv_usec = ETH_COMM_TIMEOUT_USEC;
  
  EthCmd.RequestType = VR_DOWNLOAD;
  EthCmd.Request = VR_PMAC_WRITEBUFFER;
  EthCmd.wValue = 0;
  EthCmd.wIndex = 0;
  EthCmd.wLength = htons ((WORD) bufferLength);
  memcpy (&EthCmd.bData[0], outBuffer, bufferLength);
  pthread_mutex_lock (&ethMutex);
  select (sock + 1, NULL, writefds, NULL, &tv);
  if (FD_ISSET (sock, writefds)) {
    sv = send (sock, (char*) &EthCmd, ETH_CMD_SIZE + bufferLength, 0);
  } else {
    pthread_mutex_unlock (&ethMutex);
    printf ("Timeout while waiting for eth to send\n");
    return -1;
  }
  if (sv == -1) {
    pthread_mutex_unlock (&ethMutex);
    printf ("pmacSockWriteBuffer: Send failed!\n");
    return -1;
  }

  select (sock + 1, readfds, NULL, NULL, &tv);
  if (FD_ISSET (sock, readfds)) {
    rv = recv (sock, response, 4, 0); 
    pthread_mutex_unlock (&ethMutex);

    if (rv == -1) {
      printf ("pmacSockWriteBuffer: Receive failed!\n");
    }
    return rv;
  } else {
    pthread_mutex_unlock (&ethMutex);
    printf ("Timeout while waiting for response from pmac\n");
    return -1; /* timeout*/
  }
}

/*
	Sends data to PMAC and return any available string that may be residing
	on PMAC. 
	
	Returns: -1 on failure
		 number of bytes received on success

VR_PMAC_GETRESPONSE packet causes the Ethernet connection to send a string to UMAC Turbo CPU/Communications
Board, then to return any available strings that may be residing in the PMAC. All characters up to an
<ACK> or <LF> are returned. If a <BEL> or <STX> character is detected, only the data up to the next
<CR> is returned. The maximum amount of data that is returned is 1400 Bytes. It is the caller's
responsibility to determine if there is more data to follow and if VR_PMAC_GETBUFFER needs to be
called again to retrieve all of the data available.
*/

int pmacSockGetResponse (int sock, fd_set *readfds, fd_set *writefds, char *outstr, char *response ) {
  ETHERNETCMD EthCmd;
  int sv, rv;
  struct timeval tv;
  
  tv.tv_sec = ETH_COMM_TIMEOUT_SEC;
  tv.tv_usec = ETH_COMM_TIMEOUT_USEC;
  
  EthCmd.RequestType = VR_DOWNLOAD;
  EthCmd.Request = VR_PMAC_GETRESPONSE;
  EthCmd.wValue = 0;
  EthCmd.wIndex = 0;
  EthCmd.wLength = htons ((WORD) strlen (outstr));
  memcpy (EthCmd.bData, outstr, strlen (outstr));
  pthread_mutex_lock (&ethMutex);

  select (sock + 1, NULL, writefds, NULL, &tv);
  if (FD_ISSET (sock, writefds)) {
    sv = send (sock, (char*) &EthCmd, ETH_CMD_SIZE + strlen (outstr), 0);
    if (sv == -1) {
      pthread_mutex_unlock (&ethMutex);
      printf ("pmacSockGetResponse: Send failed!\n");
      return -1;
    }
  } else {
    pthread_mutex_unlock (&ethMutex);
    printf ("Timeout while waiting for eth to send\n");
    return -1;
  }

  select (sock + 1, readfds, NULL, NULL, &tv);
  if (FD_ISSET (sock, readfds)) {
    rv = recv (sock, response, 1400, 0); 
    pthread_mutex_unlock (&ethMutex);

    if (rv == -1) {
      printf ("pmacSockGetResponse: Receive failed!\n");
    }
    return rv;
  } else {
    pthread_mutex_unlock (&ethMutex);
    printf ("Timeout while waiting for response from pmac\n");
    return -1; /* timeout*/
  }
}

/*
	Opens a socket to PMAC and sets the file descriptors
	
	Returns: -1 on failure (unable to connect...)
			 socketfd on success
*/
int pmacSockOpen (char *ipaddr, char *hostname, fd_set *readfds, fd_set *writefds) {
  int sock;
  struct sockaddr_in pmac_addr;
  
  if ((sock = socket (PF_INET, SOCK_STREAM, 0)) == -1) {
    perror ("socket");
    exit (1);
  }

  pmac_addr.sin_family = AF_INET;    /* host byte order */
  pmac_addr.sin_port = htons (PORT);  /* short, network byte order */
  pmac_addr.sin_addr = *((struct in_addr *) ipaddr);
  memset (&(pmac_addr.sin_zero), '\0', 8);  /* zero the rest of the struct */

  if (connect (sock, (struct sockaddr *) &pmac_addr, sizeof (struct sockaddr)) == -1) {
    printf ("Error while trying to establish connection with pmac! Msg: %s\n", strerror (errno));
    return -1;
  }
  
  fcntl (sock, F_SETFL, O_NONBLOCK);
  FD_ZERO (readfds);
  FD_ZERO (writefds);
  FD_SET(sock, readfds);
  FD_SET(sock, writefds);
  pthread_mutex_init (&ethMutex, NULL);
  return sock;
}

/*
	Closes the PMAC socket
*/
void pmacSockClose (int sock) {
  pthread_mutex_lock (&ethMutex);
  close (sock);
}

