//--------------------------------------------------------------------------- 
//  Project PMACUSB Firmware 
//  Quad 1, Inc. 
//  Copyright © 1999. All Rights Reserved. 
// 
//  SUBSYSTEM:    PMACUSB Expansion Card 
//  FILE:         VENDCMDS.H 
//  AUTHOR:       Henry Bausley 
// 
//  OVERVIEW 
//  ~~~~~~~~ 
//  Header file of constants for various vendor specific codes 
// 
//  NOTES 
//  ~~~~~ 
// 
//---------------------------------------------------------------------------- 
 
#ifndef _VENDCMDS_H 
#define _VENDCMDS_H 
 
// Command Structure for ethernet 
typedef struct tagEthernetCmd 
{ 
  BYTE  RequestType; 
  BYTE  Request; 
  WORD  wValue; 
  WORD  wIndex; 
  WORD  wLength; 
  BYTE  bData[1492]; 
}  ETHERNETCMD,*PETHERNETCMD; 
#define ETHERNETCMDSIZE    8 
 
//----------------------------------------------------------------------------- 
// Constants Defined by USB 1.1 Spec 
//----------------------------------------------------------------------------- 
#define VR_UPLOAD        0xC0 // command sent to host 
#define VR_DOWNLOAD      0x40 // command sent to device 
 
//----------------------------------------------------------------------------- 
// Constants Defined by Anchor Chips that are internal to CPU or reserved 
//----------------------------------------------------------------------------- 
#define VR_ANCHOR_DLD     0xA0 // handled by core -- A0-AF reserved by Anchor 
#define VR_EEPROM         0xA2 // loads (uploads) EEPROM 
#define VR_RAM            0xA3 // loads (uploads) external ram 
#define VR_SETI2CADDR     0xA4 
#define VR_GETI2C_TYPE    0xA5 // 8 or 16 byte address 
#define VR_GET_CHIP_REV   0xA6 // Rev A, B = 0, Rev	C = 2 
#define VR_TEST_MEM       0xA7 // runs mem test and	returns	result 
#define VR_RENUM          0xA8 // renum 
#define VR_DB_FX	      0xA9 // Force use of double byte address EEPROM (for FX) 
#define VR_I2C_100        0xAA // put the i2c bus in 100Khz mode 
#define VR_I2C_400        0xAB // put the i2c bus in 400Khz mode 
#define VR_NOSDPAUTO      0xAC // test code. does uploads using SUDPTR with manual length override 
 
#define GET_CHIP_REV()		((CPUCS >> 4) & 0x00FF) // EzUSB Chip Rev Field 
 
#define SERIAL_ADDR		  0x50 
#define EP0BUFF_SIZE      0x40 
 
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
#define VR_PMAC_WRITEERROR   0xC7 
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
 
 
#define EEPROMVER               8139 
#define EEPROMTIME              8147 
#define EEPROMDATE              8163 
#define TCP_EEPROMADDR          8179 
#define GATEWAYIP_EEPROMADDR    8180 
#define GATEWAYMASK_EEPROMADDR  8184 
#define IP_EEPROMADDR           8188 
#define MAC_EEPROMADDR          8182 
#define MAC_LEN                    6 
#define IP_LEN                     4 
#define TIME_LEN                  16 
#define DATE_LEN                  16 
#define VER_LEN                    8 
 
 
 
 
 
 
 
//----------------------------------------------------------------------------- 
// Prototypes 
//----------------------------------------------------------------------------- 
#ifndef _WINDOWS 
void EEPROMWrite(WORD addr, BYTE length, BYTE xdata *buf); //TPM EEPROM Write 
void EEPROMRead(WORD addr, BYTE length, BYTE xdata *buf);  //TPM EEPROM Read 
void WaitForEndpoint0(bit IN); 
#endif 
 
#endif 
