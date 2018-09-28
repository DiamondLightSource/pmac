//---------------------------------------------------------------------------
//  Project PowerPMAC Firmware
//  Delta Tau Data Systems, Inc.
//  Copyright  2007. All Rights Reserved.
//
//  SUBSYSTEM:			User Servo Driver
//  FILE:         		usrcode.c
//  TEMPLATE AUTHOR:		Henry Bausley
//
//  OVERVIEW
//  ~~~~~~~~
//  This file is where exportable user code can be placed.
//  To make a function callable as a user servo do three steps
//
//  1.)  Prototye the function user_func(void ,void );
//  2.)  Export the function  EXPORT_SYMBOL(user_func);
//  3.)  Make sure useralgo.ko has been loaded with projpp.ini
//
//--------------------------------------------------------------------------------
#include "usrcode.h"
//----------------------------------------------------------------------------------
// pp_proj.h is the C header for accessing PMAC Global, CSGlobal, Ptr vars
// _PPScriptMode_ for Pmac Script like access global & csglobal
// global Mypvar - access with "Mypvar"
// global Myparray(32) - access with "Myparray(i)"
// csglobal Myqvar - access with "Myqvar(i)" where "i" is Coord #
// csglobal Myqarray(16) - access with "Myqvar(i,j)" where "j" is index
// _EnumMode_ for Pmac enum data type checking on Set & Get global functions
// Example
// global Mypvar
// csglobal Myqvar
// "SetGlobalVar(Myqvar, data)" will give a compile error because its a csglobal var.
// "SetCSGlobalVar(Mypvar, data)" will give a compile error because its a global var.
//------------------------------------------------------------------------------------
#define _PPScriptMode_	// uncomment for Pmac Script type access
// #define _EnumMode_			// uncomment for Pmac enum data type checking on Set & Get global functions		

#include "../Include/pp_proj.h"
#ifdef __KERNEL__
// Kernal mode can't have paths with spaces and long names
//#include "../../PMACSC~1/GLOBAL~1/asharedwithcapp.pmh"
#else
//#include "../../PMAC Script Language/Global Includes/asharedwithcapp.pmh"
#endif

extern struct SHM        *pshm;  // Pointer to shared memory
extern volatile unsigned *piom;  // Pointer to I/O memory
extern void              *pushm; // Pointer to user memory

void user_phase( struct MotorData *Mptr)
{
}

double user_pid_ctrl( struct MotorData *Mptr)
{
  double *p;
  p = pushm;
  return 0;
}

void CaptCompISR(void)
{
  unsigned *pUnsigned = pushm;
  *pUnsigned = *pUnsigned + 1;
}

double GetLocal(struct LocalData *Ldata,int m)
{
  return *(Ldata->L + Ldata->Lindex + m);
}

void SetLocal(struct LocalData *Ldata,int m,double value)
{
  *(Ldata->L + Ldata->Lindex + m) = value;
}

double *GetLocalPtr(struct LocalData *Ldata,int m)
{
  return (Ldata->L + Ldata->Lindex + m);
}

double CfromScript(double cfrom_type, double arg2, double arg3, double arg4, double arg5, double arg6, double arg7, struct LocalData *Ldata)
{
  int icfrom_type = (int) cfrom_type;
  double *C, *D, *L, *R, rtn; // C, D, R - only needed if doing Kinmatics
  
  C = GetCVarPtr(Ldata);  // Only needed if doing Kinmatics
  D = GetDVarPtr(Ldata);  // Only needed if doing Kinmatics
  L = GetLVarPtr(Ldata);  // Only needed if using Ldata or Kinmatics
  R = GetRVarPtr(Ldata);  // Only needed if doing Kinmatics
  rtn = -1.0;
  return rtn;
}

