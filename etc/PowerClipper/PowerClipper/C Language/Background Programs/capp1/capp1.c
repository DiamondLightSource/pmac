#include <gplib.h>   // Global Gp Shared memory pointer
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
// #define _PPScriptMode_	// uncomment for Pmac Script type access
// #define _EnumMode_			// uncomment for Pmac enum data type checking on Set & Get global functions		// uncomment for Pmac Script type access


//----------------------------------------------------------------------------------
// To use the functions defined in the Libraries folder, create a prototype of the function 
// in this file or in a header file that is included in this file.
// For Example:
// If a Library project has been created with the following function and you intend to use 
// that function in this C file:
// int MyFunction(int MyVar)
// {
//	return MyVar*10;
// }
// Then a prototype of this function must be created in this c file or in a
// header file that is being included in this file. The prototype is the following:
// int MyFunction(int);
//------------------------------------------------------------------------------------

#include "../../Include/pp_proj.h"

int main(void) 
{
//---------------------------------------------------------------------
// Required Startup Code: Insures that this APP is run as an RT or NON
// RT APP otherwise depending upon how it is started it will inherit
// the scheduling priority and policy of the task that started it.
//---------------------------------------------------------------------
//----------------------------------------------
// Uncomment the below #define to run as a RT Linux APP
// #define RUN_AS_RT_APP
// For older F/W uncomment the following if you get a compile error:
// #define BACKGROUND_RT_PRIORITY 50
// #define NANO_5MSEC 5000000
//----------------------------------------------

  struct sched_param param;
  int done = 0;
	struct timespec sleeptime = {0};
	sleeptime.tv_nsec = NANO_5MSEC;	// #defines NANO_1MSEC, NANO_5MSEC & NANO_10MSEC are defined

  #ifndef RUN_AS_RT_APP	
//-----------------------------
// Runs as a NON RT Linux APP
//-----------------------------
    param.__sched_priority = 0;
    pthread_setschedparam(pthread_self(),  SCHED_OTHER, &param);
  #else 
//---------------------------------------------------------------
// Runs as a RT Linux APP with the same scheduling policy as the
// Background script PLCs
// To run at a recommended lower priority use BACKGROUND_SCRIPT_PLC_PRIORITY - 10
// To run at a higher priority use BACKGROUND_SCRIPT_PLC_PRIORITY + 1
//---------------------------------------------------------------------
    param.__sched_priority =  BACKGROUND_RT_PRIORITY - 10;
    pthread_setschedparam(pthread_self(),  SCHED_FIFO, &param);
  #endif

	InitLibrary();  // Required for accessing Power PMAC library
	
	while (!done)
	{
	//Put your code here
	
	  nanosleep(&sleeptime,NULL);	// sleep and yield to LINUX scheduler
	  done = 1;  // while loop only runs once.  remove for continues loop
	}
	CloseLibrary();	
	return 0;
}

