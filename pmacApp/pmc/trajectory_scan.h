; *****************************************************************************************
; Variables
; *****************************************************************************************

; Change these values for your PMAC
#define BlankAdr0           30000           ; Start of user buffer defined on PMAC
#define BlankAdrA           30001           ; Default positions for axes
#define BlankAdrB           30002
#define BlankAdrC           30003
#define BlankAdrU           30004
#define BlankAdrV           30005
#define BlankAdrW           30006
#define BlankAdrX           30007
#define BlankAdrY           30008
#define BlankAdrZ           30009
#define AxisAdr             30010           ; BlankAdr + 10 - Location of axes bit mask
#define BufferAdr           30020           ; BlankAdr + 20 - Start of points buffers
#define BuffLen             1000
#define ProgramNum          1
#define VersionNum          1.0

#define RootVar     4000

; EPICS Required Variables

#define Status              P(RootVar + 1)          ; Status of motion program for EPICS
                                                    ; 0: Initialised, 1: Active, 2: Idle, 3: Error
#define Abort               P(RootVar + 2)          ; Abort trigger for EPICS
#define Axes                P(RootVar + 3)          ; An int between 1 and 511 specifying which axes to use
#define BufferLength        P(RootVar + 4)          ; Length of a single buffer e.g. AX, AY...
#define TotalPoints         P(RootVar + 5)          ; Total number of points scanned through

#define CurrentIndex        P(RootVar + 6)          ; Current index position in buffer
#define CurrentBuffer       P(RootVar + 7)          ; Current buffer specifier - 0: A, 1: B

#define BufferAdr_A         P(RootVar + 8)          ; Start index of buffer A
#define BufferAdr_B         P(RootVar + 9)          ; Start index of buffer B
#define CurrentBufferAdr    P(RootVar + 10)         ; A or B buffer address

#define BufferFill_A        P(RootVar + 11)         ; Fill level of buffer A
#define BufferFill_B        P(RootVar + 12)         ; Fill level of buffer B
#define CurrentBufferFill   P(RootVar + 13)         ; A or B buffer fill level
#define PrevBufferFill      P(RootVar + 14)         ; Fill level of previous buffer
#define Error               P(RootVar + 15)         ; Error code  0: No error, 1: Invalid axes value,
                                                    ; 2: Move time of 0, 3: Following error/ Run-time error
#define Version             P(RootVar + 20)         ; Version number for motion program
Version = VersionNum

; Motion Program Variables
; ************************

#define Prev_A              P(RootVar + 101)        ; Specifiers for what axes are to be used
#define Prev_B              P(RootVar + 102)
#define Prev_C              P(RootVar + 103)
#define Prev_U              P(RootVar + 104)
#define Prev_V              P(RootVar + 105)
#define Prev_W              P(RootVar + 106)
#define Prev_X              P(RootVar + 107)
#define Prev_Y              P(RootVar + 108)
#define Prev_Z              P(RootVar + 109)

#define Time                P(RootVar + 110)        ; Current coordinate values
#define Current_A           P(RootVar + 111)
#define Current_B           P(RootVar + 112)
#define Current_C           P(RootVar + 113)
#define Current_U           P(RootVar + 114)
#define Current_V           P(RootVar + 115)
#define Current_W           P(RootVar + 116)
#define Current_X           P(RootVar + 117)
#define Current_Y           P(RootVar + 118)
#define Current_Z           P(RootVar + 119)
#define User                P(RootVar + 120)
#define VelMode             P(RootVar + 121)

#define Next_Time           P(RootVar + 122)        ; Time converted from Next_Time_N in 1/4ms

#define A_Vel               P(RootVar + 131)        ; Previous coordinate values
#define B_Vel               P(RootVar + 132)
#define C_Vel               P(RootVar + 133)
#define U_Vel               P(RootVar + 134)
#define V_Vel               P(RootVar + 135)
#define W_Vel               P(RootVar + 136)
#define X_Vel               P(RootVar + 137)
#define Y_Vel               P(RootVar + 138)
#define Z_Vel               P(RootVar + 139)

; Address-Based Variables
; ***********************

#define Trigger             M32

#define RTE                 M4017
RTE->Y:$203F,22

#define Next_Time_N         M4000                   ; Next coordinate values
#define Next_A              M4001
#define Next_B              M4002
#define Next_C              M4003
#define Next_U              M4004
#define Next_V              M4005
#define Next_W              M4006
#define Next_X              M4007
#define Next_Y              M4008
#define Next_Z              M4009
#define Next_User           M4010
#define NextVelMode         M4011

NextVelMode->X:$BlankAdr,4,4                        ; Set initial pointers and type
Next_User->X:$BlankAdr,0,4
Next_Time_N->Y:$BlankAdr,0,24
Next_A->L:$BlankAdrA,0,48
Next_B->L:$BlankAdrB,0,48
Next_C->L:$BlankAdrC,0,48
Next_U->L:$BlankAdrU,0,48
Next_V->L:$BlankAdrV,0,48
Next_W->L:$BlankAdrW,0,48
Next_X->L:$BlankAdrX,0,48
Next_Y->L:$BlankAdrY,0,48
Next_Z->L:$BlankAdrZ,0,48

#define Time_Adr            M4020                   ; Pointers to Next_* coordinate addresses
#define A_Adr               M4021
#define B_Adr               M4022
#define C_Adr               M4023
#define U_Adr               M4024
#define V_Adr               M4025
#define W_Adr               M4026
#define X_Adr               M4027
#define Y_Adr               M4028
#define Z_Adr               M4029
#define User_Adr            M4030
#define VelMode_Adr         M4031

Time_Adr->Y$4FA0,0,24                               ; Assignments for pointers to M address locations
A_Adr->Y:$4FA1,0,24                                 ; M0 = $4000 -> M4000 = $4FA0
B_Adr->Y:$4FA2,0,24
C_Adr->Y:$4FA3,0,24
U_Adr->Y:$4FA4,0,24
V_Adr->Y:$4FA5,0,24
W_Adr->Y:$4FA6,0,24
X_Adr->Y:$4FA7,0,24
Y_Adr->Y:$4FA8,0,24
Z_Adr->Y:$4FA9,0,24
User_Adr->Y:$4FAA,0,24
VelMode_Adr->Y:$4FAB,0,24

#define AxesParser          M4040                   ; Specifiers for what axes are to be used
#define A_Axis              M4041
#define B_Axis              M4042
#define C_Axis              M4043
#define U_Axis              M4044
#define V_Axis              M4045
#define W_Axis              M4046
#define X_Axis              M4047
#define Y_Axis              M4048
#define Z_Axis              M4049

AxesParser->Y:$AxisAdr,0,24                         ; Pointers to bits of Axes value
A_Axis->Y:$AxisAdr,0
B_Axis->Y:$AxisAdr,1
C_Axis->Y:$AxisAdr,2
U_Axis->Y:$AxisAdr,3
V_Axis->Y:$AxisAdr,4
W_Axis->Y:$AxisAdr,5
X_Axis->Y:$AxisAdr,6
Y_Axis->Y:$AxisAdr,7
Z_Axis->Y:$AxisAdr,8
