#ifdef __KERNEL__
#include <linux/module.h>
#else
#define EXPORT_SYMBOL(x) // x
#define KERN_ALERT
#define printk printf
#include <gplib.h>
#endif
#include <RtGpShm.h>	// Global Rt/Gp Externals and structures
#include <rtpmacapi.h>

int rtsprintf(char * buf, const char *fmt, ...);

double user_pid_ctrl( struct MotorData *Mptr);
EXPORT_SYMBOL(user_pid_ctrl);

void user_phase( struct MotorData *Mptr);
EXPORT_SYMBOL(user_phase);

void CaptCompISR(void);
EXPORT_SYMBOL(CaptCompISR);

double CfromScript(double arg1,double arg2,double arg3,double arg4,double arg5,double arg6,double arg7,struct LocalData *Ldata);
EXPORT_SYMBOL(CfromScript);

//----------------------------------------
// Required Kinematic define Names
//----------------------------------------
#define KinPosMotor  L
#define KinVelMotor  R
#define KinEnaAxisA 0x1
#define KinPosAxisA C[0]
#define KinVelAxisA C[32]
#define KinEnaAxisB 0x2
#define KinPosAxisB C[1]
#define KinVelAxisB C[33]
#define KinEnaAxisC 0x4
#define KinPosAxisC C[2]
#define KinVelAxisC C[34]
#define KinEnaAxisU 0x8
#define KinPosAxisU C[3]
#define KinVelAxisU C[35]
#define KinEnaAxisV 0x10
#define KinPosAxisV C[4]
#define KinVelAxisV C[36]
#define KinEnaAxisW 0x20
#define KinPosAxisW C[5]
#define KinVelAxisW C[37]
#define KinEnaAxisX 0x40
#define KinPosAxisX C[6]
#define KinVelAxisX C[38]
#define KinEnaAxisY 0x80
#define KinPosAxisY C[7]
#define KinVelAxisY C[39]
#define KinEnaAxisZ 0x100
#define KinPosAxisZ C[8]
#define KinVelAxisZ C[40]
#define KinEnaAxisAA 0x200
#define KinPosAxisAA C[9]
#define KinVelAxisAA C[41]
#define KinEnaAxisBB 0x400
#define KinPosAxisBB C[10]
#define KinVelAxisBB C[42]
#define KinEnaAxisCC 0x800
#define KinPosAxisCC C[11]
#define KinVelAxisCC C[43]
#define KinEnaAxisDD 0x1000
#define KinPosAxisDD C[12]
#define KinVelAxisDD C[44]
#define KinEnaAxisEE 0x2000
#define KinPosAxisEE C[13]
#define KinVelAxisEE C[45]
#define KinEnaAxisFF 0x4000
#define KinPosAxisFF C[14]
#define KinVelAxisFF C[46]
#define KinEnaAxisGG 0x8000
#define KinPosAxisGG C[15]
#define KinVelAxisGG C[47]
#define KinEnaAxisHH 0x10000
#define KinPosAxisHH C[16]
#define KinVelAxisHH C[48]
#define KinEnaAxisLL 0x20000
#define KinPosAxisLL C[17]
#define KinVelAxisLL C[49]
#define KinEnaAxisMM 0x40000
#define KinPosAxisMM C[18]
#define KinVelAxisMM C[50]
#define KinEnaAxisNN 0x80000
#define KinPosAxisNN C[19]
#define KinVelAxisNN C[51]
#define KinEnaAxisOO 0x100000
#define KinPosAxisOO C[20]
#define KinVelAxisOO C[52]
#define KinEnaAxisPP 0x200000
#define KinPosAxisPP C[21]
#define KinVelAxisPP C[53]
#define KinEnaAxisQQ 0x400000
#define KinPosAxisQQ C[22]
#define KinVelAxisQQ C[54]
#define KinEnaAxisRR 0x800000
#define KinPosAxisRR C[23]
#define KinVelAxisRR C[55]
#define KinEnaAxisSS 0x1000000
#define KinPosAxisSS C[24]
#define KinVelAxisSS C[56]
#define KinEnaAxisTT 0x2000000
#define KinPosAxisTT C[25]
#define KinVelAxisTT C[57]
#define KinEnaAxisUU 0x4000000
#define KinPosAxisUU C[26]
#define KinVelAxisUU C[58]
#define KinEnaAxisVV 0x8000000
#define KinPosAxisVV C[27]
#define KinVelAxisVV C[59]
#define KinEnaAxisWW 0x10000000
#define KinPosAxisWW C[28]
#define KinVelAxisWW C[60]
#define KinEnaAxisXX 0x20000000
#define KinPosAxisXX C[29]
#define KinVelAxisXX C[61]
#define KinEnaAxisYY 0x40000000
#define KinPosAxisYY C[30]
#define KinVelAxisYY C[62]
#define KinEnaAxisZZ 0x80000000
#define KinPosAxisZZ C[31]
#define KinVelAxisZZ C[63]
#define KinAxisUsed  D[0]
#define KinVelEna  D[0]
//----------------------------------------



