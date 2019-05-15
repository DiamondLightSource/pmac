#------------------------------------------------------------------------------
# Copyright (C) Delta Tau Data Systems Inc., 2007
# All rights reserved.
# 
# Generic makefile for any c user mode plc
# For a new project change the following
# 
# 1.) PROG should be assigned the name of the desired executable program
# 2.) SRCS should be assigned the 'C' source code files that need to be compiled
# 3.) issue the command 'make depend' the first time a project is created and
#     (every time an additional 'C' file is added to the project the command 
#     'make depend' must be issued)
# 4.) issue the command make clean 
# 5.) issue the command make
#
# Notes
# --------
# Change DTDEBUG above to -O2 for release w/ optimization
# Change DTDEBUG above to -g3 for debug 
# PMAC_ARCH value can be one of the following
# arm,i386,i385hv,ppc460-2,ppc460-1,ppc405
#------------------------------------------------------------------------------

PMAC_ARCH=ppc465-2

ifeq ($(PMAC_ARCH),armv71-4.1.18)
ARCH=arm
CROSS_COMPILE=arm-omron49-linux-gnueabihf-
KDIR=/opt/armv71-4.1.18-ipipe/usr/src/ipipe-ipipe-core-4.1.18-arm-9
KSRC=/opt/armv71-4.1.18-ipipe/usr/src/ipipe-ipipe-core-4.1.18-arm-9
CC=arm-omron49-linux-gnueabihf-gcc
AS=arm-omron49-linux-gnueabihf-as
LD=arm-omron49-linux-gnueabihf-gcc
INCLUDE=/opt/armv71-4.1.18-ipipe/usr/lib/gcc/arm-linux-gnueabihf/4.6/include
XENOMAI_INC_DIR=/opt/armv71-4.1.18-ipipe/usr/xenomai/include
XENOMAI_LIB_DIR=/opt/armv71-4.1.18-ipipe/usr/xenomai/lib
LIBMATHDIR=/usr/local/dtlibs/libmath
RPATH=-Wl,-rpath-link,/opt/armv71-4.1.18-ipipe/lib/arm-linux-gnueabihf
LIBMATH_RPATH=-Wl,-rpath,/opt/ppmac/libmath
ROOTFS_DIR=/opt/armv71-4.1.18-ipipe
LIBS=  -ldl -lppmac -lpthread_rt -lxenomai -lpthread -lgcc_s -lmath -lm -L$(LIBMATHDIR)
endif

ifeq ($(PMAC_ARCH),armv71-4.1.18-usermode)
ARCH=arm
CROSS_COMPILE=arm-omron49-linux-gnueabihf-
KDIR=/opt/armv71-4.1.18-ipipe/usr/src/ipipe-ipipe-core-4.1.18-arm-9
KSRC=/opt/armv71-4.1.18-ipipe/usr/src/ipipe-ipipe-core-4.1.18-arm-9
CC=arm-omron49-linux-gnueabihf-gcc
AS=arm-omron49-linux-gnueabihf-as
LD=arm-omron49-linux-gnueabihf-gcc
INCLUDE=/opt/armv71-4.1.18-ipipe/usr/lib/gcc/arm-linux-gnueabihf/4.6/include
XENOMAI_INC_DIR=/opt/armv71-4.1.18-ipipe/usr/xenomai/include
XENOMAI_LIB_DIR=/opt/armv71-4.1.18-ipipe/usr/xenomai/lib
LIBMATHDIR=/usr/local/dtlibs/libmath
RPATH=-Wl,-rpath-link,/opt/armv71-4.1.18-ipipe/lib/arm-linux-gnueabihf
LIBMATH_RPATH=-Wl,-rpath,/opt/ppmac/libmath
ROOTFS_DIR=/opt/armv71-4.1.18-ipipe
EXTRA_DEFINES=-D__POSIX_MODE__ -D__ACONTIS__
LIBS=  -ldl -lppmac -lpthread_rt -lxenomai -lpthread -lgcc_s -lmath -lm -L$(LIBMATHDIR)
endif

ifeq ($(PMAC_ARCH),armv71-3.14.28)
ARCH=arm
CROSS_COMPILE=arm-omron-linux-gnueabihf-
KDIR=/opt/arm-rootfs/usr/src/linux-3.14.28
KSRC=/opt/arm-rootfs/usr/src/linux-3.14.28
CC=arm-omron-linux-gnueabihf-gcc
AS=arm-omron-linux-gnueabihf-as
LD=arm-omron-linux-gnueabihf-gcc
INCLUDE=/opt/arm-rootfs/usr/lib/gcc/arm-linux-gnueabihf/4.6/include
XENOMAI_INC_DIR=/opt/arm-rootfs/usr/xenomai/include
XENOMAI_LIB_DIR=/opt/arm-rootfs/usr/xenomai/lib
LIBMATHDIR=/usr/local/dtlibs/libmath
RPATH=-Wl,-rpath-link,/opt/arm-rootfs/lib/arm-linux-gnueabihf
LIBMATH_RPATH=-Wl,-rpath,/opt/ppmac/libmath
ROOTFS_DIR=/opt/arm-rootfs
LIBS=  -ldl -lppmac -lpthread_rt -lxenomai -lpthread -lm -lmath -L$(LIBMATHDIR)
endif

ifeq ($(PMAC_ARCH),armv71-3.14.28-usermode)
ARCH=arm
CROSS_COMPILE=arm-omron-linux-gnueabihf-
KDIR=/opt/arm-rootfs/usr/src/linux-3.14.28
KSRC=/opt/arm-rootfs/usr/src/linux-3.14.28
CC=arm-omron-linux-gnueabihf-gcc
AS=arm-omron-linux-gnueabihf-as
LD=arm-omron-linux-gnueabihf-gcc
INCLUDE=/opt/arm-rootfs/usr/lib/gcc/arm-linux-gnueabihf/4.6/include
XENOMAI_LIB_DIR=/opt/arm-rootfs/usr/xenomai/lib
LIBMATHDIR=/usr/local/dtlibs/libmath
RPATH=-Wl,-rpath-link,/opt/arm-rootfs/lib/arm-linux-gnueabihf
LIBMATH_RPATH=-Wl,-rpath,/opt/ppmac/libmath
ROOTFS_DIR=/opt/arm-rootfs
EXTRA_DEFINES=-D__POSIX_MODE__ -D__ACONTIS__
LIBS=  -ldl -lppmac -lpthread_rt -lxenomai -lpthread -lm -lmath -L$(LIBMATHDIR)
endif

ifeq ($(PMAC_ARCH),i386)
ARCH=i386
CROSS_COMPILE=i686-meau-linux-gnu-
KDIR=/opt/i386-linux-rootfs/usr/src/linux-headers-3.14.28-xenomai-2.6.4
KSRC=/opt/i386-linux-rootfs/usr/src/linux-headers-3.14.28-xenomai-2.6.4
CC=i686-meau-linux-gnu-gcc
AS=i686-meau-linux-gnu-as
LD=i686-meau-linux-gnu-gcc
INCLUDE=/opt/i386-linux-rootfs/usr/lib/gcc/i686-linux-gnu/4.6/include
XENOMAI_INC_DIR=/opt/i386-linux-rootfs/usr/include/xenomai
XENOMAI_LIB_DIR=/opt/i386-linux-rootfs/usr/lib
LIBMATHDIR=/usr/local/dtlibs/libmath
RPATH=-Wl,-rpath-link,/opt/i386-linux-rootfs/lib/i386-linux-gnu
LIBMATH_RPATH=-Wl,-rpath,/opt/ppmac/libmath
ROOTFS_DIR=/opt/i386-linux-rootfs
LIBS=  -ldl -lppmac -lpthread_rt -lxenomai -lpthread -lm -lmath -L$(LIBMATHDIR)
endif

ifeq ($(PMAC_ARCH),i386-usermode)
ARCH=i386
CROSS_COMPILE=i686-meau-linux-gnu-
KDIR=/opt/i386-linux-rootfs/usr/src/linux-headers-3.14.28-xenomai-2.6.4
KSRC=/opt/i386-linux-rootfs/usr/src/linux-headers-3.14.28-xenomai-2.6.4
CC=i686-meau-linux-gnu-gcc
AS=i686-meau-linux-gnu-as
LD=i686-meau-linux-gnu-gcc
INCLUDE=/opt/i386-linux-rootfs/usr/lib/gcc/i686-linux-gnu/4.6/include
XENOMAI_INC_DIR=/opt/i386-linux-rootfs/usr/include/xenomai
XENOMAI_LIB_DIR=/opt/i386-linux-rootfs/usr/lib
LIBMATHDIR=/usr/local/dtlibs/libmath
RPATH=-Wl,-rpath-link,/opt/i386-linux-rootfs/lib/i386-linux-gnu
LIBMATH_RPATH=-Wl,-rpath,/opt/ppmac/libmath
ROOTFS_DIR=/opt/i386-linux-rootfs
EXTRA_DEFINES=-D__POSIX_MODE__ -D__ACONTIS__
LIBS=  -ldl -lppmac -lpthread_rt -lxenomai -lpthread -lm -lmath -L$(LIBMATHDIR)
endif

ifeq ($(PMAC_ARCH),i386hv)
ARCH=i386
CROSS_COMPILE=i686-meau-linux-gnu-
KDIR=/opt/i386-preemptrt-rootfs/usr/src/linux-3.12.42
KSRC=/opt/i386-preemptrt-rootfs/usr/src/linux-3.12.42
CC=i686-meau-linux-gnu-gcc
AS=i686-meau-linux-gnu-as
LD=i686-meau-linux-gnu-gcc
INCLUDE=/opt/i386-preemptrt-rootfs/usr/lib/gcc/i686-redhat-linux/4.4.4/include
XENOMAI_LIB_DIR=/opt/i386-preemptrt-rootfs/lib
LIBMATHDIR=/usr/local/dtlibs/libmath
LIBMATH_RPATH=-Wl,-rpath,/opt/ppmac/libmath
ROOTFS_DIR=/opt/i386-preemptrt-rootfs
EXTRA_DEFINES=-D__HYPERVISOR__
LIBS=  -ldl -lrt -lpthread -lppmac -lmath -L$(LIBMATHDIR)
endif

ifeq ($(PMAC_ARCH),i386hv-usermode)
ARCH=i386
CROSS_COMPILE=i686-meau-linux-gnu-
KDIR=/opt/i386-preemptrt-rootfs/usr/src/linux-3.12.42
KSRC=/opt/i386-preemptrt-rootfs/usr/src/linux-3.12.42
CC=i686-meau-linux-gnu-gcc
AS=i686-meau-linux-gnu-as
LD=i686-meau-linux-gnu-gcc
INCLUDE=/opt/i386-preemptrt-rootfs/usr/lib/gcc/i686-redhat-linux/4.4.4/include
XENOMAI_LIB_DIR=/opt/i386-preemptrt-rootfs/lib
LIBMATHDIR=/usr/local/dtlibs/libmath
LIBMATH_RPATH=-Wl,-rpath,/opt/ppmac/libmath
ROOTFS_DIR=/opt/i386-preemptrt-rootfs
EXTRA_DEFINES=-D__POSIX_MODE__ -D__ACONTIS__ -D__HYPERVISOR__
LIBS=  -ldl -lrt -lpthread -lppmac -lm -lmath -L$(LIBMATHDIR)
endif

ifeq ($(PMAC_ARCH),ppc465-2)
ARCH=powerpc
CROSS_COMPILE=powerpc-meau-linux-gnu-
KDIR=/opt/powerpc-465-rootfs/usr/src/linux-3.2.21-serengeti-smp
KSRC=/opt/powerpc-465-rootfs/usr/src/linux-3.2.21-serengeti-smp
CC=powerpc-meau-linux-gnu-gcc
AS=powerpc-meau-linux-gnu-as
LD=powerpc-meau-linux-gnu-gcc
INCLUDE=/opt/powerpc-465-rootfs/usr/lib/gcc/powerpc-linux-gnu/4.6/include
XENOMAI_INC_DIR=/opt/powerpc-465-rootfs/usr/local/xenomai-2.6.2.1/include
XENOMAI_LIB_DIR=/opt/powerpc-465-rootfs/usr/local/xenomai-2.6.2.1/lib
RPATH=-Wl,-rpath-link,/opt/powerpc-465-rootfs/lib/powerpc-linux-gnu
ROOTFS_DIR=/opt/powerpc-465-rootfs
LIBS=  -ldl -lrt -lppmac -lpthread_rt -lxenomai -lpthread -lm
endif

ifeq ($(PMAC_ARCH),ppc465-1)
ARCH=powerpc
CROSS_COMPILE=powerpc-meau-linux-gnu-
KDIR=/opt/powerpc-465-rootfs/usr/src/linux-3.2.21-serengeti-smp
KSRC=/opt/powerpc-465-rootfs/usr/src/linux-3.2.21-serengeti-smp
CC=powerpc-meau-linux-gnu-gcc
AS=powerpc-meau-linux-gnu-as
LD=powerpc-meau-linux-gnu-gcc
INCLUDE=/opt/powerpc-465-rootfs/usr/lib/gcc/powerpc-linux-gnu/4.6/include
XENOMAI_INC_DIR=/opt/powerpc-465-rootfs/usr/local/xenomai-2.6.2.1/include
XENOMAI_LIB_DIR=/opt/powerpc-465-rootfs/usr/local/xenomai-2.6.2.1/lib
RPATH=-Wl,-rpath-link,/opt/powerpc-465-rootfs/lib/powerpc-linux-gnu
ROOTFS_DIR=/opt/powerpc-465-rootfs
LIBS=  -ldl -lrt -lppmac -lpthread_rt -lxenomai -lpthread -lm
endif

ifeq ($(PMAC_ARCH),ppc405)
ARCH=powerpc
CROSS_COMPILE=powerpc-405-linux-gnu-
KDIR=/opt/powerpc-405-rootfs/usr/src/linux-2.6.30.3-xeno-2.5.6
KSRC=/opt/powerpc-405-rootfs/usr/src/linux-2.6.30.3-xeno-2.5.6
CC=powerpc-405-linux-gnu-gcc
AS=powerpc-405-linux-gnu-as
LD=powerpc-405-linux-gnu-gcc
INCLUDE=/usr/local/powerpc-405-linux-gnu/include
XENOMAI_INC_DIR=/opt/powerpc-405-rootfs/usr/local/xenomai-2.5.6/include
XENOMAI_LIB_DIR=/opt/powerpc-405-rootfs/usr/local/xenomai-2.5.6/lib
LIB_DIR=/opt/powerpc-405-rootfs/lib
ROOTFS_DIR=/opt/powerpc-405-rootfs
LIBS=  -ldl -lrt -lppmac -lpthread_rt -lxenomai -lpthread -lm -L$(LIB_DIR)
endif

RTPMACINCLUDEDIR=/usr/local/dtlibs/rtpmac
LIBPPMACINCLUDEDIR=/usr/local/dtlibs/libppmac
LIBOPENERDIR=/usr/local/dtlibs/libopener
export ARCH
export CROSS_COMPILE

CFLAGS =  -mhard-float -funsigned-char --sysroot=$(ROOTFS_DIR) \
-I$(RTPMACINCLUDEDIR) -I$(LIBPPMACINCLUDEDIR) -I$(LIBOPENERDIR) -I$(XENOMAI_INC_DIR) -I$(XENOMAI_INC_DIR)/posix  -I$(LIBMATHDIR) \
-D_GNU_SOURCE -D_REENTRANT -D__XENO__ $(EXTRA_DEFINES) -DOPENER_SUPPORT_64BIT_DATATYPES

DTDEBUG = -g3

ifneq ($(PMAC_ARCH),i386hv-usermode)
ifneq ($(PMAC_ARCH),i386hv)
WRAPS = -Wl,--wrap,shm_open \
-Wl,--wrap,pthread_create \
-Wl,--wrap,pthread_create \
-Wl,--wrap,pthread_setschedparam \
-Wl,--wrap,pthread_getschedparam \
-Wl,--wrap,pthread_yield \
-Wl,--wrap,sched_yield \
-Wl,--wrap,pthread_kill \
-Wl,--wrap,sem_init \
-Wl,--wrap,sem_destroy \
-Wl,--wrap,sem_post \
-Wl,--wrap,sem_timedwait \
-Wl,--wrap,sem_wait \
-Wl,--wrap,sem_trywait \
-Wl,--wrap,sem_getvalue \
-Wl,--wrap,sem_open \
-Wl,--wrap,sem_close \
-Wl,--wrap,sem_unlink \
-Wl,--wrap,clock_getres \
-Wl,--wrap,clock_gettime \
-Wl,--wrap,clock_settime \
-Wl,--wrap,clock_nanosleep \
-Wl,--wrap,nanosleep \
-Wl,--wrap,pthread_mutexattr_init \
-Wl,--wrap,pthread_mutexattr_destroy \
-Wl,--wrap,pthread_mutexattr_gettype \
-Wl,--wrap,pthread_mutexattr_settype \
-Wl,--wrap,pthread_mutexattr_getprotocol \
-Wl,--wrap,pthread_mutexattr_setprotocol \
-Wl,--wrap,pthread_mutexattr_getpshared \
-Wl,--wrap,pthread_mutexattr_setpshared \
-Wl,--wrap,pthread_mutex_init \
-Wl,--wrap,pthread_mutex_destroy \
-Wl,--wrap,pthread_mutex_lock \
-Wl,--wrap,pthread_mutex_trylock \
-Wl,--wrap,pthread_mutex_timedlock \
-Wl,--wrap,pthread_mutex_unlock \
-Wl,--wrap,pthread_condattr_init \
-Wl,--wrap,pthread_condattr_destroy \
-Wl,--wrap,pthread_condattr_getclock \
-Wl,--wrap,pthread_condattr_setclock \
-Wl,--wrap,pthread_condattr_getpshared \
-Wl,--wrap,pthread_condattr_setpshared \
-Wl,--wrap,pthread_cond_init \
-Wl,--wrap,pthread_cond_destroy \
-Wl,--wrap,pthread_cond_wait \
-Wl,--wrap,pthread_cond_timedwait \
-Wl,--wrap,pthread_cond_signal \
-Wl,--wrap,pthread_cond_broadcast \
-Wl,--wrap,mq_open \
-Wl,--wrap,mq_close \
-Wl,--wrap,mq_unlink \
-Wl,--wrap,mq_getattr \
-Wl,--wrap,mq_setattr \
-Wl,--wrap,mq_send \
-Wl,--wrap,mq_timedsend \
-Wl,--wrap,mq_receive \
-Wl,--wrap,mq_timedreceive \
-Wl,--wrap,mq_notify \
-Wl,--wrap,open \
-Wl,--wrap,socket \
-Wl,--wrap,close \
-Wl,--wrap,ioctl \
-Wl,--wrap,read \
-Wl,--wrap,write \
-Wl,--wrap,recvmsg \
-Wl,--wrap,sendmsg \
-Wl,--wrap,recvfrom \
-Wl,--wrap,sendto \
-Wl,--wrap,recv \
-Wl,--wrap,send \
-Wl,--wrap,getsockopt \
-Wl,--wrap,setsockopt \
-Wl,--wrap,bind \
-Wl,--wrap,connect \
-Wl,--wrap,listen \
-Wl,--wrap,accept \
-Wl,--wrap,getsockname \
-Wl,--wrap,getpeername \
-Wl,--wrap,shutdown \
-Wl,--wrap,timer_create \
-Wl,--wrap,timer_delete \
-Wl,--wrap,timer_settime \
-Wl,--wrap,timer_getoverrun \
-Wl,--wrap,timer_gettime \
-Wl,--wrap,ftruncate \
-Wl,--wrap,ftruncate64 \
-Wl,--wrap,close \
-Wl,--wrap,shm_open \
-Wl,--wrap,shm_unlink \
-Wl,--wrap,mmap \
-Wl,--wrap,mmap64 \
-Wl,--wrap,munmap \
-Wl,--wrap,select
endif
endif

LDFLAGS = $(LIBS) --sysroot=$(ROOTFS_DIR) \
 \
-L../../../Bin/Debug/ \
-L$(XENOMAI_LIB_DIR) -L$(LIBPPMACINCLUDEDIR) -L$(RTPMACINCLUDEDIR) \
-Wl,-rpath,/var/ftp/usrflash/Project/C\ Language/Libraries \
-Wl,-rpath,/var/ftp/usrflash/Project/Bin/Debug \
-Wl,-rpath,/opt/ppmac/libppmac \
$(LIBMATH_RPATH) \
$(RPATH) \
$(WRAPS)

RM = rm -f
SRCS = \
capp1.c
OBJS = $(SRCS:.c=.o)
PROG = "../../../Bin/Debug/capp1.out"

# now comes a meta-rule for compiling any "C" source file.
$(PROG): $(OBJS)
	$(LD) -o $(PROG) $(OBJS) $(LDFLAGS) $(CUSTOMLDFLAGS)

%.o: %.c clean
	$(CC) $(CFLAGS) $(CUSTOMCFLAGS) $(DTDEBUG) -c $< -o $@

bclean:
	$(RM) $(PROG) $(OBJS) *.log

clean:
	$(RM) $(PROG) $(OBJS)

depend:
	$(RM) ../../../Bin/Debug/dependency.lst
	makedepend -f- -- $(CFLAGS) -- $(SRCS) > ../../../Bin/Debug/dependency.lst

