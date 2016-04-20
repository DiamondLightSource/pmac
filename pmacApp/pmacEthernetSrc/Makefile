TOP = ../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================
CROSS_COMPILER_TARGET_ARCHS=
#=============================
# build a support library
LIBRARY = pmac-eth


DBD += pmac-ethSupport.dbd

DBDINC += statusRecord

INC += drvPmacEth.h
INC += pmacEthernet.h

# The following are compiled and added to the Support library
pmac-eth_SRCS += pmacError.c
pmac-eth_SRCS += devPmacMbx.c
pmac-eth_SRCS += devPmacRam.c
pmac-eth_SRCS += pmacRam.c
pmac-eth_SRCS += drvPmacEth.c
pmac-eth_SRCS += pmacEthernet.c
pmac-eth_SRCS += statusRecord.c

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE
