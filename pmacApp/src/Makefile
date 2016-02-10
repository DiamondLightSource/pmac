TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

#==================================================
# build a support library

LIBRARY_IOC += pmac

# xxxRecord.h will be created from xxxRecord.dbd
#DBDINC += xxxRecord
# install pmac.dbd into <top>/dbd
DBD += pmac.dbd

# specify all source files to be compiled and added to the library
#pmac_SRCS += xxx

pmac_LIBS += $(EPICS_BASE_IOC_LIBS)

#===========================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

