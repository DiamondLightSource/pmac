TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE

#=============================
# Build the IOC support library
ifdef SSH
	LIBRARY_Linux += powerPmacAsynPort

	DBD += drvAsynPowerPMACPort.dbd

	# Link locally-provided code into the support library,
	powerPmacAsynPort_SRCS += sshDriver.cpp
	powerPmacAsynPort_SRCS += drvAsynPowerPMACPort.cpp

	powerPmacAsynPort_LIBS += asyn
	powerPmacAsynPort_LIBS += motor
	powerPmacAsynPort_LIBS += ssh2
	powerPmacAsynPort_LIBS += $(EPICS_BASE_IOC_LIBS)
	ssh2_DIR = $(SSH_LIB)
	USR_INCLUDES+= $(SSH_INCLUDE)
	
	#=============================
	# Build the interactive shell test application
	
	PROD_IOC_Linux = powerPMACShell
	
	powerPMACShell_SRCS += interactive.cpp
	
	# Link in the code from the support library
	powerPMACShell_LIBS += powerPmacAsynPort
	powerPMACShell_LIBS += asyn
	powerPMACShell_LIBS += motor
	powerPMACShell_LIBS += ssh2
	ssh2_DIR = $(SSH_LIB)
	USR_INCLUDES+= $(SSH_INCLUDE)

	# Finally link to the EPICS Base libraries
	powerPMACShell_LIBS += $(EPICS_BASE_IOC_LIBS)

endif

#=============================


include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

