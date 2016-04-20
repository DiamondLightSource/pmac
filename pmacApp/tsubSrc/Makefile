TOP = ../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================
USR_CFLAGS += -DDEBUG

### build a support library
LIBRARY_IOC += tsub
LIB_LIBS += $(EPICS_BASE_IOC_LIBS)

### tsub record
tsub_SRCS  += tsubRecord.c

### tsubRecord.h will be created from tsubRecord.dbd
DBDINC += tsubRecord

### install <name>.dbd into <top>/dbd
DBD += tsubInclude.dbd

### Tsub functions sources
tsub_SRCS  += tsubSeq.c
tsub_SRCS  += tsubBD.c
tsub_SRCS  += tsubBD3.c
tsub_SRCS  += tsubCCD2009.c
# tsub_SRCS  += tsubCCD.c
# tsub_SRCS  += tsubCCD_bm_no_extern_offset.c
tsub_SRCS  += tsubCS.c
# tsub_SRCS  += tsubDN.c
tsub_SRCS  += tsubHS.c
tsub_SRCS  += tsubMD.c
tsub_SRCS  += tsubMI.c
tsub_SRCS  += tsubMO_2mtr_cry2down.c
# tsub_SRCS  += tsubMO_3mtr_cry2up.c
# tsub_SRCS  += tsubMO_3mtr_cry2down.c
tsub_SRCS  += tsubSH.c
tsub_SRCS  += tsubTB.c
# tsub_SRCS  += tsubA.c
tsub_SRCS  += tsubX.c
# tsub_SRCS  += tsubX_bm_extern_offset.c
tsub_SRCS  += tsubXY.c
tsub_SRCS  += tsubXYZ.c
tsub_SRCS  += tsubXYZ_frac.c
tsub_SRCS  += tsubAcc65e.c
tsub_SRCS  += tsubCollimator.c

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE
tsubRecord$(OBJ): $(COMMON_DIR)/tsubRecord.h
tsubSeq$(OBJ): $(COMMON_DIR)/tsubRecord.h
tsubBD$(OBJ): $(COMMON_DIR)/tsubRecord.h
tsubBD3$(OBJ): $(COMMON_DIR)/tsubRecord.h
tsubCCD2009$(OBJ): $(COMMON_DIR)/tsubRecord.h
tsubCS$(OBJ): $(COMMON_DIR)/tsubRecord.h
tsubHS$(OBJ): $(COMMON_DIR)/tsubRecord.h
tsubMD$(OBJ): $(COMMON_DIR)/tsubRecord.h
tsubMI$(OBJ): $(COMMON_DIR)/tsubRecord.h
tsubMO_2mtr_cry2down$(OBJ): $(COMMON_DIR)/tsubRecord.h
tsubSH$(OBJ): $(COMMON_DIR)/tsubRecord.h
tsubTB$(OBJ): $(COMMON_DIR)/tsubRecord.h
tsubX$(OBJ): $(COMMON_DIR)/tsubRecord.h
tsubXY$(OBJ): $(COMMON_DIR)/tsubRecord.h
tsubXYZ$(OBJ): $(COMMON_DIR)/tsubRecord.h
tsubXYZ_frac$(OBJ): $(COMMON_DIR)/tsubRecord.h
tsubAcc65e$(OBJ): $(COMMON_DIR)/tsubRecord.h
tsubCollimator$(OBJ): $(COMMON_DIR)/tsubRecord.h
