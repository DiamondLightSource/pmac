cmake_minimum_required(VERSION 3.5)
project(pmac)

set(WORK "/dls_sw/work/R3.14.12.7/support")
set(SUPPORT "/dls_sw/prod/R3.14.12.7/support")
set(EPICS_BASE "/dls_sw/epics/R3.14.12.7/base")
set(ASYN "${SUPPORT}/asyn/4-33")
set(BUSY "${SUPPORT}/busy/1-7")
set(MOTOR "${SUPPORT}/motor/6-10-1dls1")
set(CALC "${SUPPORT}/calc/3-7")

include_directories(
    ${ASYN}/include
    ${BUSY}/include
    ${MOTOR}/include
    ${WORK}/include
    ${SUPPORT}/include
    ${CALC}/include
    ${EPICS_BASE}/include
    ${EPICS_BASE}/include/os/Linux)

set(PMAC_SOURCE_FILES
    pmacApp/pmacAsynIPPortSrc/pmacAsynIPPort.c
    pmacApp/pmacAsynIPPortSrc/pmacAsynIPPort.h
    pmacApp/pmacAsynVMEPortSrc/asynUploadFile.c
    pmacApp/pmacAsynVMEPortSrc/devPmacMbx.c
    pmacApp/pmacAsynVMEPortSrc/devPmacRam.c
    pmacApp/pmacAsynVMEPortSrc/devStatusSoft.c
    pmacApp/pmacAsynVMEPortSrc/drvPmac.c
    pmacApp/pmacAsynVMEPortSrc/drvPmac.h
    pmacApp/pmacAsynVMEPortSrc/pmacDriver.c
    pmacApp/pmacAsynVMEPortSrc/pmacDriver.h
    pmacApp/pmacAsynVMEPortSrc/pmacError.c
    pmacApp/pmacAsynVMEPortSrc/pmacError.h
    pmacApp/pmacAsynVMEPortSrc/pmacSerial.c
    pmacApp/pmacAsynVMEPortSrc/pmacVme.c
    pmacApp/pmacAsynVMEPortSrc/pmacVme.h
    pmacApp/pmacAsynVMEPortSrc/statusRecord.c
    pmacApp/pmacAsynVMEPortSrc/tsubRecord.c
    pmacApp/pmacAsynVMEPortSrc/O.vxWorks-ppc604_long/pmacLibrary_ctdt.c
    pmacApp/pmacAsynVMEPortSrc/O.Common/statusRecord.h
    pmacApp/pmacAsynVMEPortSrc/O.Common/tsubRecord.h
    pmacApp/powerPmacAsynPortSrc/drvAsynPowerPMACPort.cpp
    pmacApp/powerPmacAsynPortSrc/drvAsynPowerPMACPort.h
    pmacApp/powerPmacAsynPortSrc/interactive.cpp
    pmacApp/powerPmacAsynPortSrc/libssh2_config.h
    pmacApp/powerPmacAsynPortSrc/sshDriver.cpp
    pmacApp/powerPmacAsynPortSrc/sshDriver.h
    pmacApp/src/CharIntHashtable.cpp
    pmacApp/src/CharIntHashtable.h
    pmacApp/src/Hashtable.cpp
    pmacApp/src/Hashtable.h
    pmacApp/src/IntegerHashtable.cpp
    pmacApp/src/IntegerHashtable.h
    pmacApp/src/StringHashtable.cpp
    pmacApp/src/StringHashtable.h
    pmacApp/src/pmacAxis.cpp
    pmacApp/src/pmacAxis.h
    pmacApp/src/pmacCSAxis.cpp
    pmacApp/src/pmacCSAxis.h
    pmacApp/src/pmacCSController.cpp
    pmacApp/src/pmacCSController.h
    pmacApp/src/pmacCSMonitor.cpp
    pmacApp/src/pmacCSMonitor.h
    pmacApp/src/pmacCallbackInterface.cpp
    pmacApp/src/pmacCallbackInterface.h
    pmacApp/src/pmacCallbackStore.cpp
    pmacApp/src/pmacCallbackStore.h
    pmacApp/src/pmacCommandStore.cpp
    pmacApp/src/pmacCommandStore.h
    pmacApp/src/pmacController.cpp
    pmacApp/src/pmacController.h
    pmacApp/src/pmacCsGroups.cpp
    pmacApp/src/pmacCsGroups.h
    pmacApp/src/pmacDebugger.cpp
    pmacApp/src/pmacDebugger.h
    pmacApp/src/pmacGroupsHashtable.cpp
    pmacApp/src/pmacGroupsHashtable.h
    pmacApp/src/pmacHardwareInterface.cpp
    pmacApp/src/pmacHardwareInterface.h
    pmacApp/src/pmacHardwarePower.cpp
    pmacApp/src/pmacHardwarePower.h
    pmacApp/src/pmacHardwareTurbo.cpp
    pmacApp/src/pmacHardwareTurbo.h
    pmacApp/src/pmacMessageBroker.cpp
    pmacApp/src/pmacMessageBroker.h
    pmacApp/src/pmacTrajectory.cpp
    pmacApp/src/pmacTrajectory.h
    pmacApp/unitTests/MockPMACAsynDriver.cpp
    pmacApp/unitTests/MockPMACAsynDriver.h
    pmacApp/unitTests/pmac-test.cpp
    pmacApp/unitTests/pmac-valgrind.cpp
    pmacApp/unitTests/pmacTestUtilities.cpp
    pmacApp/unitTests/pmacTestingUtilities.h
    pmacApp/unitTests/test_CharIntHashtable.cpp
    pmacApp/unitTests/test_IntegerHashtable.cpp
    pmacApp/unitTests/test_PMACCommandStore.cpp
    pmacApp/unitTests/test_PMACController.cpp
    pmacApp/unitTests/test_PMACCsGroups.cpp
    pmacApp/unitTests/test_PMACGroupsHashtable.cpp
    pmacApp/unitTests/test_PMACMessageBroker.cpp
    pmacApp/unitTests/test_PMACTrajectory.cpp
    pmacApp/unitTests/test_StringHashtable.cpp
    iocs/example/labApp/src/labMain.cpp
    iocs/lab/labApp/src/labMain.cpp
    iocs/lab/labApp/src/O.linux-x86_64/lab_registerRecordDeviceDriver.cpp
    iocs/gui/labApp/src/labMain.cpp
    iocs/gui/labApp/src/O.linux-x86_64/lab_registerRecordDeviceDriver.cpp
    iocs/lab-ppmac/lab-ppmacApp/src/lab-ppmacMain.cpp
    iocs/lab-ppmac/lab-ppmacApp/src/O.linux-x86_64/lab-ppmac_registerRecordDeviceDriver.cpp
    iocs/simple-power-pmac/labApp/src/labMain.cpp
    iocs/simple-power-pmac/labApp/src/O.linux-x86_64/lab_registerRecordDeviceDriver.cpp
    include/pmacController.h
    include/pmacAxis.h
    include/pmacCSController.h
    include/pmacCSAxis.h
    include/pmacCSMonitor.h
    include/pmacCsGroups.h
    include/Hashtable.h
    include/StringHashtable.h
    include/IntegerHashtable.h
    include/pmacGroupsHashtable.h
    include/CharIntHashtable.h
    include/pmacCommandStore.h
    include/pmacMessageBroker.h
    include/pmacTrajectory.h
    include/pmacHardwareInterface.h
    include/pmacHardwareTurbo.h
    include/pmacHardwarePower.h
    include/pmacCallbackStore.h
    include/pmacCallbackInterface.h
    include/pmacDebugger.h
    include/statusRecord.h
    include/tsubRecord.h
    include/pmacAsynIPPort.h
    include/drvPmacEth.h
    include/pmacEthernet.h
    cmake-build-debug/CMakeFiles/feature_tests.c
    cmake-build-debug/CMakeFiles/3.12.3/CompilerIdC/CMakeCCompilerId.c
    cmake-build-debug/CMakeFiles/3.12.3/CompilerIdCXX/CMakeCXXCompilerId.cpp
    cmake-build-debug/CMakeFiles/3.13.1/CompilerIdC/CMakeCCompilerId.c
    cmake-build-debug/CMakeFiles/3.13.1/CompilerIdCXX/CMakeCXXCompilerId.cpp
    cmake-build-debug/CMakeFiles/3.13.2/CompilerIdC/CMakeCCompilerId.c
    cmake-build-debug/CMakeFiles/3.13.2/CompilerIdCXX/CMakeCXXCompilerId.cpp
)

add_library(pmac_source ${PMAC_SOURCE_FILES})
add_custom_target(pmac_build
    COMMAND $(MAKE) -C /home/hgv27681/R3.14.12.7/support/pmac
    SOURCES ${PMAC_SOURCE_FILES})
add_executable(debug ${PMAC_SOURCE_FILES})