#!/bin/bash
# Make sure we exit on any error
set -e

mkdir external
cd external

wget -nv https://github.com/epics-modules/asyn/archive/R4-26.tar.gz
tar -zxf R4-26.tar.gz
echo "EPICS_BASE=/usr/lib/epics" > asyn-R4-26/configure/RELEASE
#echo "EPICS_LIBCOM_ONLY=YES" >> asyn-R4-26/configure/CONFIG_SITE
make -C asyn-R4-26/

wget https://github.com/epics-modules/calc/archive/R3-6-1.tar.gz
tar -zxf R3-6-1.tar.gz
echo "EPICS_BASE=/usr/lib/epics" > calc-R3-6-1/configure/RELEASE
make -C calc-R3-6-1/

wget -nv https://github.com/epics-modules/busy/archive/R1-6-1.tar.gz
tar -zxf R1-6-1.tar.gz
echo "ASYN=`pwd`/asyn-R4-26" > busy-R1-6-1/configure/RELEASE
echo "EPICS_BASE=/usr/lib/epics" >> busy-R1-6-1/configure/RELEASE
make -C busy-R1-6-1/

wget -nv https://github.com/epics-modules/motor/archive/R6-9.tar.gz
tar -zxf R6-9.tar.gz
echo "TOP = .." > motor-R6-9/motorApp/Makefile
echo "include \$(TOP)/configure/CONFIG" >> motor-R6-9/motorApp/Makefile
echo "DIRS += MotorSrc" >> motor-R6-9/motorApp/Makefile
echo "DIRS += Db" >> motor-R6-9/motorApp/Makefile
echo "include \$(TOP)/configure/RULES_DIRS" >> motor-R6-9/motorApp/Makefile
echo "======= Motor Makefile ========================================="
cat motor-R6-9/motorApp/Makefile
echo "ASYN=`pwd`/asyn-R4-26" > motor-R6-9/configure/RELEASE
echo "BUSY=`pwd`/busy-R1-6-1" >> motor-R6-9/configure/RELEASE
echo "EPICS_BASE=/usr/lib/epics" >> motor-R6-9/configure/RELEASE
make -C motor-R6-9/

cd ..

