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

wget -nv http://www.aps.anl.gov/bcda/synApps/tar/busy_R1-6-1.tar.gz
tar -zxf busy_R1-6-1.tar.gz
echo "ASYN=`pwd`/asyn-R4-26" > busy-1-6-1/configure/RELEASE 
echo "EPICS_BASE=/usr/lib/epics" >> busy-1-6-1/configure/RELEASE
make -C busy-1-6-1/

wget -nv http://www.aps.anl.gov/bcda/synApps/motor/tar/motorR6-9.tar.gz
tar -zxf motorR6-9.tar.gz
echo "ASYN=`pwd`/asyn-R4-26" > motorR6-9/configure/RELEASE
echo "BUSY=`pwd`/busy-1-6-1" >> motorR6-9/configure/RELEASE
echo "EPICS_BASE=/usr/lib/epics" >> motorR6-9/configure/RELEASE
make -C motorR6-9/

cd ..

