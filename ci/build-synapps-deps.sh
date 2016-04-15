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

cd ..

