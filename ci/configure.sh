#!/bin/bash
# Configure ADCore in preparation for build
# Make sure we exit on any error
set -e

# Generate the configure/RELEASE.local and configure/CONFIG_SITE.linux-x86_64.Common
# with the details of where to find various external libraries.
echo "EPICS_BASE=/usr/lib/epics"             >> configure/RELEASE.local
echo "BOOST=/usr"                            >> configure/CONFIG_SITE.linux-x86_64.Common
echo "BOOST_LIB=/usr/lib/x86_64-linux-gnu"   >> configure/CONFIG_SITE.linux-x86_64.Common
echo "BOOST_INCLUDE=-I/usr/include"          >> configure/CONFIG_SITE.linux-x86_64.Common
echo "SSH="                                  >> configure/CONFIG_SITE.linux-x86_64.Common
echo "HOST_OPT=NO"                           >> configure/CONFIG_SITE.linux-x86_64.Common 
echo "USR_CXXFLAGS_Linux=--coverage"         >> configure/CONFIG_SITE.linux-x86_64.Common 
echo "USR_LDFLAGS_Linux=--coverage"          >> configure/CONFIG_SITE.linux-x86_64.Common 

echo "ASYN=`pwd`/external/asyn-R4-26"        >> configure/RELEASE.local
echo "CALC=`pwd`/external/calc-R3-6-1"       >> configure/RELEASE.local
echo "BUSY=`pwd`/external/busy-R1-6-1"        >> configure/RELEASE.local
echo "MOTOR=`pwd`/external/motor-R6-9"        >> configure/RELEASE.local

echo "======= configure/RELEASE.local ========================================="
cat configure/RELEASE.local

echo "======= configure/CONFIG_SITE.linux-x86_64.Common ======================="
cat configure/CONFIG_SITE.linux-x86_64.Common

# Remove the RELEASE.linux-x86_64.Common file
rm configure/RELEASE.linux-x86_64.Common

# Stop the building of database templates, fails without vdct and not required
# for unit tests.  Also no screen installation required.
echo "TOP = .." > pmacApp/Makefile
echo "include \$(TOP)/configure/CONFIG" >> pmacApp/Makefile
echo "DIRS := \$(DIRS) \$(filter-out \$(DIRS), \$(wildcard *src*))" >> pmacApp/Makefile
echo "DIRS := \$(DIRS) \$(filter-out \$(DIRS), \$(wildcard *Src*))" >> pmacApp/Makefile
echo "DIRS += unitTests" >> pmacApp/Makefile
echo "include \$(TOP)/configure/RULES_DIRS" >> pmacApp/Makefile

echo "======= pmacApp/Makefile ==============================================="
cat pmacApp/Makefile

echo "== ls /usr/lib =="
ls /usr/lib

echo "== ls /usr/lib/x86_64-linux-gnu =="
ls /usr/lib/x86_64-linux-gnu

echo "== ls /usr/lib/openssh =="
ls /usr/lib/openssh

echo "== ls /usr/include =="
ls /usr/include

