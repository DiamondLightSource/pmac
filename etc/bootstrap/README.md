Intro
=====

This is specific to the hardware that giles uses for testing at DLS but the
information may also be generally useful.

This bootstrap information allows us to set up a pmac clipper
and a power clipper from scratch. The resulting configuration is
eight dummy axes and the necessary config to run the lab example ioc or
lab-ppmac example ioc. This includes trajectory scans and rudimentaty
homing PLC.

In both cases with the ioc running the system tests in test/test_pmac should
pass.


Bootstrap the pmac clipper
==========================

Note that some of the files in this dir are copies from
/dls_sw/work/motion/Common so may need updates if these ever change.

  - connect to clipper command line
    - dls-pmac-control.py -o tcpip -s 172.23.240.97 -p 1025
  - reset the brick
    - $$$ ***
  - use load pmc button to load the following:-
    - ./PMAC_CLIPPER_M_variables.pmc
    - ./PMAC_CLIPPER_Startup.pmc
    - ../../pmacApp/pmc/PROG10_CS_motion.pmc
    - ../../pmacApp/pmc/trajectory_scan_clipper.pmc
    - ./lab.pmc

Thats it - ready to run tests

Bootstrap the power clipper
===========================

To bootstrap the Power Clipper requires downloading the Delta Tau IDE project
from ../PowerClipper/PowerClipper/

You could do this with the IDE but there may be issues with version changes
since this project was created. So a manual approach is recommended as
follows:

## Manual procedure for downloading a project

Replace path, pc ip address and fed id as appropriate:

  - ssh root@172.23.240.160
  - do a reset first:
    - gpascii
    - $$$ ***
    - <ctrl D>  (exit gpascii)
  - cd /var/ftp/usrflash
  - mv Project ProjectOld
  - scp -r hgv27681@172.23.245.116:/scratch/work/pmac/etc/PowerClipper/PowerClipper Project
  - mkdir -p /var/ftp/usrflash/Project/Log/
  - projpp
    - check in Project/Log/pp_error.log for errors
  - in order to do a save this file must exist:
    - echo > /var/ftp/usrflash/Project/Configuration/pp_save.tpl
  - save the project so it survives a reboot
    - gpascii
    - sav (wait ~10 seconds to see output)
    - <ctrl D>  (exit gpascii)


