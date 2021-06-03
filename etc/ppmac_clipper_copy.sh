#!/bin/bash

## The pmc files for Power Pmac are maintained in the main project
## this script copies them into the power pmac clipper project for
## download into a ppmac clipper

## these files cant be symlinked because of issues with symlinks and
## windows

# run this from the etc directory after changes have beem made in the
# pmacApp/pmc directory

cp ../pmacApp/pmc/trajectory_scan_code_ppmac.pmc PowerClipper/PowerClipper/PMAC\ Script\ Language/Motion\ Programs/trajectory_scan_code_ppmac.pmc
cp ../pmacApp/pmc/trajectory_scan_definitions_ppmac.pmh PowerClipper/PowerClipper/PMAC\ Script\ Language/Global\ Includes/trajectory_scan_definitions_ppmac.pmh

cp ../pmacApp/pmc/trajectory_scan_code_ppmac.pmc LabPowerPmac/PMAC\ Script\ Language/Motion\ Programs/trajectory_scan_code_ppmac.pmc
cp ../pmacApp/pmc/trajectory_scan_definitions_ppmac.pmh LabPowerPmac/PMAC\ Script\ Language/Global\ Includes/trajectory_scan_definitions_ppmac.pmh
