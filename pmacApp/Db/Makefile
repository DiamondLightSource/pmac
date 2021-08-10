TOP=../..
include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE

#----------------------------------------------------
#  Optimization of db files using dbst (DEFAULT: NO)
#DB_OPT = YES

#----------------------------------------------------
# Create and install (or just install) into <top>/db
# databases, templates, substitutions like this


DB += pmacController.template
DB += pmacCsController.template
DB += pmacControllerTrajectory.template
DB += pmacStatusAxis.template
DB += pmacStatus8Axes.template
DB += pmacStatus32Axes.template
DB += powerPmacStatus.template
DB += pmacStatus.template
DB += pmacDirectMotor.template
DB += pmac_cs_axis.template
DB += dls_pmac_asyn_motor.template
DB += motor_in_cs.template
DB += eloss_kill_autohome_records.template
DB += pmacVariableWrite.template
DB += pmacVariableRead.template
DB += autohome.template
DB += encoder_readback.template
DB += run_plc.template
DB += run_command.template
DB += motion_stop.template


# The following templates are used for Coordinate Systems
DB += dls_pmac_cs_asyn_motor.template
DB += 1jack_compensated.template
DB += 2jack.template
DB += 3jack_mirror.template
DB += 3jack.template
DB += accel_dcm.template
DB += aperture_slits.template
DB += B22_optics_box.template
DB += bender.template
DB += blade_slits.template
DB += flexure_slits.template
DB += gap_and_centre_slits.template
DB += IDT_sagittal_bender.template
DB += IDT_sagittal_dcm.template
DB += multi_beamstop_on_platform.template
DB += qcm.template
DB += slit4-gui.template
DB += symetrie_hexapod.template
DB += translated_motor.template
DB += zform.template


# The following templates are used for CSS screens
DB += pmacCSS.template

#----------------------------------------------------
# If <anyname>.db template is not named <anyname>*.template add
# <anyname>_template = <templatename>

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

