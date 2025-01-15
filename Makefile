 #Makefile at top of application tree
 TOP = .
 include $(TOP)/configure/CONFIG
 DIRS := $(DIRS) $(filter-out $(DIRS), configure)
 DIRS := $(DIRS) $(filter-out $(DIRS), $(wildcard *App))
 DIRS := $(DIRS) $(filter-out $(DIRS), $(wildcard iocBoot))

ifneq (,$(findstring R3.14, $(EPICS_BASE)))
  # If R3.14 is in EPICS_BASE then make the builder IOCs
  # DLS IOC Builder is not currently supported in epics 7

  DIRS := $(DIRS) $(filter-out $(DIRS), $(wildcard etc))

  ifeq ($(wildcard etc),etc)
         include $(TOP)/etc/makeIocs/Makefile.iocs
         UNINSTALL_DIRS += documentation/doxygen $(IOC_DIRS)
  endif


  # If prod is not in pwd then make the iocs
  ifeq (,$(findstring prod, $(shell pwd)))
  DIRS := $(DIRS) $(filter-out $(DIRS), $(wildcard iocs))
  endif
endif

 define DIR_template
  $(1)_DEPEND_DIRS = configure
 endef
 $(foreach dir, $(filter-out configure,$(DIRS)),$(eval $(call DIR_template,$(dir))))
 iocBoot_DEPEND_DIRS += $(filter %App,$(DIRS))
 etc_DEPEND_DIRS += pmacApp
 iocs_DEPEND_DIRS += etc
 include $(TOP)/configure/RULES_TOP
