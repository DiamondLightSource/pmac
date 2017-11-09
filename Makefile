 #Makefile at top of application tree
 TOP = .
 include $(TOP)/configure/CONFIG
 DIRS := $(DIRS) $(filter-out $(DIRS), configure)
 DIRS := $(DIRS) $(filter-out $(DIRS), $(wildcard *App))
 DIRS := $(DIRS) $(filter-out $(DIRS), $(wildcard iocBoot))

 DIRS := $(DIRS) $(filter-out $(DIRS), $(wildcard etc))

 ifeq ($(wildcard etc),etc)
         include $(TOP)/etc/makeIocs/Makefile.iocs
         UNINSTALL_DIRS += documentation/doxygen $(IOC_DIRS)
 endif


 # If prod is not in pwd then make the iocs
 ifeq (,$(findstring prod, $(shell pwd)))
 DIRS := $(DIRS) $(filter-out $(DIRS), $(wildcard iocs))
 endif

 define DIR_template
  $(1)_DEPEND_DIRS = configure
 endef
 $(foreach dir, $(filter-out configure,$(DIRS)),$(eval $(call DIR_template,$(dir))))
 iocBoot_DEPEND_DIRS += $(filter %App,$(DIRS))
 iocs_DEPEND_DIRS += etc
 include $(TOP)/configure/RULES_TOP
