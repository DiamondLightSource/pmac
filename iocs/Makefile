TOP = ..
include $(TOP)/configure/CONFIG
-include $(TOP)/etc/makeIocs/Makefile.iocs

# only build real iocs if not in prod
ifeq (,$(findstring prod, $(shell pwd)))
	DIRS := $(wildcard $(IOCS))
endif

include $(TOP)/configure/RULES_DIRS
