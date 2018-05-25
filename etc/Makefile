TOP = ..

include $(TOP)/configure/CONFIG

BUILDER_AVAILABLE := $(shell which dls-xml-iocbuilder.py 2> /dev/null)
ifdef BUILDER_AVAILABLE
    DIRS := $(DIRS) $(filter-out $(DIRS), $(wildcard makeIocs))
else
    $(info Skipping etc folder - no dls-xml-iocbuilder.py available)
endif

include $(TOP)/configure/RULES_DIRS