TOP = ../..
include $(TOP)/configure/CONFIG

# set to -d to get debugging
DEBUG :=

# These are the xml files we will make iocs out of
XMLS := $(wildcard *.xml)

# These are the IOCS
IOCS := $(patsubst %.xml, %, $(XMLS))

# These are the IOCS we don't want to make simulations out of
NO_SIMS := example

# Make simulations from all IOCS not in NO_SIMS
SIMS := $(patsubst %, %_sim, $(filter-out $(NO_SIMS), $(IOCS)))

# These are the IOC dirs to make
IOC_DIRS := $(patsubst %, $(TOP)/iocs/%, $(IOCS) $(SIMS))

# Add the created iocs and sims to the install target
install: $(IOC_DIRS)

# General rule for a Standard IOC
$(TOP)/iocs/%: %.xml
	dls-xml-iocbuilder.py $(DEBUG) $<

# General rule for a simulation IOC
$(TOP)/iocs/%_sim: %.xml
	dls-xml-iocbuilder.py $(DEBUG) --sim=linux-x86 $<

# Remove the generated iocs on clean
clean:
	rm -rf $(IOC_DIRS)
