/*
 * pmacCsGroup.cpp
 *
 *  Created on: 7 May 2015
 *      Author: Giles Knap
 *
 *  This class supports the controller's ability to switch axes in and out of coordinate systems.
 *
 *  It defines a collection of groups which each list axes, and their definition within a Coordinate
 *  system.
 *
 *  It no longer also allows for coordinated deferred moves of real axes in a CS
 *  Instead, this functionality is now managed through pmacCsController deferred moves
 */

#include "pmacCsGroups.h"
#include "pmacController.h"
#include "math.h"

// offset to get i-variables for coordinate systems
#define CSVAR(n) n+50

/**
 * pmacCsGroups constructor
 * @param pController the parent controller (friend) object
 *
 */
pmacCsGroups::pmacCsGroups(pmacController *pController) :
        pmacDebugger("pmacCSGroups"),
        pC_(pController),
        currentGroup(0) {
  // Note assume all axes are not in a coordinate system initially
  // PINI of the $(P):COORDINATE_SYS_GROUP will set the cs group later

  // set up a map of axis names and the number of the Q variable used
  // to control it (in prog 101)
  axisNamesToQ.insert('A', 71);
  axisNamesToQ.insert('B', 72);
  axisNamesToQ.insert('C', 73);
  axisNamesToQ.insert('U', 74);
  axisNamesToQ.insert('V', 75);
  axisNamesToQ.insert('W', 76);
  axisNamesToQ.insert('X', 77);
  axisNamesToQ.insert('Y', 78);
  axisNamesToQ.insert('Z', 79);
}

/**
 * pmacCsGroups destructor
 *
 */
pmacCsGroups::~pmacCsGroups() {
}

/**
 * Creates a new group of coordinate systems
 * This is the function called from iocsh with pmacCreateCsGroup
 *
 * @param id a unique identifier for this group for use in calls to addAxisGroup
 * @param name a description of this group
 * @param axisCount the number of axes to be described in this group
 * 					(using calls to addAxisGroup)
 *
 */
void pmacCsGroups::addGroup(int id, const std::string &name, int axisCount) {
  static const char *functionName = "addGroup";
  debug(DEBUG_FLOW, functionName);
  // axisCount is not required for this implementation - keeping it
  // in case we need to drop STL
  pmacCsGroup *group = new pmacCsGroup;
  group->name = name;
  debugf(DEBUG_VARIABLE, functionName, "Adding group %s with %d axes", name.c_str(), axisCount);
  csGroups.insert(id, group);
}

/**
 * Adds a new axis to a coordinate system group.
 * This is the function called from iocsh with pmacCsGroupAddAxis
 *
 * @param id a unique identifier for the group to which we are adding
 * @param axis the controller's real axis number
 * @param axisDef the CS definition for the axis
 * 					e.g. one of A, B, C, U, V, X, Y, Z, I
 * 					e.g. U*2300+20
 * @param coordinateSysNumber the coordinate system in which the axis
 * 			definition is to be defined
 *
 */
asynStatus pmacCsGroups::addAxisToGroup(int id, int axis, const std::string &axisDef,
                                        int coordSysNumber) {
  static const char *functionName = "addAxisToGroup";
  asynStatus status = asynSuccess;
  debug(DEBUG_FLOW, functionName);
  pmacCsAxisDef *def = new pmacCsAxisDef;
  def->axisDefinition = axisDef;
  def->axisNo = axis;
  def->coordSysNumber = coordSysNumber;

  pmacCsGroup *pGrp = (pmacCsGroup *) csGroups.lookup(id);
  if (pGrp == NULL) {
    status = asynError;
    debug(DEBUG_ERROR, functionName, "Invalid Coordinate System Group Number", id);
  } else {
    pGrp->axisDefs.insert(axis, def);
    debugf(DEBUG_VARIABLE,
           functionName,
           "axis %d, CS %d, def %s, COUNT %d\n",
           axis,
           coordSysNumber,
           axisDef.c_str(),
           (int) pGrp->axisDefs.count());
  }
  return status;
}

/**
 * Returns the currently assigned coordinate system number for the given axis
 * or 0 if it is in no coordinate system
 *
 * @param axis the controller's real axis number
 * @return coordinate system number
 *
 */
int pmacCsGroups::getAxisCoordSys(int axis) {
  static const char *functionName = "getAxisCoordSys";
  pmacCsGroup *pGrp;
  pmacCsAxisDef *axd = NULL;

  debug(DEBUG_FLOW, functionName);

  pGrp = (pmacCsGroup *) csGroups.lookup(currentGroup);
  if (pGrp != NULL) {
    axd = (pmacCsAxisDef *) pGrp->axisDefs.lookup(axis);
  }
  if (axd == NULL) {
    throw std::out_of_range("Axis not in CS group definition");
  }
  debug(DEBUG_VARIABLE, functionName, "Coordinate system number", axd->coordSysNumber);
  return axd->coordSysNumber;
}

/**
 * switches all axes to be mapped to the coordinate system definitions
 * defined in the given group
 *
 * @param id the coordinate system group identifier
 * @return status
 *
 */
asynStatus pmacCsGroups::switchToGroup(int id) {
  static const char *functionName = "switchToGroup";
  char command[PMAC_MAXBUF] = {0};
  char response[PMAC_MAXBUF] = {0};
  asynStatus cmdStatus = asynSuccess;
  pmacCsGroup *pGrp;
  pmacCsAxisDefList *pAxisDefs;
  debug(DEBUG_FLOW, functionName);

  if (csGroups.lookup(id) == NULL) {
    // do nothing, this will happen at startup if no groups are defined
  } else {
    // First abort motion on the currently selected group
    pGrp = (pmacCsGroup *) csGroups.lookup(currentGroup);
    if (pGrp != NULL) {
      pAxisDefs = &(pGrp->axisDefs);
      if (pAxisDefs->count() > 0) {
        int axis = pAxisDefs->firstKey();
        pmacCsAxisDef *axd = (pmacCsAxisDef *) pAxisDefs->lookup(axis);
        sprintf(command, "&%dA", axd->coordSysNumber);
        cmdStatus = pC_->lowLevelWriteRead(command, response);
        while (pAxisDefs->hasNextKey() == true && cmdStatus == asynSuccess) {
          axis = pAxisDefs->nextKey();
          axd = (pmacCsAxisDef *) pAxisDefs->lookup(axis);
          sprintf(command, "&%dA", axd->coordSysNumber);
          cmdStatus = pC_->lowLevelWriteRead(command, response);
        }
      }
    }

    // Now undefine all cs mappings
    strcpy(command, "undefine all");
    cmdStatus = pC_->lowLevelWriteRead(command, response);

    // Now redefine the mappings for the new group
    pGrp = (pmacCsGroup *) csGroups.lookup(id);
    pAxisDefs = &(pGrp->axisDefs);
    if (cmdStatus == asynSuccess) {
      currentGroup = id;

      if (pAxisDefs->count() > 0) {
        int axis = pAxisDefs->firstKey();
        pmacCsAxisDef *axd = (pmacCsAxisDef *) pAxisDefs->lookup(axis);
        sprintf(command, "&%d #%d->%s", axd->coordSysNumber,
                axd->axisNo,
                axd->axisDefinition.c_str());
        cmdStatus = pC_->lowLevelWriteRead(command, response);
        while (pAxisDefs->hasNextKey() && cmdStatus == asynSuccess) {
          axis = pAxisDefs->nextKey();
          axd = (pmacCsAxisDef *) pAxisDefs->lookup(axis);
          sprintf(command, "&%d #%d->%s", axd->coordSysNumber,
                  axd->axisNo,
                  axd->axisDefinition.c_str());
          cmdStatus = pC_->lowLevelWriteRead(command, response);

          // ensure that the Q7x will be set correctly in pmacController::makeCSDemandsConsistent
          pC_->csResetAllDemands = true;
        }
      }
    }
    if (cmdStatus == asynSuccess) {
      cmdStatus = this->redefineLookaheads();
    }
  }
  return cmdStatus;
}

asynStatus pmacCsGroups::clearCurrentGroup() {
  static const char *functionName = "clearCurrentGroup";
  char command[PMAC_MAXBUF] = {0};
  char response[PMAC_MAXBUF] = {0};
  asynStatus cmdStatus = asynSuccess;
  pmacCsGroup *pGrp;
  pmacCsAxisDefList *pAxisDefs;
  debug(DEBUG_FLOW, functionName);

  // Abort motion on the currently selected group
  pGrp = (pmacCsGroup *) csGroups.lookup(currentGroup);
  if (pGrp != NULL) {
    pAxisDefs = &(pGrp->axisDefs);
    if (pAxisDefs->count() > 0) {
      int axis = pAxisDefs->firstKey();
      pmacCsAxisDef *axd = (pmacCsAxisDef *) pAxisDefs->lookup(axis);
      sprintf(command, "&%dA", axd->coordSysNumber);
      cmdStatus = pC_->lowLevelWriteRead(command, response);
      while (pAxisDefs->hasNextKey() && cmdStatus == asynSuccess) {
        axis = pAxisDefs->nextKey();
        axd = (pmacCsAxisDef *) pAxisDefs->lookup(axis);
        sprintf(command, "&%dA", axd->coordSysNumber);
        cmdStatus = pC_->lowLevelWriteRead(command, response);
      }
    }
  }

  // Now un-define all cs mappings
  strcpy(command, "undefine all");
  cmdStatus = pC_->lowLevelWriteRead(command, response);

  return cmdStatus;
}

asynStatus pmacCsGroups::manualGroup(const std::string &groupDef) {
  static const char *functionName = "manualGroup";
  char command[PMAC_MAXBUF] = {0};
  char response[PMAC_MAXBUF] = {0};
  asynStatus status = asynSuccess;
  debug(DEBUG_FLOW, functionName);

  // Set the current group to -1 to signify manual group
  currentGroup = -1;

  // Now attempt to clear the current group
  status = clearCurrentGroup();

  // If the clear worked then make the new assignments
  if (status == asynSuccess) {
    sprintf(command, "%s", groupDef.c_str());
    status = pC_->lowLevelWriteRead(command, response);
  }

  // Now re-define the lookaheads
  if (status == asynSuccess) {
    status = this->redefineLookaheads();
  }

  return status;
}

asynStatus pmacCsGroups::redefineLookaheads() {
  asynStatus status = asynSuccess;
  char reply[PMAC_MAXBUF];
  char cmd[PMAC_MAXBUF];
  int cs = 0;
  static const char *functionName = "refefineLookaheads";

  debug(DEBUG_FLOW, functionName);
  // First any gather buffer must be deleted along with any defined lookaheads
  strcpy(cmd, "DELETE ALL TEMPS");
  if (pC_->lowLevelWriteRead(cmd, reply) != asynSuccess) {
    debug(DEBUG_ERROR, functionName, "Failed to send command", cmd);
    status = asynError;
  }

  // Next we must re-define the lookaheads
  if (status == asynSuccess) {
    for (cs = pC_->csCount; cs > 0; cs--) {
      sprintf(cmd, "&%dDEFINE LOOKAHEAD 50,10", cs);
      if (pC_->lowLevelWriteRead(cmd, reply) != asynSuccess) {
        debug(DEBUG_ERROR, functionName, "Failed to send command", cmd);
        status = asynError;
      }
    }
  }

  return status;
}

