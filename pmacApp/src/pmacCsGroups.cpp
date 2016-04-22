/*
 * pmacCsGroup.cpp
 *
 *  Created on: 7 May 2015
 *      Author: Giles Knap
 *
 *  This class supports the controller's ability to switch axes in and out of coordinate systems.
 *
 *  It defines a collection of groups which each list axes, and their definition within a Coordinate system.
 *  It also allows for coordinated deferred moves of real axes in a CS where they are directly mapped to
 *  CS axes. This last feature currently requires the following PROG101 installed on the geobrick.
 *
 *  It also requires that lookahead buffers are defined for the CS in question + the following (n = CS number)
 *  i5n13=10   ; segmentation time (needed for lookahead)
 *  i5n20=50   ; lookahead length (needed to limit max velocity to max set in CS)
 *  i5n50=0    ; DISable kinematics (unless the CS does have a FORWARD/REVERSE kinematic)

 OPEN PROG 101
 CLEAR
 LINEAR
 ABS
 TM(Q70)
 FRAX(A,B,C,U,V,W,X,Y,Z)
 A(Q71)B(Q72)C(Q73)U(Q74)V(Q75)W(Q76)X(Q77)Y(Q78)Z(Q79)
 DWELL0
 CLOSE

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
		pC_(pController),
		currentGroup(0)
{
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
pmacCsGroups::~pmacCsGroups()
{
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
void pmacCsGroups::addGroup(int id, const std::string& name, int axisCount)
{
	// axisCount is not required for this implementation - keeping it
	// in case we need to drop STL
  pmacCsGroup *group = new pmacCsGroup;
	group->name = name;

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
asynStatus pmacCsGroups::addAxisToGroup(int id, int axis, const std::string& axisDef,
		int coordSysNumber)
{
	static const char *functionName = "pmacCsGroups::addAxisToGroup";
	asynStatus status = asynSuccess;
  pmacCsAxisDef *def = new pmacCsAxisDef;
	def->axisDefinition = axisDef;
	def->axisNo = axis;
	def->coordSysNumber = coordSysNumber;

	pmacCsGroup *pGrp = (pmacCsGroup *)csGroups.lookup(id);
	if (pGrp == NULL){
	  status = asynError;
    asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR, "%s: Invalid Coordinate System Group Number\n", functionName);
	} else {
	  pGrp->axisDefs.insert(axis, def);
	  asynPrint(pC_->pasynUserSelf, ASYN_TRACE_FLOW, "%s: axis %d, CS %d, def %s, COUNT %d\n",
	            functionName, axis, coordSysNumber, axisDef.c_str(), (int)pGrp->axisDefs.count());
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
int pmacCsGroups::getAxisCoordSys(int axis)
{
  pmacCsAxisDef *axd = (pmacCsAxisDef *)((pmacCsGroup *)csGroups.lookup(currentGroup))->axisDefs.lookup(axis);
  if (axd == NULL){
    throw std::out_of_range("Axis not in CS group definition");
  }
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
asynStatus pmacCsGroups::switchToGroup(int id)
{
	static const char *functionName = "pmacCsGroups::switchToGroup";
	char command[PMAC_MAXBUF] = {0};
	char response[PMAC_MAXBUF] = {0};
	asynStatus cmdStatus;

	if (csGroups.lookup(id) == NULL)
	{
		cmdStatus = asynError;
		asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR, "%s: Invalid Coordinate System Group Number\n", functionName);
	}
	else
	{
	  // First abort motion on the currently selected group
    pmacCsGroup *pGrp = (pmacCsGroup *)csGroups.lookup(currentGroup);
    pmacCsAxisDefList *pAxisDefs = &(pGrp->axisDefs);
    if (pAxisDefs->count() > 0){
      int axis = pAxisDefs->firstKey();
      pmacCsAxisDef *axd = (pmacCsAxisDef *)pAxisDefs->lookup(axis);
      sprintf(command, "&%dA", axd->coordSysNumber);
      cmdStatus = pC_->lowLevelWriteRead(command, response);
      while (pAxisDefs->hasNextKey() == true && cmdStatus == asynSuccess){
        axis = pAxisDefs->nextKey();
        axd = (pmacCsAxisDef *)pAxisDefs->lookup(axis);
        sprintf(command, "&%dA", axd->coordSysNumber);
        cmdStatus = pC_->lowLevelWriteRead(command, response);
      }
    }

    // Now undefine all cs mappings
    strcpy(command, "undefine all");
    cmdStatus = pC_->lowLevelWriteRead(command, response);

	  // Now redefine the mappings for the new group
	  pGrp = (pmacCsGroup *)csGroups.lookup(id);
    pAxisDefs = &(pGrp->axisDefs);

		if (cmdStatus == asynSuccess)
		{
			currentGroup = id;

			if (pAxisDefs->count() > 0){
			  int axis = pAxisDefs->firstKey();
        pmacCsAxisDef *axd = (pmacCsAxisDef *)pAxisDefs->lookup(axis);
        sprintf(command, "&%d #%d->%s", axd->coordSysNumber,
            axd->axisNo,
            axd->axisDefinition.c_str());
        cmdStatus = pC_->lowLevelWriteRead(command, response);
        while (pAxisDefs->hasNextKey() == true && cmdStatus == asynSuccess){
          axis = pAxisDefs->nextKey();
          axd = (pmacCsAxisDef *)pAxisDefs->lookup(axis);
          sprintf(command, "&%d #%d->%s", axd->coordSysNumber,
              axd->axisNo,
              axd->axisDefinition.c_str());
          cmdStatus = pC_->lowLevelWriteRead(command, response);
        }
			}
		}
	}
	return cmdStatus;
}

/**
 * Aborts motion programs in the CS for which the given axis is currently mapped,
 * used to stop motion intiated in the deferred coordinated move
 *
 * @param axis the axis to stop
 * @return status
 *
 */
asynStatus pmacCsGroups::abortMotion(int axis)
{
	char command[PMAC_MAXBUF] = {0};
	char response[PMAC_MAXBUF] = {0};
	int coordSysNum = getAxisCoordSys(axis);
	asynStatus status = asynSuccess;

	if(coordSysNum != 0)
	{
		sprintf(command, "&%dA", coordSysNum);

		//Execute the deferred move
		status = pC_->lowLevelWriteRead(command, response);
	}

	return status;
}

/**
 * Initiates a deferred move where all axes start and stop at the same time.
 * This is done using motion program 101 on the controller. The parameters
 * for the move are extracted from members of the pmaxAxis objects in the same
 * fashion as the already existing standard deferred move.
 *
 * @return status
 *
 */
asynStatus pmacCsGroups::processDeferredCoordMoves(void)
{
	asynStatus status = asynSuccess;
	pmacAxis *pAxis = NULL;
	static const char *functionName = "pmacCsGroups::processDeferredCoordMoves";
	int coordSysNumber = 0;
	pmacCsaxisNamesToQ moveList;
	float maxTimeToMove = 0;

	asynPrint(pC_->pasynUserSelf, ASYN_TRACE_FLOW, "%s\n", functionName);

	// Scan all axes and verify that a valid coordinated deferred move can be made.
	// All real axes with a deferred move request must be in the same coordinate system
	// and have a one to one mapping to a coordinate system axis
	for (int axis = 0; axis < pC_->numAxes_ && status == asynSuccess; axis++)
	{
		pAxis = pC_->getAxis(axis);
		if (pAxis != NULL)
		{
			if (pAxis->deferredMove_)
			{
				if(coordSysNumber == 0)
				{
					abortMotion(axis);
					coordSysNumber = getAxisCoordSys(axis);
					if(coordSysNumber == 0)
					{
						status = asynError;
						asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR,
								"%s Error: deferred coordinated move on real axis %d not in a coordinate system\n",
								functionName, axis);
					}
				}
				else
				{
					if(getAxisCoordSys(axis) != coordSysNumber)
					{
						status = asynError;
						asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR,
								"%s Error: deferred coordinated move on multiple coordinate systems %d and %d\n",
								functionName, coordSysNumber, getAxisCoordSys(axis));
					}
				}

				if(status == asynSuccess)
				{
//          std::string axisDef = csGroups[currentGroup].axisDefs[axis].axisDefinition;
          pmacCsAxisDef *axd = (pmacCsAxisDef *)((pmacCsGroup *)csGroups.lookup(currentGroup))->axisDefs.lookup(axis);
          std::string axisDef = axd->axisDefinition;
					if(axisDef.length() != 1 || axisNamesToQ.lookup(axisDef[0]) != NULL)
					{
						status = asynError;
						asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR,
								"%s Error: illegal deferred coordinated move on real axis %d defined as %s in CS %d\n",
								functionName, axis, axisDef.c_str(), coordSysNumber);
					}
					else
					{
//            moveList[axisDef[0]] = axis;
            moveList.insert(axisDef[0], axis);
						float time = pAxis->deferredTime_;
						maxTimeToMove = (time > maxTimeToMove) ? time : maxTimeToMove;
					}
				}
			}
		}
	}

	// we now have a list of axes that need to move - build the pmac command to do so
	if(status == asynSuccess && moveList.count() > 0)
	{
		char command[PMAC_MAXBUF] = {0};
		char response[PMAC_MAXBUF] = {0};
		char moveStr[PMAC_MAXBUF] = {0};

//		pmacCsaxisNamesToQ::iterator it;
//    for (it = moveList.begin(); it != moveList.end() && status == asynSuccess; it++)
//    {
//      char axisDef = it->first;
//      int axisNum = it->second;
//
//      pAxis = pC_->getAxis(axisNum);
//
//      sprintf(moveStr, "%s Q%d=%f", moveStr, axisNamesToQ[axisDef], pAxis->deferredPosition_);
//    }

    char axisDef = moveList.firstKey();
    int axisNum = moveList.lookup(axisDef);
    pAxis = pC_->getAxis(axisNum);
    sprintf(moveStr, "%s Q%d=%f", moveStr, axisNamesToQ.lookup(axisDef), pAxis->deferredPosition_);
    while (moveList.hasNextKey())
    {
      axisDef = moveList.nextKey();
      axisNum = moveList.lookup(axisDef);
      pAxis = pC_->getAxis(axisNum);
      sprintf(moveStr, "%s Q%d=%f", moveStr, axisNamesToQ.lookup(axisDef), pAxis->deferredPosition_);
    }

		// add in the axes in this CS that have not had a deferred move - send their current position
		for (int axis = 0; axis < pC_->numAxes_ && status == asynSuccess; axis++)
		{
			if(getAxisCoordSys(axis) == coordSysNumber)
			{
				pAxis = pC_->getAxis(axis);
				if (pAxis != NULL && !pAxis->deferredMove_)
				{
          pmacCsAxisDef *axd = (pmacCsAxisDef *)((pmacCsGroup *)csGroups.lookup(currentGroup))->axisDefs.lookup(axis);
          std::string axisDef = axd->axisDefinition;
//					std::string axisDef = csGroups[currentGroup].axisDefs[axis].axisDefinition;
					sprintf(moveStr, "%s Q%d=%f", moveStr, axisNamesToQ.lookup(axisDef[0]), pAxis->previous_position_);
					if(pAxis->moving_)
					{
						/// TODO remove this clause -
						// status = asynError;
						asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR,
								"%s **** WARNING: deferred coordinated move - real axis %d in CS %d is already moving\n",
								functionName, axis, coordSysNumber);
					}
				}
			}
		}

		if(status == asynSuccess)
		{
			sprintf(command, "&%d Q70=%f %s B101R", coordSysNumber, maxTimeToMove, moveStr);

			//Execute the deferred move
			asynPrint(pC_->pasynUserSelf, ASYN_TRACE_FLOW,
					"%s : deferred move command = %s\n", functionName, command);

			if (pC_->lowLevelWriteRead(command, response) != asynSuccess)
			{
				asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR,
						"%s ERROR Sending Deferred Move Command.\n", functionName);
				pC_->setIntegerParam(pC_->PMAC_C_CommsError_, pC_->PMAC_ERROR_);
				status = asynError;
			}
			else
			{
				pC_->setIntegerParam(pC_->PMAC_C_CommsError_, pC_->PMAC_OK_);
				status = asynSuccess;
			}
		}
	}

	//Clear deferred move flag for the axes involved (do this even if there was an error)
	for (int axis = 0; axis < pC_->numAxes_; axis++)
	{
		pAxis = pC_->getAxis(axis);
		if (pAxis != NULL)
		{
			pAxis->deferredMove_ = 0;
		}
	}
	return status;
}
