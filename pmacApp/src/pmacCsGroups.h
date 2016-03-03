/*
 * pmacCsGroup.h
 *
 *  Created on: 7 May 2015
 *      Author: hgv27681
 */

#ifndef PMACAPP_PMACASYNMOTORPORTSRC_PMACCSGROUPS_H_
#define PMACAPP_PMACASYNMOTORPORTSRC_PMACCSGROUPS_H_

#define MAX_AXIS_DEF 32

#include <stdio.h>
#include <vector>
#include <string>
#include <asynDriver.h>
#include <map>
#include "pmacGroupsHashtable.h"
#include "CharIntHashtable.h"

class pmacController;

class pmacCsGroups
{
public:
	pmacCsGroups(pmacController *pController);
	virtual ~pmacCsGroups();

	void addGroup(int id, char* name, int axisCount);
	void addAxisToGroup(int id, int axis, char* axisDef, int coordSysNumber);
	int getAxisCoordSys(int axis);
	asynStatus switchToGroup(int id);
	asynStatus processDeferredCoordMoves(void);
	asynStatus abortMotion(int axis);

private:
	struct pmacCsAxisDef
	{
		std::string axisDefinition;
		int	axisNo;
		int	coordSysNumber;
	};

//  typedef std::map <int, pmacCsAxisDef> pmacCsAxisDefList;
  typedef pmacGroupsHashtable pmacCsAxisDefList;
	struct pmacCsGroup
	{
		std::string	name;
		pmacCsAxisDefList axisDefs;
	};

//  typedef std::map <int, pmacCsGroup> pmacCsGroupList;
//  typedef std::map <char, int> pmacCsaxisNamesToQ;
  typedef pmacGroupsHashtable pmacCsGroupList;
  typedef CharIntHashtable pmacCsaxisNamesToQ;

	pmacController *pC_;
	pmacCsGroupList	csGroups;
	pmacCsaxisNamesToQ axisNamesToQ;
	int currentGroup;

	friend class pmacController;
	friend class pmacAxis;
};

#endif /* PMACAPP_PMACASYNMOTORPORTSRC_PMACCSGROUPS_H_ */
