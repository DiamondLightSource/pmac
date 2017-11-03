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
#include "pmacDebugger.h"
#include "pmacGroupsHashtable.h"
#include "CharIntHashtable.h"

class pmacController;

class pmacCsGroups : public pmacDebugger {
public:
    pmacCsGroups(pmacController *pController);

    virtual ~pmacCsGroups();

    void addGroup(int id, const std::string &name, int axisCount);

    asynStatus addAxisToGroup(int id, int axis, const std::string &axisDef, int coordSysNumber);

    int getAxisCoordSys(int axis);

    asynStatus switchToGroup(int id);

    asynStatus clearCurrentGroup();

    asynStatus manualGroup(const std::string &groupDef);

    asynStatus redefineLookaheads();


private:
    struct pmacCsAxisDef {
        std::string axisDefinition;
        int axisNo;
        int coordSysNumber;
    };

//  typedef std::map <int, pmacCsAxisDef> pmacCsAxisDefList;
    typedef pmacGroupsHashtable pmacCsAxisDefList;
    struct pmacCsGroup {
        std::string name;
        pmacCsAxisDefList axisDefs;
    };

//  typedef std::map <int, pmacCsGroup> pmacCsGroupList;
//  typedef std::map <char, int> pmacCsaxisNamesToQ;
    typedef pmacGroupsHashtable pmacCsGroupList;
    typedef CharIntHashtable pmacCsaxisNamesToQ;

    pmacController *pC_;
    pmacCsGroupList csGroups;
    pmacCsaxisNamesToQ axisNamesToQ;
    int currentGroup;

    friend class pmacController;

    friend class pmacAxis;
};

#endif /* PMACAPP_PMACASYNMOTORPORTSRC_PMACCSGROUPS_H_ */
