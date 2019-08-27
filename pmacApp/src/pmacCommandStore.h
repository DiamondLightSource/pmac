/*
 * pmacCommandStore.h
 *
 *  Created on: 3 Feb 2016
 *      Author: gnx91527
 */

#ifndef PMACAPP_SRC_PMACCOMMANDSTORE_H_
#define PMACAPP_SRC_PMACCOMMANDSTORE_H_

#include "epicsTypes.h"
#include "epicsMutex.h"
#include "epicsStdio.h"
#include "StringHashtable.h"
#include "pmacDebugger.h"

class pmacCommandStore : public pmacDebugger {
public:
    pmacCommandStore();

    virtual ~pmacCommandStore();

    int addItem(const std::string &key);

    int deleteItem(const std::string &key);

    bool checkForItem(const std::string &key);

    std::string readValue(const std::string &key);

    int size();

    std::string readCommandString(int index);

    int countCommandStrings();

    int updateReply(const std::string &cmd, const std::string &reply);

    void report();

    std::string getVariablesList(
            const std::string & substring,
            const std::string & remove=std::string());

private:
    void buildCommandString();

    StringHashtable store;
    char commandString[100][1024];
    int qtyCmdStrings;
};

#endif /* PMACAPP_SRC_PMACCOMMANDSTORE_H_ */
