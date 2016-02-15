/*
 * pmacCommandStore.h
 *
 *  Created on: 3 Feb 2016
 *      Author: gnx91527
 */

#ifndef PMACAPP_SRC_PMACCOMMANDSTORE_H_
#define PMACAPP_SRC_PMACCOMMANDSTORE_H_

#include "epicsTypes.h"
#include "StringHashtable.h"

class pmacCommandStore
{
public:
  pmacCommandStore();
  virtual ~pmacCommandStore();
  int addItem(const std::string& key);
  int deleteItem(const std::string& key);
  bool checkForItem(const std::string& key);
  std::string readValue(const std::string& key);
  int size();
  std::string readCommandString();
  int updateReply(const std::string& reply);
  void report();

private:
  void buildCommandString();

  StringHashtable store;
  char commandString[1024];
};

#endif /* PMACAPP_SRC_PMACCOMMANDSTORE_H_ */
