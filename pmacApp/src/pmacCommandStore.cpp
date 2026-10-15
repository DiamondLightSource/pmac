/*
 * pmacCommandStore.cpp
 *
 *  Created on: 11 Feb 2016
 *      Author: gnx91527
 */

#include "pmacCommandStore.h"
#include <stdio.h>
#include <string.h>

#define PMAC_MAX_REQUESTS 40

pmacCommandStore::pmacCommandStore() :
        pmacDebugger("pmacCommandStore"),
        qtyCmdStrings(0) {
  int index = 0;
  for (index = 0; index < 100; index++) {
    strcpy(commandString[index], "");
  }
}

pmacCommandStore::~pmacCommandStore() {

}

int pmacCommandStore::addItem(const std::string &key) {
  this->store.insert(key, "");
  this->buildCommandString();
  return 0;
}

int pmacCommandStore::deleteItem(const std::string &key) {
  this->store.remove(key);
  this->buildCommandString();
  return 0;
}

bool pmacCommandStore::checkForItem(const std::string &key) {
  return this->store.hasKey(key);
}

std::string pmacCommandStore::readValue(const std::string &key) {
  return this->store.lookup(key);
}

int pmacCommandStore::size() {
  return this->store.count();
}

std::string pmacCommandStore::readCommandString(int index) {
  static const char *functionName = "readCommandString";
  std::string returnString = "";
  returnString.assign(this->commandString[index]);
  if (returnString.find_first_not_of(" ") != std::string::npos) {
    returnString = returnString.substr(returnString.find_first_not_of(" "));
  }
  debug(DEBUG_VARIABLE, functionName, "Command string", returnString);
  return returnString;
}

int pmacCommandStore::countCommandStrings() {
  return qtyCmdStrings;
}

int pmacCommandStore::updateReply(const std::string &cmd, const std::string &reply) {
  static const char *functionName = "updateReply";

  std::string keys = cmd;
  std::string data = reply;

  int status = 0;
  int running = 1;
  while (data.find("\r") != std::string::npos && running == 1) {
    std::string val = data.substr(0, data.find("\r"));
    data = data.substr(data.find("\r") + 1);
    if (keys.find(" ") != std::string::npos) {
      std::string key = keys.substr(0, keys.find(" "));
      debug(DEBUG_VARIABLE, functionName, "KEY  ", key);
      debug(DEBUG_VARIABLE, functionName, "VALUE", val);
      keys = keys.substr(keys.find(" ") + 1);
      if (this->store.hasKey(key)) {
        this->store.insert(key, val);
      }
    } else {
      std::string key = keys;
      debug(DEBUG_VARIABLE, functionName, "KEY  ", key);
      debug(DEBUG_VARIABLE, functionName, "VALUE", val);
      if (this->store.hasKey(key)) {
        this->store.insert(key, val);
      }
      running = 0;
    }
  }

  /*
  std::string data = reply;
  // We need to loop over the reply string splitting each data item by \r
  std::string key = this->store.firstKey();
  if (key == ""){
    // No data items in the store
    running = 0;
  }
  while (data.find("\r") != std::string::npos && running == 1){
    std::string val = data.substr(0, data.find("\r"));
    data = data.substr(data.find("\r")+1);
//    printf("Key: %s   value: %s\n", key.c_str(), val.c_str());
    this->store.insert(key, val);
    if (this->store.hasNextKey()){
      key = this->store.nextKey();
    } else {
      running = 0;
    }
  }
  if (data.find("\r") != std::string::npos){
    // Too many data items
    debug(DEBUG_ERROR, functionName, "Too many data items supplied");
    status = -1;
  } else {
    if (running == 1){
      // Not enough data items
      debug(DEBUG_ERROR, functionName, "Not enough data items supplied");
      status = -1;
    }

  }
  this->unlock();
  */
  return status;
}

void pmacCommandStore::report() {
  std::string key = this->store.firstKey();
  printf("[%s] => %s\n", key.c_str(), this->store.lookup(key).c_str());
  while (this->store.hasNextKey()) {
    key = this->store.nextKey();
    printf("[%s] => %s\n", key.c_str(), this->store.lookup(key).c_str());
  }
}

void pmacCommandStore::buildCommandString() {
  char curStr[1024];  //NSCL/FRIB
  int index = 0;
  qtyCmdStrings = 0;
  // Fill up command string buffers, MAX_VALS in each
  std::string key = this->store.firstKey();
  sprintf(commandString[qtyCmdStrings], " %s", key.c_str());
  index++;
  while (this->store.hasNextKey()) {
    key = this->store.nextKey();
    strcpy(curStr, commandString[qtyCmdStrings]);
    sprintf(commandString[qtyCmdStrings], "%s %s", curStr, key.c_str());
    index++;
    if (index == PMAC_MAX_REQUESTS) {
      // Move onto next buffer
      index = 0;
      qtyCmdStrings++;
      strcpy(commandString[qtyCmdStrings], "");
    }
  }
  // Finally +1 to command strings
  qtyCmdStrings++;
}

std::string pmacCommandStore::getVariablesList(
        const std::string & substring, const std::string & remove) {
  std::string result;
  std::string value;
  size_t index;
  std::string key = this->store.firstKey();
  char tmp[20];

  do {
    if (key.find(substring) != std::string::npos) {
      value = this->store.lookup(key);
      // don't report on zero value variables
      if (value != "0") {
        epicsSnprintf(tmp, sizeof(tmp), "%s=%s ", key.c_str(), value.c_str());
        result.append(tmp);
      }
    }
    key = this->store.nextKey();
  } while (this->store.hasNextKey());

  // remove repetitive strings that the caller asked us to remove
  if (!remove.empty()) {
    while (index = result.find(remove), index != std::string::npos) {
      result.replace(index, remove.length(), "");
    }
  }
  return result;
}

