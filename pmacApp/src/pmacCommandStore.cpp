/*
 * pmacCommandStore.cpp
 *
 *  Created on: 11 Feb 2016
 *      Author: gnx91527
 */

#include "pmacCommandStore.h"
#include <stdio.h>
#include <string.h>

pmacCommandStore::pmacCommandStore()
{

}

pmacCommandStore::~pmacCommandStore()
{

}

int pmacCommandStore::addItem(const std::string& key)
{
  this->store.insert(key, "");
  this->buildCommandString();
  return 0;
}

int pmacCommandStore::deleteItem(const std::string& key)
{
  this->store.remove(key);
  this->buildCommandString();
  return 0;
}

bool pmacCommandStore::checkForItem(const std::string& key)
{
  return this->store.hasKey(key);
}

std::string pmacCommandStore::readValue(const std::string& key)
{
  return this->store.lookup(key);
}

int pmacCommandStore::size()
{
  return this->store.count();
}

std::string pmacCommandStore::readCommandString()
{
  std::string returnString = "";
  returnString.assign(this->commandString);
  returnString = returnString.substr(returnString.find_first_not_of(" "));
  return returnString;
}

int pmacCommandStore::updateReply(const std::string& reply)
{
  int running = 1;
  int status = 0;
  std::string data = reply;
  // We need to loop over the reply string splitting each data item by \r
  std::string key = this->store.firstKey();
  while (data.find("\r") != std::string::npos && running == 1){
    std::string val = data.substr(0, data.find("\r"));
    data = data.substr(data.find("\r")+1);
    this->store.insert(key, val);
    if (this->store.hasNextKey()){
      key = this->store.nextKey();
    } else {
      running = 0;
    }
  }
  if (data.find("\r") != std::string::npos){
    // Too many data items
    printf("Too many data items supplied\n");
    status = -1;
  } else {
    if (running == 1){
      // Not enough data items
      printf("Not enough data items supplied\n");
      status = -1;
    }

  }
  return status;
}

void pmacCommandStore::report()
{
  std::string key = this->store.firstKey();
  printf("[%s] => %s\n", key.c_str(), this->store.lookup(key).c_str());
  while (this->store.hasNextKey()){
    key = this->store.nextKey();
    printf("[%s] => %s\n", key.c_str(), this->store.lookup(key).c_str());
  }
}

void pmacCommandStore::buildCommandString()
{
  strcpy(commandString, "");
  std::string key = this->store.firstKey();
  sprintf(commandString, "%s %s", commandString, key.c_str());
  while (this->store.hasNextKey()){
    key = this->store.nextKey();
    sprintf(commandString, "%s %s", commandString, key.c_str());
  }
  //printf("%s\n", commandString);
}


