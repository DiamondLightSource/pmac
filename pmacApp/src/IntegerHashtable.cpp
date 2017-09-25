/*
 * IntegerHashtable.cpp
 *
 *  Created on: 17 Feb 2016
 *      Author: gnx91527
 */

#include "IntegerHashtable.h"
#include <string.h>
#include <stdio.h>

IntegerHashtable::IntegerHashtable() : Hashtable() {
}

IntegerHashtable::~IntegerHashtable() {
}

int IntegerHashtable::lookup(const std::string &key) {
  int returnInt = 0;
  void *vPtr = Hashtable::lookup((const void *) key.c_str());
  if (vPtr != NULL) {
    returnInt = *((int *) vPtr);
  } else {
    throw std::out_of_range("IntegerHashtable: Key not found");
  }
  return returnInt;
}

void IntegerHashtable::insert(const std::string &key, int value) {
  if (key != "") {
    int *val = (int *) malloc(sizeof(int));
    *val = value;
    void *vPtr = Hashtable::insert((const void *) key.c_str(), (void *) val);
    if (vPtr != NULL) {
      free(vPtr);
    }
  }
}

int IntegerHashtable::remove(const std::string &key) {
  void *vPtr = Hashtable::remove((const void *) key.c_str());
  int returnInt = 0;
  if (vPtr != NULL) {
    returnInt = *((int *) vPtr);
    free(vPtr);
  } else {
    throw std::out_of_range("IntegerHashtable: Key not found");
  }
  return returnInt;
}

bool IntegerHashtable::hasKey(const std::string &key) {
  bool retVal = false;
  void *vPtr = Hashtable::lookup((const void *) key.c_str());
  if (vPtr != NULL) {
    retVal = true;
  }
  return retVal;
}

std::string IntegerHashtable::firstKey() {
  char *key = NULL;
  table_entry *entry = this->internal_begin();
  if (entry != NULL) {
    key = (char *) entry->key;
  }
  std::string returnString = "";
  if (key != NULL) {
    returnString.assign(key);
  }
  return returnString;
}

bool IntegerHashtable::hasNextKey() {
  return this->internal_hasNext();
}

std::string IntegerHashtable::nextKey() {
  char *key = NULL;
  table_entry *entry = this->internal_next();
  if (entry != NULL) {
    key = (char *) entry->key;
  }
  std::string returnString = "";
  if (key != NULL) {
    returnString.assign(key);
  }
  return returnString;
}

void *IntegerHashtable::copy_key(const void *key) {
  char *retKey = (char *) malloc((strlen((char *) key) + 1) * sizeof(char));
  strcpy(retKey, (char *) key);
  return retKey;
}

bool IntegerHashtable::compare_key(const void *key1, const void *key2) {
  return strcmp((char *) key1, (char *) key2) == 0;
}

void IntegerHashtable::release_key(void *key) {
  free(key);
}
