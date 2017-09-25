/*
 * StringHashtable.cpp
 *
 *  Created on: 9 Feb 2016
 *      Author: gnx91527
 */

#include "StringHashtable.h"
#include <string.h>
#include <stdio.h>

StringHashtable::StringHashtable() : Hashtable() {
}

StringHashtable::~StringHashtable() {
}

std::string StringHashtable::lookup(const std::string &key) {
  std::string returnString = "";
  void *vPtr = Hashtable::lookup((const void *) key.c_str());
  if (vPtr != NULL) {
    returnString.assign((char *) vPtr);
  }
  return returnString;
}

std::string StringHashtable::insert(const std::string &key, const std::string &value) {
  std::string returnString = "";
  if (key != "") {
    char *val = (char *) malloc(((int) value.length() + 1) * sizeof(char));
    strcpy(val, value.c_str());
    void *vPtr = Hashtable::insert((const void *) key.c_str(), (void *) val);
    if (vPtr != NULL) {
      returnString.assign((char *) vPtr);
      free(vPtr);
    }
  }
  return returnString;
}

std::string StringHashtable::remove(const std::string &key) {
  void *vPtr = Hashtable::remove((const void *) key.c_str());
  std::string returnString = "";
  if (vPtr != NULL) {
    returnString.assign((char *) vPtr);
    free(vPtr);
  }
  return returnString;
}

bool StringHashtable::hasKey(const std::string &key) {
  bool retVal = false;
  void *vPtr = Hashtable::lookup((const void *) key.c_str());
  if (vPtr != NULL) {
    retVal = true;
  }
  return retVal;
}

std::string StringHashtable::firstKey() {
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

bool StringHashtable::hasNextKey() {
  return this->internal_hasNext();
}

std::string StringHashtable::nextKey() {
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

void *StringHashtable::copy_key(const void *key) {
  char *retKey = (char *) malloc((strlen((char *) key) + 1) * sizeof(char));
  strcpy(retKey, (char *) key);
  return retKey;
}

bool StringHashtable::compare_key(const void *key1, const void *key2) {
  return strcmp((char *) key1, (char *) key2) == 0;
}

void StringHashtable::release_key(void *key) {
  free(key);
}
