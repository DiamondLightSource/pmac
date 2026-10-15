/*
 * ArrayInfoHashtable.cpp
 *
 *  Created on: 12 Jun 2026
 *      Author: dcy
 */

#include "ArrayInfoHashtable.h"
#include <string.h>
#include <stdio.h>

struct ArrayItem
{
  int         iAsynReason;
  epicsUInt32 uArraySize;
  int         bWriteOnly;
  void*       pArrayCache;
};

ArrayInfoHashtable::ArrayInfoHashtable() : Hashtable() {
}

ArrayInfoHashtable::~ArrayInfoHashtable() {
}

bool ArrayInfoHashtable::lookup(int iAsynReason, const char** pszKey, epicsUInt32* puArraySize, int* pbWriteOnly, void** ppArrayCache) {
  for (table_entry* p=internal_begin(); p; p=internal_next()) {
    struct ArrayItem* ptr = (struct ArrayItem*)p->value;
    if (ptr && ptr->iAsynReason == iAsynReason) {
      if (pszKey)       *pszKey       = (const char*)p->key;
      if (puArraySize)  *puArraySize  = ptr->uArraySize;
      if (pbWriteOnly)  *pbWriteOnly  = ptr->bWriteOnly;
      if (ppArrayCache) *ppArrayCache = ptr->pArrayCache;
      return true;
    }
  }
  return false;
}

bool ArrayInfoHashtable::lookup(const std::string &key, int* piAsynReason, epicsUInt32* puArraySize, int* pbWriteOnly, void** ppArrayCache) {
  struct ArrayItem *ptr = (struct ArrayItem*)Hashtable::lookup((const void *) key.c_str());
  if (ptr) {
    if (piAsynReason) *piAsynReason = ptr->iAsynReason;
    if (puArraySize)  *puArraySize  = ptr->uArraySize;
    if (pbWriteOnly)  *pbWriteOnly  = ptr->bWriteOnly;
    if (ppArrayCache) *ppArrayCache = ptr->pArrayCache;
    return true;
  }
  return false;
}

bool ArrayInfoHashtable::insert(const std::string &key, int iAsynReason, epicsUInt32 uArraySize, int bWriteOnly, void* pArrayCache) {
  if (key.empty())
    return false;
  else {
    struct ArrayItem *val = (struct ArrayItem *) malloc(sizeof(struct ArrayItem));
    val->iAsynReason = iAsynReason;
    val->uArraySize  = uArraySize;
    val->bWriteOnly  = bWriteOnly;
    val->pArrayCache = pArrayCache;
    void *vPtr = Hashtable::insert((const void *) key.c_str(), (void *) val);
    if (vPtr != NULL) {
      free(vPtr);
      return false;
    }
  }
  return true;
}

void ArrayInfoHashtable::remove(const std::string &key) {
  void *vPtr = Hashtable::remove((const void *) key.c_str());
  if (vPtr != NULL)
    free(vPtr);
}

bool ArrayInfoHashtable::hasKey(const std::string &key) {
  return (Hashtable::lookup((const void *) key.c_str()) != NULL);
}

std::string ArrayInfoHashtable::firstKey() {
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

bool ArrayInfoHashtable::hasNextKey() {
  return this->internal_hasNext();
}

std::string ArrayInfoHashtable::nextKey() {
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

bool ArrayInfoHashtable::writeOnly(const std::string &key)
{
  int *ptr = (int*)Hashtable::lookup((const void *) key.c_str());
  return (ptr != NULL && ptr[2] != 0);
}

void *ArrayInfoHashtable::copy_key(const void *key) {
  char *retKey = (char *) malloc((strlen((char *) key) + 1) * sizeof(char));
  strcpy(retKey, (char *) key);
  return retKey;
}

bool ArrayInfoHashtable::compare_key(const void *key1, const void *key2) {
  return strcmp((char *) key1, (char *) key2) == 0;
}

void ArrayInfoHashtable::release_key(void *key) {
  free(key);
}
