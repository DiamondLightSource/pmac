/*
 * pmacGroupsHashtable.cpp
 *
 *  Created on: 10 Feb 2016
 *      Author: gnx91527
 */

#include "pmacGroupsHashtable.h"
#include <string.h>
#include <stdlib.h>

pmacGroupsHashtable::pmacGroupsHashtable() : Hashtable() {
}

pmacGroupsHashtable::~pmacGroupsHashtable() {
}

void *pmacGroupsHashtable::lookup(int key) {
  return Hashtable::lookup(&key);
}

void *pmacGroupsHashtable::insert(int key, void *value) {
  return Hashtable::insert(&key, value);
}

void *pmacGroupsHashtable::remove(int key) {
  return Hashtable::remove(&key);
}

int pmacGroupsHashtable::firstKey() {
  int *key = NULL;
  table_entry *entry = this->internal_begin();
  if (entry != NULL) {
    key = (int *) entry->key;
  }
  return *key;
}

bool pmacGroupsHashtable::hasNextKey() {
  return this->internal_hasNext();
}

int pmacGroupsHashtable::nextKey() {
  int *key = NULL;
  table_entry *entry = this->internal_next();
  if (entry != NULL) {
    key = (int *) entry->key;
  }
  return *key;

}

void *pmacGroupsHashtable::copy_key(const void *key) {
  void *retKey = malloc(sizeof(int));
  memcpy(retKey, key, sizeof(int));
  return retKey;
}

bool pmacGroupsHashtable::compare_key(const void *key1, const void *key2) {
  if (key1 == NULL || key2 == NULL) {
    return false;
  }
  return (*(int *) key1 == *(int *) key2);
}

void pmacGroupsHashtable::release_key(void *key) {
  free(key);
}
