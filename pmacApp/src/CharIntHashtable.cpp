/*
 * CharIntHashtable.cpp
 *
 *  Created on: 9 Feb 2016
 *      Author: gnx91527
 */

#include "CharIntHashtable.h"
#include <string.h>
#include <stdio.h>
#include <stdexcept>

CharIntHashtable::CharIntHashtable() : Hashtable() {
}

CharIntHashtable::~CharIntHashtable() {
}

int CharIntHashtable::lookup(const char key) {
  char nkey[2];
  nkey[0] = key;
  nkey[1] = 0;
  void *ptr = Hashtable::lookup(&nkey);
  if (ptr == NULL) {
    throw (std::out_of_range("No entry in hashtable"));
  }
  return *(int *) ptr;
}

int CharIntHashtable::insert(const char key, int value) {
  char nkey[2];
  int *val = (int *) malloc(sizeof(int));
  *val = value;
  nkey[0] = key;
  nkey[1] = 0;
  void *vPtr = Hashtable::insert(&nkey, (void *) val);
  if (vPtr != NULL) {
    return *(int *) vPtr;
  }
  return 0;
}

int CharIntHashtable::remove(const char key) {
  char nkey[2];
  nkey[0] = key;
  nkey[1] = 0;
  return *(int *) Hashtable::remove(&nkey);
}

char CharIntHashtable::firstKey() {
  char *key = NULL;
  table_entry *entry = this->internal_begin();
  if (entry != NULL) {
    key = (char *) entry->key;
  }
  return *key;
}

bool CharIntHashtable::hasNextKey() {
  return this->internal_hasNext();
}

char CharIntHashtable::nextKey() {
  char *key = NULL;
  table_entry *entry = this->internal_next();
  if (entry != NULL) {
    key = (char *) entry->key;
  }
  return *key;
}

void *CharIntHashtable::copy_key(const void *key) {
  char *retKey = (char *) malloc(sizeof(char));
  *retKey = *(char *) key;
  return retKey;
}

bool CharIntHashtable::compare_key(const void *key1, const void *key2) {
  return (*(char *) key1 == *(char *) key2);
}

void CharIntHashtable::release_key(void *key) {
  free(key);
}
