/*
 * CharIntHashtable.cpp
 *
 *  Created on: 9 Feb 2016
 *      Author: gnx91527
 */

#include "CharIntHashtable.h"
#include <string.h>
#include <stdio.h>

CharIntHashtable::CharIntHashtable() : Hashtable()
{
}

CharIntHashtable::~CharIntHashtable()
{
}

int CharIntHashtable::lookup(const char key)
{
  return *(int *)Hashtable::lookup(&key);
}

int CharIntHashtable::insert(const char key, int value)
{
  int *val = (int *)malloc(sizeof(int));
  *val = value;
  void *vPtr = Hashtable::insert(&key, (void *)val);
  if (vPtr != NULL){
    return *(int *)vPtr;
  }
  return NULL;
}

int CharIntHashtable::remove(const char key)
{
  return *(int *)Hashtable::remove(&key);
}

char CharIntHashtable::firstKey()
{
  char *key = NULL;
  table_entry *entry = this->internal_begin();
  if (entry != NULL){
    key = (char *)entry->key;
  }
  return *key;
}

bool CharIntHashtable::hasNextKey()
{
  return this->internal_hasNext();
}

char CharIntHashtable::nextKey()
{
  char *key = NULL;
  table_entry *entry = this->internal_next();
  if (entry != NULL){
    key = (char *)entry->key;
  }
  return *key;
}

void *CharIntHashtable::copy_key(const void *key)
{
  char *retKey = (char *)malloc(sizeof(char));
  *retKey = *(char *)key;
  return retKey;
}

bool CharIntHashtable::compare_key(const void *key1, const void *key2)
{
  return (*(char *)key1 == *(char *)key2);
}

void CharIntHashtable::release_key(void *key)
{
  free(key);
}
