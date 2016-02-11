/*
 * Hashtable.cpp
 *
 *  Created on: 9 Feb 2016
 *      Author: gnx91527
 */

#include "Hashtable.h"
#include <string.h>
#include <stdio.h>

size_t Hashtable::INITIAL_SIZE = 8;
hash_t Hashtable::EMPTY_HASH   = 0;  // Marks unused slot
hash_t Hashtable::DELETED_HASH = -1; // Marks deleted slot, "tombstone"

Hashtable::Hashtable()
{
  this->entries = 0;
  this->deleted = 0;
  this->size_mask = Hashtable::INITIAL_SIZE - 1;
  this->walk_index = 0;
  this->table = (table_entry *)calloc(Hashtable::INITIAL_SIZE, sizeof(struct table_entry));
}

Hashtable::~Hashtable()
{
  free(this->table);
}

void *Hashtable::lookup(const void *key)
{
  bool found;
  return this->lookup(key, this->hash_string(key), &found)->value;
}

void *Hashtable::insert(const void *key, void *value)
{
  hash_t hash = this->hash_string(key);
  bool found;
  struct table_entry *entry = lookup(key, hash, &found);
  // Proper management of deleted and entry counts.
  if (entry->hash == EMPTY_HASH){
    // Adding a new entry.
    this->entries += 1;
  } else if (entry->hash == DELETED_HASH){
    // Overwriting a deleted key.
    this->deleted -= 1;
    // Otherwise updated a value in place, no action required.
  }

  if (!found){
    // New entry, set up key and hash.
    entry->hash = hash;
    entry->key = this->copy_key(key);
  }

  void *old_value = entry->value;
  entry->value = value;

  // Check for over-full hash table, expand if necessary, if more than 2/3 full.
  if (3 * this->entries >= 2 * this->size_mask){
    this->resize(0);
  }
  return old_value;
}

void *Hashtable::remove(const void *key)
{
  bool found;
  void *old_value = NULL;
  struct table_entry *entry = lookup(key, this->hash_string(key), &found);
  if (entry){
    old_value = entry->value;
    if (found){
      release_key(entry->key);
      entry->hash = DELETED_HASH;
      entry->key = NULL;
      entry->value = NULL;
      this->deleted += 1;
    }
  }
  return old_value;
}

size_t Hashtable::count()
{
  return this->entries - this->deleted;
}

void Hashtable::resize(size_t min_size)
{
  // Compute new size taking deleted items into account: next power of two
  // with at least 50% table free.
  size_t entries = this->entries - this->deleted;
  if (min_size < 2 * entries){
    min_size = 2 * entries;
  }
  size_t new_size = Hashtable::INITIAL_SIZE;
  while (new_size < min_size){
    new_size <<= 1;
  }

  table_entry *oldtable = this->table;
  size_t old_size = this->size_mask;
  this->table = (table_entry *)calloc(new_size, sizeof(struct table_entry));
  for (size_t ix = 0; ix < new_size; ix ++){
    this->table->hash = Hashtable::EMPTY_HASH;
  }
  this->size_mask = new_size - 1;
  this->entries = entries;
  this->deleted = 0;

  for (size_t ix = 0; ix <= old_size; ix ++){
    struct table_entry *entry = &oldtable[ix];
    if (!empty_entry(entry)){
      bool found;
      struct table_entry *new_entry = this->lookup(entry->key, entry->hash, &found);
      new_entry->hash = entry->hash;
      new_entry->key = entry->key;
      new_entry->value = entry->value;
    }
  }

  // Release old table resources.
  free(oldtable);
}

table_entry *Hashtable::internal_begin()
{
  this->walk_index = 0;
  return this->internal_next();
}

bool Hashtable::internal_hasNext()
{
  for (size_t ix = this->walk_index; ix <= this->size_mask; ix ++){
    struct table_entry *entry = &this->table[ix];
    if (!empty_entry(entry)){
      return true;
    }
  }
  return false;
}

table_entry *Hashtable::internal_next()
{
  for (size_t ix = this->walk_index; ix <= this->size_mask; ix ++){
    struct table_entry *entry = &this->table[ix];
    if (!empty_entry(entry)){
      this->walk_index = ix+1;
      return entry;
    }
  }
  return NULL;
}

void Hashtable::keys(void **keys)
{
  int keycount = 0;
  keys = (void **)malloc(sizeof(void *)*this->count());
  for (size_t ix = 0; ix <= this->size_mask; ix ++){
    struct table_entry *entry = &this->table[ix];
    printf("ix: %d\n", ix);
    if (!empty_entry(entry)){
      printf("not empty\n");
      keys[keycount] = entry->key;
      printf("key: %s\n", (char *)keys[keycount]);
      keycount++;
    }
  }
}

bool Hashtable::empty_entry(struct table_entry *entry)
{
  return entry->hash == EMPTY_HASH  ||  entry->hash == DELETED_HASH;
}

struct table_entry *Hashtable::lookup(const void *key, hash_t hash, bool *found)
{
  // Walk the hash table taking all bits of the hash value into account.
  // Algorithm taken from Python Objects/dictobject.c:lookupdict. */
  hash_t perturb = hash;
  struct table_entry *deleted_entry = NULL;
  for (size_t ix = (size_t) hash; ; perturb >>= 5, ix = 1 + 5*ix + (size_t) perturb){
    struct table_entry *entry = &this->table[ix & this->size_mask];
    if (entry->hash == hash  &&  this->compare_key(key, entry->key)){
      // Match.
      *found = true;
      return entry;
    } else if (entry->hash == EMPTY_HASH){
      // End of hash chain.  Return this entry, or the first deleted entry
      // if one was found.
      *found = false;
      if (deleted_entry){
        return deleted_entry;
      } else {
        return entry;
      }
    } else if (entry->hash == DELETED_HASH  &&  deleted_entry == NULL){
      deleted_entry = entry;
    }
  }
  return NULL;
}

hash_t Hashtable::hash_string(const void *key)
{
  hash_t retVal = 0;
  const char *s = (const char *)key;
  size_t length = strlen(s);
  if (length == 0){
    retVal = 0;
  } else {
    hash_t hash = (hash_t) *s++ << 7;
    for (size_t i = 1; i < length; i++){
      hash = (1000003 * hash) ^ (hash_t) (unsigned int) *s++;
    }
    retVal = hash ^ length;
  }
  return retVal;
}
