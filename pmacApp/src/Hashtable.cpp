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
hash_t Hashtable::EMPTY_HASH = 0;  // Marks unused slot
hash_t Hashtable::DELETED_HASH = -1; // Marks deleted slot, "tombstone"

/**
 * Constructor.  Takes no parameters.  Allocates memory for the initial size of
 * the table as specified by the INITIAL_SIZE constant.  Initialises all other
 * private member variables.
 */
Hashtable::Hashtable() {
  this->entries = 0;
  this->deleted = 0;
  this->size_mask = Hashtable::INITIAL_SIZE - 1;
  this->walk_index = 0;
  this->table = (table_entry *) calloc(Hashtable::INITIAL_SIZE, sizeof(struct table_entry));
}

/**
 * Destructor.  Takes no parameters.  Frees the previously allocated table memory.
 */
Hashtable::~Hashtable() {
  free(this->table);
}

/**
 * Looks up the value of an entry according to the supplied key. If there is no entry
 * found then the return value is NULL, otherwise the value is returned as a void ptr.
 *
 * @param key The key used to lookup the value.
 * @return value
 */
void *Hashtable::lookup(const void *key) {
  bool found;
  return this->lookup(key, this->hash_string(key), &found)->value;
}

/**
 * Inserts a new key, value pair into the hashtable. If an existing value already
 * exists for the key then the old value is returned, otherwise the method returns
 * NULL.
 *
 * @param key The key used to index into the hashtable.
 * @param value The value to store in the hashtable.
 * @return existing value or NULL
 */
void *Hashtable::insert(const void *key, void *value) {
  hash_t hash = this->hash_string(key);
  bool found;
  struct table_entry *entry = lookup(key, hash, &found);
  // Proper management of deleted and entry counts.
  if (entry->hash == EMPTY_HASH) {
    // Adding a new entry.
    this->entries += 1;
  } else if (entry->hash == DELETED_HASH) {
    // Overwriting a deleted key.
    this->deleted -= 1;
    // Otherwise updated a value in place, no action required.
  }

  if (!found) {
    // New entry, set up key and hash.
    entry->hash = hash;
    entry->key = this->copy_key(key);
  }

  void *old_value = entry->value;
  entry->value = value;

  // Check for over-full hash table, expand if necessary, if more than 2/3 full.
  if (3 * this->entries >= 2 * this->size_mask) {
    this->resize(0);
  }
  return old_value;
}

/**
 * Removes a key, value pair from the hashtable. The value is returned if it exists,
 * otherwise the method returns NULL.
 *
 * @param key The key used to index into the hashtable.
 * @return removed value or NULL
 */
void *Hashtable::remove(const void *key) {
  bool found;
  void *old_value = NULL;
  struct table_entry *entry = lookup(key, this->hash_string(key), &found);
  if (entry) {
    old_value = entry->value;
    if (found) {
      release_key(entry->key);
      entry->hash = DELETED_HASH;
      entry->key = NULL;
      entry->value = NULL;
      this->deleted += 1;
    }
  }
  return old_value;
}

/**
 * Returns the current count of items in the hashtable.
 *
 * @return size of hashtable
 */
size_t Hashtable::count() {
  return this->entries - this->deleted;
}

/**
 * Resizes the hashtable, re-allocates memory and frees up memory
 * from the original table
 *
 * @param min_size The minimum size of the new hashtable.
 *
 */
void Hashtable::resize(size_t min_size) {
  // Compute new size taking deleted items into account: next power of two
  // with at least 50% table free.
  size_t entries = this->entries - this->deleted;
  if (min_size < 2 * entries) {
    min_size = 2 * entries;
  }
  size_t new_size = Hashtable::INITIAL_SIZE;
  while (new_size < min_size) {
    new_size <<= 1;
  }

  table_entry *oldtable = this->table;
  size_t old_size = this->size_mask;
  this->table = (table_entry *) calloc(new_size, sizeof(struct table_entry));
  for (size_t ix = 0; ix < new_size; ix++) {
    this->table->hash = Hashtable::EMPTY_HASH;
  }
  this->size_mask = new_size - 1;
  this->entries = entries;
  this->deleted = 0;

  for (size_t ix = 0; ix <= old_size; ix++) {
    struct table_entry *entry = &oldtable[ix];
    if (!empty_entry(entry)) {
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

/**
 * Used for iterating over the hashtable. Returns the table entry
 * item from the beginning of the hashtable.
 *
 * @return first entry from the hashtable
 */
table_entry *Hashtable::internal_begin() {
  this->walk_index = 0;
  return this->internal_next();
}

/**
 * Used for iterating over the hashtable.  Checks if there is another
 * entry in the hashtable after the current entry.
 *
 * @return true if there is another entry, false otherwise
 */
bool Hashtable::internal_hasNext() {
  for (size_t ix = this->walk_index; ix <= this->size_mask; ix++) {
    struct table_entry *entry = &this->table[ix];
    if (!empty_entry(entry)) {
      return true;
    }
  }
  return false;
}

/**
 * Used for iterating over the hashtable.  Returns the next entry in the
 * hashtable.  If there is no entry then this returns NULL.
 *
 * @return next entry from the hashtable
 */
table_entry *Hashtable::internal_next() {
  for (size_t ix = this->walk_index; ix <= this->size_mask; ix++) {
    struct table_entry *entry = &this->table[ix];
    if (!empty_entry(entry)) {
      this->walk_index = ix + 1;
      return entry;
    }
  }
  return NULL;
}

/**
 * Returns an array of the keys from the hashtable.
 *
 * @param keys The pointer to array of keys.
 */
void Hashtable::keys(void **keys) {
  int keycount = 0;
  keys = (void **) malloc(sizeof(void *) * this->count());
  for (size_t ix = 0; ix <= this->size_mask; ix++) {
    struct table_entry *entry = &this->table[ix];
    //printf("ix: %d\n", ix);
    if (!empty_entry(entry)) {
      //printf("not empty\n");
      keys[keycount] = entry->key;
      //printf("key: %s\n", (char *)keys[keycount]);
      keycount++;
    }
  }
}

/**
 * Checks if a particular hashtable entry is empty or deleted.
 *
 * @param entry The table entry to check.
 * @return true if the entry is empty, false otherwise
 */
bool Hashtable::empty_entry(struct table_entry *entry) {
  return entry->hash == EMPTY_HASH || entry->hash == DELETED_HASH;
}

/**
 * Looks for a key/hash within the hashtable and returns the hashtable
 * entry item if it is found.  Also
 *
 * @param key The key to search for.
 * @param hash The hash of the key to search for.
 * @param found Return boolean true if key found, false otherwise.
 * @return table entry if found, NULL otherwise.
 */
struct table_entry *Hashtable::lookup(const void *key, hash_t hash, bool *found) {
  // Walk the hash table taking all bits of the hash value into account.
  // Algorithm taken from Python Objects/dictobject.c:lookupdict. */
  hash_t perturb = hash;
  struct table_entry *deleted_entry = NULL;
  for (size_t ix = (size_t) hash;; perturb >>= 5, ix = 1 + 5 * ix + (size_t) perturb) {
    struct table_entry *entry = &this->table[ix & this->size_mask];
    if (entry->hash == hash && this->compare_key(key, entry->key)) {
      // Match.
      *found = true;
      return entry;
    } else if (entry->hash == EMPTY_HASH) {
      // End of hash chain.  Return this entry, or the first deleted entry
      // if one was found.
      *found = false;
      if (deleted_entry) {
        return deleted_entry;
      } else {
        return entry;
      }
    } else if (entry->hash == DELETED_HASH && deleted_entry == NULL) {
      deleted_entry = entry;
    }
  }
  return NULL;
}

/**
 * Turns a given key into a hash value.
 *
 * @param key The key to turn into a hash.
 * @return hash value of the key.
 */
hash_t Hashtable::hash_string(const void *key) {
  hash_t retVal = 0;
  const char *s = (const char *) key;
  size_t length = strlen(s);
  if (length == 0) {
    retVal = 0;
  } else {
    hash_t hash = (hash_t) *s++ << 7;
    for (size_t i = 1; i < length; i++) {
      hash = (1000003 * hash) ^ (hash_t) (unsigned int) *s++;
    }
    retVal = hash ^ length;
  }
  return retVal;
}
