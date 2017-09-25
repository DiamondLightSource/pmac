/*
 * Hashtable.h
 *
 *  Created on: 9 Feb 2016
 *      Author: gnx91527
 */

#ifndef PMACAPP_SRC_HASHTABLE_H_
#define PMACAPP_SRC_HASHTABLE_H_

#include <stdlib.h>

// Hashes are long integers, 64-bits on the right architecture.
typedef long unsigned int hash_t;

struct table_entry {
    hash_t hash;
    void *key;    // Null if absent
    void *value;
};

class Hashtable {
public:
    Hashtable();

    virtual ~Hashtable();

    // Look up key in hash table, return NULL if not found.
    void *lookup(const void *key);

    // Inserts (key,value) pair in hash table.  If value is already present its old
    // value is returned before being overwritten with the new value, otherwise NULL
    // is returned.
    void *insert(const void *key, void *value);

    // Deletes key from hash table, returning the old value if present, otherwise NULL.
    void *remove(const void *key);

    // Returns the number of entries in the table.
    size_t count();

    // Resizes hash table to have at least the given number of slots.  Can also be
    // used after deleting entries to compress table.
    void resize(size_t min_size);

    table_entry *internal_begin();

    bool internal_hasNext();

    table_entry *internal_next();

    void keys(void **keys);

    // Sanity checking of hash table consistency, raises assert fail if any error is
    // found.  Should only fail in presence of hash table bug, memory overwrite, or
    // key lifetime mismanagement.
    void validate(struct hash_table *table);

protected:
    virtual void *copy_key(const void *key) = 0;

    virtual bool compare_key(const void *key1, const void *key2) = 0;

    virtual void release_key(void *key) = 0;

    bool empty_entry(struct table_entry *entry);

    struct table_entry *lookup(const void *key, hash_t hash, bool *found);

    hash_t hash_string(const void *key);

private:

    static size_t INITIAL_SIZE; // initial size of data store constant
    static hash_t EMPTY_HASH;   // constant to represent an empty hash
    static hash_t DELETED_HASH; // constant to represent a deleted hash

    size_t entries;             // Number of entries in table
    size_t deleted;             // Number of deleted entries in table
    size_t size_mask;           // True size is power of 2, mask selects modulo size
    size_t walk_index;          // Used for keeping track of next and has next
    struct table_entry *table;  // Actual data storage pointer

};

#endif /* PMACAPP_SRC_HASHTABLE_H_ */
