/*
 * pmacGroupsHashtable.h
 *
 *  Created on: 10 Feb 2016
 *      Author: gnx91527
 */

#ifndef PMACAPP_SRC_PMACGROUPSHASHTABLE_H_
#define PMACAPP_SRC_PMACGROUPSHASHTABLE_H_

#include "Hashtable.h"

class pmacGroupsHashtable : public Hashtable {
public:
    pmacGroupsHashtable();

    virtual
    ~pmacGroupsHashtable();

    void *lookup(int key);

    void *insert(int key, void *value);

    void *remove(int key);

    int firstKey();

    bool hasNextKey();

    int nextKey();


protected:
    virtual void *copy_key(const void *key);

    virtual bool compare_key(const void *key1, const void *key2);

    virtual void release_key(void *key);

};

#endif /* PMACAPP_SRC_PMACGROUPSHASHTABLE_H_ */
