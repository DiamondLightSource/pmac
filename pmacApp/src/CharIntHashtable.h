/*
 * CharIntHashtable.h
 *
 *  Created on: 9 Feb 2016
 *      Author: gnx91527
 */

#ifndef PMACAPP_SRC_CharIntHashtable_H_
#define PMACAPP_SRC_CharIntHashtable_H_

#include "Hashtable.h"

class CharIntHashtable : public Hashtable {
public:
    CharIntHashtable();

    virtual ~CharIntHashtable();

    int lookup(const char key);

    int insert(const char key, int value);

    int remove(const char key);

    char firstKey();

    bool hasNextKey();

    char nextKey();


protected:
    virtual void *copy_key(const void *key);

    virtual bool compare_key(const void *key1, const void *key2);

    virtual void release_key(void *key);

};

#endif /* PMACAPP_SRC_CharIntHashtable_H_ */
