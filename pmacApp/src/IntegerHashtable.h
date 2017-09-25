/*
 * IntegerHashtable.h
 *
 *  Created on: 17 Feb 2016
 *      Author: gnx91527
 */

#ifndef PMACAPP_SRC_INTEGERHASHTABLE_H_
#define PMACAPP_SRC_INTEGERHASHTABLE_H_

#include "Hashtable.h"
#include <string>
#include <stdexcept>

class IntegerHashtable : public Hashtable {
public:
    IntegerHashtable();

    virtual ~IntegerHashtable();

    int lookup(const std::string &key);

    void insert(const std::string &key, int value);

    int remove(const std::string &key);

    bool hasKey(const std::string &key);

    std::string firstKey();

    bool hasNextKey();

    std::string nextKey();

protected:
    virtual void *copy_key(const void *key);

    virtual bool compare_key(const void *key1, const void *key2);

    virtual void release_key(void *key);

};

#endif /* PMACAPP_SRC_INTEGERHASHTABLE_H_ */
