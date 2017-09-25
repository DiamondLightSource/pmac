/*
 * StringHashtable.h
 *
 *  Created on: 9 Feb 2016
 *      Author: gnx91527
 */

#ifndef PMACAPP_SRC_STRINGHASHTABLE_H_
#define PMACAPP_SRC_STRINGHASHTABLE_H_

#include "Hashtable.h"
#include <string>

class StringHashtable : public Hashtable {
public:
    StringHashtable();

    virtual ~StringHashtable();

    std::string lookup(const std::string &key);

    std::string insert(const std::string &key, const std::string &value);

    std::string remove(const std::string &key);

    bool hasKey(const std::string &key);

    std::string firstKey();

    bool hasNextKey();

    std::string nextKey();


protected:
    virtual void *copy_key(const void *key);

    virtual bool compare_key(const void *key1, const void *key2);

    virtual void release_key(void *key);

};

#endif /* PMACAPP_SRC_STRINGHASHTABLE_H_ */
