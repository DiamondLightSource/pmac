/*
 * ArrayInfoHashtable.h
 *
 *  Created on: 12 Jun 2026
 *      Author: dcy
 */

#ifndef PMACAPP_SRC_STRINGINTINTHASHTABLE_H_
#define PMACAPP_SRC_STRINGINTINTHASHTABLE_H_

#include "Hashtable.h"
#include <epicsTypes.h>
#include <string>

class ArrayInfoHashtable : public Hashtable {
public:
    ArrayInfoHashtable();

    virtual ~ArrayInfoHashtable();

    bool lookup(int iAsynReason, const char** pszKey, epicsUInt32* puArraySize, int* pbWriteOnly, void** ppArrayCache);

    bool lookup(const std::string &key, int* piAsynReason, epicsUInt32* puArraySize, int* pbWriteOnly, void** ppArrayCache);

    bool insert(const std::string &key, int iAsynReason, epicsUInt32 uArraySize, int bWriteOnly, void* pArrayCache);

    void remove(const std::string &key);

    bool hasKey(const std::string &key);

    std::string firstKey();

    bool hasNextKey();

    std::string nextKey();

    bool writeOnly(const std::string &key);

protected:
    virtual void *copy_key(const void *key);

    virtual bool compare_key(const void *key1, const void *key2);

    virtual void release_key(void *key);

};

#endif /* PMACAPP_SRC_ARRAYINFOHASHTABLE_H_ */
