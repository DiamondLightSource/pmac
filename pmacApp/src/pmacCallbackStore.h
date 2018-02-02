/*
 * pmacCallbackStore.h
 *
 *  Created on: 11 Feb 2016
 *      Author: gnx91527
 */

#ifndef PMACAPP_SRC_PMACCALLBACKSTORE_H_
#define PMACAPP_SRC_PMACCALLBACKSTORE_H_

#include "pmacCallbackInterface.h"
#include "pmacCommandStore.h"

#define MAX_REGISTERED_CALLBACKS 128
#define MAX_REGISTERED_LOCKS 128

class pmacCallbackStore {
public:
    pmacCallbackStore(int type);

    virtual ~pmacCallbackStore();

    int registerCallback(pmacCallbackInterface *cbPtr);

    int callCallbacks(pmacCommandStore *sPtr);

private:
    int size;
    int type;
    pmacCallbackInterface **callbacks;
};

#endif /* PMACAPP_SRC_PMACCALLBACKSTORE_H_ */
