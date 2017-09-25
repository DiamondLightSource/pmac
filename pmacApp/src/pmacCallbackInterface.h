/*
 * pmacCallbackInterface.h
 *
 *  Created on: 11 Feb 2016
 *      Author: gnx91527
 */

#ifndef PMACAPP_SRC_PMACCALLBACKINTERFACE_H_
#define PMACAPP_SRC_PMACCALLBACKINTERFACE_H_

#include "pmacCommandStore.h"

class pmacCallbackInterface {
public:
    pmacCallbackInterface();

    virtual ~pmacCallbackInterface();

    virtual void callback(pmacCommandStore *sPtr, int type) = 0;
};

#endif /* PMACAPP_SRC_PMACCALLBACKINTERFACE_H_ */
