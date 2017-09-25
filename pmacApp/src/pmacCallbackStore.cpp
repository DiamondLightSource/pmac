/*
 * pmacCallbackStore.cpp
 *
 *  Created on: 11 Feb 2016
 *      Author: gnx91527
 */

#include "pmacCallbackStore.h"
#include <stdio.h>

pmacCallbackStore::pmacCallbackStore(int type) :
        size(0),
        type(type) {
  callbacks = (pmacCallbackInterface **) malloc(
          MAX_REGISTERED_CALLBACKS * sizeof(pmacCallbackInterface *));
}

pmacCallbackStore::~pmacCallbackStore() {
}

int pmacCallbackStore::registerCallback(pmacCallbackInterface *cbPtr) {
  callbacks[size] = cbPtr;
  size++;
  return 0;
}

int pmacCallbackStore::callCallbacks(pmacCommandStore *sPtr) {
  for (int index = 0; index < size; index++) {
//    printf("Calling back index %d\n", index);
    if (callbacks[index] != NULL) {
      callbacks[index]->callback(sPtr, type);
    }
  }
  return 0;
}

