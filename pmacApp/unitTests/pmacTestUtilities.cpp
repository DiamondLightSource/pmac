/*
 * pmacTestUtilities.cpp
 *
 *  Created on: 6 Apr 2016
 *      Author: gnx91527
 */

#include "pmacTestingUtilities.h"

void uniqueAsynPortName(std::string& name)
{
  static unsigned long counter = 0;
  std::stringstream ss;

  ss << name << "_" << counter;
  name = ss.str();
  counter++;
}


