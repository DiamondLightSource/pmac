/*
 * pmacTestingUtilities.h
 *
 *  Created on: 6 Apr 2016
 *      Author: gnx91527
 */

#ifndef PMACAPP_UNITTESTS_PMACTESTINGUTILITIES_H_
#define PMACAPP_UNITTESTS_PMACTESTINGUTILITIES_H_

#include <string>
#include <iostream>
#include <sstream>
#include <fstream>
#include <unistd.h>
#include <ios>

void uniqueAsynPortName(std::string& name);
void process_mem_usage(double& vm_usage, double& resident_set);



#endif /* PMACAPP_UNITTESTS_PMACTESTINGUTILITIES_H_ */
