/*
 * test_PMACTrajectory.cpp
 *
 *  Created on: 26 Oct 2016
 *      Author: gnx91527
 */


#include <stdio.h>


#include "boost/test/unit_test.hpp"

#include <string.h>
#include <stdint.h>

#include <deque>
#include <tr1/memory>
#include <iostream>
#include <fstream>

#include "pmacTrajectory.h"

struct PMACTrajectoryFixture
{
  pmacTrajectory trajectory;

  PMACTrajectoryFixture()
  {
  }

  ~PMACTrajectoryFixture()
  {
  }
};

BOOST_FIXTURE_TEST_SUITE(PMACTrajectoryTest, PMACTrajectoryFixture)

BOOST_AUTO_TEST_CASE(test_PMACTrajectory)
{
  // Verify the initial values
  BOOST_CHECK_EQUAL(trajectory.getNoOfAxes(), 9);
  BOOST_CHECK_EQUAL(trajectory.getNoOfValidPoints(), 0);
  BOOST_CHECK_EQUAL(trajectory.getTotalNoOfPoints(), 0);

  // Initialise to 10000 points
  trajectory.initialise(10000);

  // Verify the initialised values
  BOOST_CHECK_EQUAL(trajectory.getNoOfAxes(), 9);
  BOOST_CHECK_EQUAL(trajectory.getNoOfValidPoints(), 0);
  BOOST_CHECK_EQUAL(trajectory.getTotalNoOfPoints(), 10000);

  // Set up some values
  int vel[10] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10};
  int user[10] = {10, 9, 8, 7, 6, 5, 4, 3, 2, 1};
  double time[10] = {21, 22, 23, 24, 25, 26, 27, 28, 29, 30};

  double *pos[9];
  for (int axis = 0; axis < 9; axis++){
    pos[axis] = (double *)malloc(sizeof(double) * 10);
    for (int index = 0; index < 10; index++){
      pos[axis][index] = (double)((axis*10)+index);
    }
  }
  // Append the values
  trajectory.append(pos, time, user, vel, 10);

  // Verify the indexes after append
  BOOST_CHECK_EQUAL(trajectory.getNoOfAxes(), 9);
  BOOST_CHECK_EQUAL(trajectory.getNoOfValidPoints(), 10);
  BOOST_CHECK_EQUAL(trajectory.getTotalNoOfPoints(), 10000);

  // Append the values
  trajectory.append(pos, time, user, vel, 10);

  // Verify the indexes after append
  BOOST_CHECK_EQUAL(trajectory.getNoOfAxes(), 9);
  BOOST_CHECK_EQUAL(trajectory.getNoOfValidPoints(), 20);
  BOOST_CHECK_EQUAL(trajectory.getTotalNoOfPoints(), 10000);

  // Try to read a point outside of range
  int timeVal;
  BOOST_CHECK_EQUAL(trajectory.getTime(-1, &timeVal), asynError);
  BOOST_CHECK_EQUAL(trajectory.getTime(20, &timeVal), asynError);
  int userVal;
  BOOST_CHECK_EQUAL(trajectory.getUserMode(-1, &userVal), asynError);
  BOOST_CHECK_EQUAL(trajectory.getUserMode(20, &userVal), asynError);
  int velocityVal;
  BOOST_CHECK_EQUAL(trajectory.getVelocityMode(-1, &velocityVal), asynError);
  BOOST_CHECK_EQUAL(trajectory.getVelocityMode(20, &velocityVal), asynError);
  double positionVal;
  BOOST_CHECK_EQUAL(trajectory.getPosition(-1, 0, &positionVal), asynError);
  BOOST_CHECK_EQUAL(trajectory.getPosition(9, 0, &positionVal), asynError);
  BOOST_CHECK_EQUAL(trajectory.getPosition(0, -1, &positionVal), asynError);
  BOOST_CHECK_EQUAL(trajectory.getPosition(0, 20, &positionVal), asynError);

  // Now check some valid read outs
  BOOST_CHECK_EQUAL(trajectory.getTime(5, &timeVal), asynSuccess);
  BOOST_CHECK_EQUAL(timeVal, 26);
  BOOST_CHECK_EQUAL(trajectory.getTime(18, &timeVal), asynSuccess);
  BOOST_CHECK_EQUAL(timeVal, 29);
  BOOST_CHECK_EQUAL(trajectory.getUserMode(2, &userVal), asynSuccess);
  BOOST_CHECK_EQUAL(userVal, 8);
  BOOST_CHECK_EQUAL(trajectory.getUserMode(19, &userVal), asynSuccess);
  BOOST_CHECK_EQUAL(userVal, 1);
  BOOST_CHECK_EQUAL(trajectory.getVelocityMode(4, &velocityVal), asynSuccess);
  BOOST_CHECK_EQUAL(velocityVal, 5);
  BOOST_CHECK_EQUAL(trajectory.getVelocityMode(14, &velocityVal), asynSuccess);
  BOOST_CHECK_EQUAL(velocityVal, 5);
  BOOST_CHECK_EQUAL(trajectory.getPosition(2, 5, &positionVal), asynSuccess);
  BOOST_CHECK_EQUAL(positionVal, 25);
  BOOST_CHECK_EQUAL(trajectory.getPosition(4, 6, &positionVal), asynSuccess);
  BOOST_CHECK_EQUAL(positionVal, 46);

  // Append points up to the maximum
  for (int index = 0; index < 998; index++){
    trajectory.append(pos, time, user, vel, 10);
  }

  // Verify the indexes after append
  BOOST_CHECK_EQUAL(trajectory.getNoOfAxes(), 9);
  BOOST_CHECK_EQUAL(trajectory.getNoOfValidPoints(), 10000);
  BOOST_CHECK_EQUAL(trajectory.getTotalNoOfPoints(), 10000);

  // Try to append once more and verify it fails
  BOOST_CHECK_EQUAL(trajectory.append(pos, time, user, vel, 10), asynError);
  BOOST_CHECK_EQUAL(trajectory.getNoOfAxes(), 9);
  BOOST_CHECK_EQUAL(trajectory.getNoOfValidPoints(), 10000);
  BOOST_CHECK_EQUAL(trajectory.getTotalNoOfPoints(), 10000);

  // Re-initialise and check indexes are reset
  trajectory.initialise(200);
  // Verify the initialised values
  BOOST_CHECK_EQUAL(trajectory.getNoOfAxes(), 9);
  BOOST_CHECK_EQUAL(trajectory.getNoOfValidPoints(), 0);
  BOOST_CHECK_EQUAL(trajectory.getTotalNoOfPoints(), 200);

}

BOOST_AUTO_TEST_SUITE_END()


