#include "api.h"

#ifndef ROBOT_H
#define ROBOT_H

using namespace pros;

//Devices
extern Controller master;

extern Motor leftFront;
extern Motor leftBack;
extern Motor rightFront;
extern Motor rightBack;

extern Motor leftIntake;
extern Motor rightIntake;
extern Motor midRoller;
extern Motor topRoller;

extern Imu imuA;
extern Imu imuB;

extern ADIEncoder sEncoder;
extern ADIEncoder rEncoder;

//Tasks
extern Task *tracking;

#endif
