#ifndef _GLOBALS_HPP_
#define _GLOBALS_HPP_

#include "api.h"

//Drive Motors
extern pros::Motor LF;
extern pros::Motor LB;
extern pros::Motor RF;
extern pros::Motor RB;

//Intake Motors
extern pros::Motor rightIntake;
extern pros::Motor leftIntake;

//Conveyor Motors
extern pros::Motor midRoller;
extern pros::Motor topRoller;

//Sensors
extern pros::ADIAnalogIn line;
extern pros::ADIAnalogIn midLine;
extern pros::ADIEncoder sideEnc;
extern pros::ADIEncoder rightEnc;
extern pros::ADIEncoder leftEnc;

//Misc
extern pros::Controller master;
extern pros::Imu imu;

#endif
