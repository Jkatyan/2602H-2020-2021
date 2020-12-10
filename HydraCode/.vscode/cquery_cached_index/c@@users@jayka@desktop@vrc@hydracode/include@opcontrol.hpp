#ifndef OPCONTROL_HPP
#define OPCONTROL_HPP

#include "main.h"
#include "misc/PID.hpp"

int Brian = 1;
int setMotor = 1; //Resets Motor Positions
int trayMove = 0;

PID liftPID;
PID trayPID;
//PID intakeOnePID
//PID intakeTwoPID
int lastTime = 0;
double liftTarget;

enum positionsTray {
	rest = 0,
  deposit = 500
} positionsTray;

enum positionsLift {
	ground = 0,
  low = 250,
  high = 500
} positionsLift;

#endif
