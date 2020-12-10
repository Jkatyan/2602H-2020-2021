#ifndef _DRIVE_OKAPI_H_
#define _DRIVE_OKAPI_H_

#include "api.h"
#include "okapi/api.hpp"
extern okapi::Controller controller;

void left_drive(int power);

void right_drive(int power);

void set_drive(int left, int right);

void brake();

void reset();

void setDriveBrakes(pros::motor_brake_mode_e_t mode);

void driveLeft(double target, int time, float speed);

void driveRight(double target, int time, float speed);

void drive(double target, int time, float speed);

void driveSetState(double target, int state, int time, float speed);

void fastDrive(double target, int speed);

void rotateRaw(double target, int time, float speed);

void rotateImu(double degrees, int time, float speed);

void rotateTracker(double target, int time, float speed);

void okapi_waitUntilSettled();

void okapi_drive(okapi::QLength dist, int rpm);

void okapi_driveAsync(okapi::QLength dist, int rpm);

void okapi_turn(okapi::QAngle deg, int rpm);

void initDrive();

void tankDrive();

void arcadeDrive();

#endif
