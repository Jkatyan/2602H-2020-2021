#ifndef _DRIVE_OKAPI_H_
#define _DRIVE_OKAPI_H_

#include "api.h"
#include "okapi/api.hpp"
extern okapi::Controller controller;

void left_drive(int power);

void right_drive(int power);

void set_drive(int left, int right);

void stop();

void setDriveBrakes(pros::motor_brake_mode_e_t mode);

void reset();

void drive(double target, int time, float speed);

void driveSetState(double target, int state, int time, float speed);

void driveAngle(double target, double heading, int time, float speed);

void fastDrive(double target, int speed); 

void rotate(double target, int time, float speed);

void rotateRight(double target, int time, float speed);

void rotateLeft(double target, int time, float speed);

void okapi_waitUntilSettled();

void okapi_resetPos();

void okapi_setPos(okapi::QLength x, okapi::QLength y, okapi::QLength deg);

void okapi_drive(okapi::QLength dist, int rpm);

void okapi_driveAsync(okapi::QLength dist, int rpm);

void okapi_driveToPoint(okapi::QLength x, okapi::QLength y, int rpm);

void okapi_driveToPoint(okapi::QLength x, okapi::QLength y, okapi::QAngle angle, int rpm);

void okapi_turn(okapi::QAngle deg, int rpm);

void okapi_turnToAngle(okapi::QAngle deg, int rpm);

void okapi_arc(okapi::QLength x, okapi::QLength y, okapi::QAngle deg, bool reversed, int rpm);

void initDrive();

void tankDrive();

void arcadeDrive();

#endif
