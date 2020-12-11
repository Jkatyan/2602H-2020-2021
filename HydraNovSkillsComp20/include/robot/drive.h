#ifndef _DRIVE_H_
#define _DRIVE_H_

#include "api.h"

void initDrive();

void tankDrive();

void brake();

int right_pos();

int left_pos();

void setDriveBrakes (pros::motor_brake_mode_e_t mode);

void reset();

void resetPID();

void driveLeft(double target, int time, float speed);

void driveRight(double target, int time, float speed);

void driveForward(double target, int time, float speed);

void driveBackwards(double target, int time, float speed);

void driveForwardSetState(double target, int state, int time, float speed);

void driveBackwardsSetState(double target, int state, int time, float speed);

void slowDrive(double target, int time, float speed, float endSpeed);

void fastDrive(double target, int speed);

void rotateRaw(double target, int time, float speed);

void rotateImu(double degrees, int time, float speed);

void rotateTracker(double target, int time, float speed);

#endif
