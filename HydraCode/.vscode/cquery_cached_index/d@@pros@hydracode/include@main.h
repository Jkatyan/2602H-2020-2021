//PROS CODE
#ifndef _PROS_MAIN_H_
#define _PROS_MAIN_H_
#define PROS_USE_SIMPLE_NAMES
#define PROS_USE_LITERALS

//LIBRARIES
#include "api.h"
#include "okapi/api.hpp"
#include "robotConfig.h"

//PORTS
#define R1 1 //Right Front
#define R2 10 //Right Back
#define L1 11 //Left Front
#define L2 20 //Left Back

#define LIFT 12 //Lift
#define INTA 19 //Diagonal Intake
#define INTB 15 //Vertical Intake
#define DOOR 16 //DOOR

extern pros::Controller master;
extern pros::Motor lB;
extern pros::Motor lF;
extern pros::Motor rB;
extern pros::Motor rF;
extern pros::Motor door;
extern pros::Motor diag;
extern pros::Motor vert;
extern pros::Motor lift;

//PROS CODE
#ifdef __cplusplus
extern "C" {
#endif
void autonomous(void);
void initialize(void);
void disabled(void);
void competition_initialize(void);
void opcontrol(void);
#ifdef __cplusplus
}
#endif
#ifdef __cplusplus
#endif
#endif
