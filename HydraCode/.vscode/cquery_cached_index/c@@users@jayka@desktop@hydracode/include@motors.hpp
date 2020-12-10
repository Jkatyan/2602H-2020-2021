#ifndef MOTORS_HPP
#define MOTORS_HPP

#include "main.h"

//PORTS
#define R1 1 //Right Front
#define R2 10 //Right Back
#define L1 11 //Left Front
#define L2 20 //Left Back

#define LIFT 12 //Lift
#define INTA 19 //Diagonal Intake
#define INTB 15 //Vertical Intake
#define DOOR 16 //DOOR

#define GYROPORT 3

//MOTOR CONFIG
/*DIRECTION*/ #define RFREV false
/*true = rev*/#define RBREV false
              #define LFREV true
              #define LBREV true

              #define LIFTREV false
              #define INTAREV false
              #define INTBREV false
              #define DOORREV false

/*GEARSET*/   #define RFG E_MOTOR_GEARSET_18
/*18 = Green*/#define RBG E_MOTOR_GEARSET_18
/*06 = Blue*/ #define LFG E_MOTOR_GEARSET_18
/*36 = Red*/  #define LBG E_MOTOR_GEARSET_18

              #define LIFTG E_MOTOR_GEARSET_18
              #define INTAG E_MOTOR_GEARSET_18
              #define INTBG E_MOTOR_GEARSET_18
              #define DOORG E_MOTOR_GEARSET_18

//CHASSIS CORRECTION
#define RC 0.9927
#define LC 1

#endif
