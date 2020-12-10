#include "main.h"

//HELPER FUNCTIONS
void setClaw(int power);

//DRIVER CONTROL FUNCTIONS
void clawControl();
void clawBrakeMode(pros::motor_brake_mode_e_t mode);
//AUTONOMOUS CONTROL FUNCTIONS

//Claw States
#define START 0
#define OPEN 100
#define CLOSED 300
