#include "main.h"

//HELPER FUNCTIONS
void setLift(int power){
  liftRight = power;
  liftLeft = power;
}
//DRIVER CONTROL FUNCTIONS
void liftControl(){

}
void liftBrakeMode(pros::motor_brake_mode_e_t mode){
  liftRight.set_brake_mode(mode);
  liftLeft.set_brake_mode(mode);
}
//AUTONOMOUS CONTROL FUNCTIONS
