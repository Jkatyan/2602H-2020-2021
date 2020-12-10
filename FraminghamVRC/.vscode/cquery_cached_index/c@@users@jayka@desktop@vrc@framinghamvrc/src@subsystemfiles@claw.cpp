#include "main.h"

//HELPER FUNCTIONS
void setClaw(int power){
  clawRight = power;
  clawLeft = power;
}
//DRIVER CONTROL FUNCTIONS
void clawControl(){
  int clawPower = 127 * (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1) - master.get_digital(pros::E_CONTROLLER_DIGITAL_L2));
  setClaw(clawPower);
}
void clawBrakeMode(pros::motor_brake_mode_e_t mode){
  clawRight.set_brake_mode(mode);
  clawLeft.set_brake_mode(mode);
}
//AUTONOMOUS CONTROL FUNCTIONS
