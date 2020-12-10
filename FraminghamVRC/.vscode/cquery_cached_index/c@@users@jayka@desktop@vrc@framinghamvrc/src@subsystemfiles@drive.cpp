#include "main.h"

//HELPER FUNCTIONS
void setDrive(int left, int right){
  lB = LC * left;
  lF = LC * left;
  rB = RC * right;
  rF = RC * right;
}
//DRIVER CONTROL FUNCTIONS
void tankDrive(){
  #define DEADBANDVALUE 1
  int leftJoystick = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
  int rightJoystick = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);
  if (abs(leftJoystick) < DEADBANDVALUE){
    leftJoystick = 0;
  }
  if (abs(rightJoystick) < DEADBANDVALUE){
    rightJoystick = 0;
  }
  setDrive(leftJoystick, rightJoystick);
}
void driveBrakeMode(pros::motor_brake_mode_e_t mode){
  lB.set_brake_mode(mode);
  lF.set_brake_mode(mode);
  rB.set_brake_mode(mode);
  rF.set_brake_mode(mode);
}
//AUTONOMOUS CONTROL FUNCTIONS
