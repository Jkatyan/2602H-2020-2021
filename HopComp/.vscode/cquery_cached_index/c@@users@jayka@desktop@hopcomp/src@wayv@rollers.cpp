#include "main.h"

//Intake Motors
pros::Motor rightIntake(RIGHT_INTAKE, RI_REVERSED);
pros::Motor leftIntake(LEFT_INTAKE, LI_REVERSED);

//Conveyor Motors
pros::Motor midRoller(MID_ROLLER, MID_REVERSED);
pros::Motor topRoller(TOP_ROLLER, TOP_REVERSED);

//Sensors
pros::ADIAnalogIn line(LINE_SENSOR);
pros::ADIAnalogIn midLine(MID_LINE_SENSOR);

pros::Controller masterRollers(CONTROLLER_MASTER);

int intakeMode;

void setIntakeBrakes(pros::motor_brake_mode_e_t mode){
  rightIntake.set_brake_mode(mode);
	leftIntake.set_brake_mode(mode);
	midRoller.set_brake_mode(mode);
	topRoller.set_brake_mode(mode);
}

void setState(int state){
  intakeMode = state;
  /*0 = Everything stopped
  * 1 = Front rollers in full speed, everything else stopped.
  * 2 = Front rollers out full speed, everything else stopped.
  * 3 = Front rollers in full speed, rollers rolling up to indexer
  * 4 = Everything out full speed
  * 5 = Everything in full speed
  * 6 = Front rollers out full speed, rollers in full speed
  * 7 = Front rollers opposite directions
  * 8 = Mid and front rollers stopped, Top roller in
  * 9 = Mid roller stopped, front rollers and top in
  * 10 = Top roller stopped, front and mid rollers out
  * 11 = Mid and Top rollers in, front rollers stopped
  * 12 = Mode 3 but without front rollers
  */
}

void setRollers(int rInt, int lInt, int midRoll, int topRoll){
  (rInt == 0) ? rightIntake.move_velocity(0) : rightIntake.move(rInt);
  (lInt == 0) ? leftIntake.move_velocity(0) : leftIntake.move(rInt);
  (midRoll == 0) ? midRoller.move_velocity(0) : midRoller.move(rInt);
  (topRoll == 0) ? topRoller.move_velocity(0) : topRoller.move(rInt);
}

int intakeTask(){
  while(true){
    if(intakeMode == 0){
      setRollers(0, 0, 0, 0);
    }
    else if(intakeMode == 1){
      setRollers(-127, 127, 0, 0);
    }
    else if(intakeMode == 2){
      setRollers(127, -127, 0, 0);
    }
    else if(intakeMode == 3){
      if(line.get_value() >= LINE_LIMIT){
        setRollers(-127, 127, 45, 60);
      }
      else{
        setRollers(-127, 127, 80, -127);
      }
    }
    else if(intakeMode == 4){
      setRollers(127, -127, -127, -127);
    }
    else if(intakeMode == 5){
      setRollers(-127, 127, 127, 127);
    }
    else if(intakeMode == 6){
      setRollers(127, -127, 127, 127);
    }
    else if(intakeMode == 7){
      setRollers(127, 127, 0, 0);
    }
    else if(intakeMode == 8){
      setRollers(0, 0, 0, 127);
    }
    else if(intakeMode == 9){
      setRollers(-127, 127, 0, 127);
    }
    else if(intakeMode == 10){
      setRollers(127, -127, -127, 0);
    }
    else if(intakeMode == 11){
      setRollers(0, 0, 127, 127);
    }
    else if(intakeMode == 12){
      if(line.get_value() >= LINE_LIMIT){
        setRollers(0, 0, 45, 60);
      }
      else{
        setRollers(0, 0, 80, -127);
      }
    }
    else {
      continue;
    }
    pros::delay(10);
  }
}

void initIntakes(){
  setIntakeBrakes(MOTOR_BRAKE_HOLD);
  pros::Task intake_task(intakeTask);
}

void intakeOp(){
  intakeMode = -1;
  if(masterRollers.get_digital(DIGITAL_L1)){
    if(masterRollers.get_digital(DIGITAL_L2)){
      midRoller.move(127);
      topRoller.move(127);
    }
    else if(line.get_value() >= LINE_LIMIT && !(masterRollers.get_digital(DIGITAL_L1) && masterRollers.get_digital(DIGITAL_L2))){
      midRoller.move(60);
      topRoller.move(60);
    }
    else{
      midRoller.move(80);
      topRoller.move(-127);
    }
  }
  else if(masterRollers.get_digital(DIGITAL_L2)){
    midRoller.move(-127);
    topRoller.move(-127);
  }
  else{
    midRoller.move_velocity(0);
    topRoller.move_velocity(0);
  }

  if(masterRollers.get_digital(DIGITAL_R1)){
    if(masterRollers.get_digital(DIGITAL_R2)){
      rightIntake.move(127);
      leftIntake.move(127);
    }
    else{
    rightIntake.move(-127);
    leftIntake.move(127);
    }
  }
  else if(masterRollers.get_digital(DIGITAL_R2)){
    rightIntake.move(127);
    leftIntake.move(-127);
  }
  else{
    rightIntake.move_velocity(0);
    leftIntake.move_velocity(0);
  }
}
