#include "main.h"

pros::Motor rightIntake(RIGHT_INTAKE);
pros::Motor leftIntake(LEFT_INTAKE);
pros::Motor midRoller(MID_ROLLER);
pros::Motor topRoller(TOP_ROLLER);

pros::ADIAnalogIn line(LINE_SENSOR);
pros::ADIAnalogIn mid_line(MID_LINE_SENSOR);
pros::Controller master(CONTROLLER_MASTER);

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
  * 13 = Scores, delays the front rollers
  */
}

int intakeTask(){
  while(true){
    if(intakeMode == 0){
      rightIntake.move_velocity(0);
      leftIntake.move_velocity(0);
      midRoller.move_velocity(0);
      topRoller.move_velocity(0);
    }
    else if(intakeMode == 1){
      rightIntake.move(-127);
      leftIntake.move(127);
      midRoller.move_velocity(0);
      topRoller.move_velocity(0);
    }
    else if(intakeMode == 2){
      rightIntake.move(127);
      leftIntake.move(-127);
      midRoller.move_velocity(0);
      topRoller.move_velocity(0);
    }
    else if(intakeMode == 3){
      if(line.get_value() >= LINE_LIMIT){
        midRoller.move(45);
        topRoller.move(60);
        rightIntake.move(-127);
        leftIntake.move(127);
      }
      else{
        midRoller.move(80);
        topRoller.move(-127);
        rightIntake.move(-127);
        leftIntake.move(127);
      }
    }
    else if(intakeMode == 4){
      rightIntake.move(127);
      leftIntake.move(-127);
      midRoller.move(-127);
      topRoller.move(-127);
    }
    else if(intakeMode == 5){
      rightIntake.move(-127);
      leftIntake.move(127);
      midRoller.move(70);
      topRoller.move(MAX_SCORE_SPEED);
    }
    else if(intakeMode == 6){
      rightIntake.move(127);
      leftIntake.move(-127);
      midRoller.move(MAX_SCORE_SPEED);
      topRoller.move(MAX_SCORE_SPEED);
    }
    else if(intakeMode == 7){
      rightIntake.move(127);
      leftIntake.move(127);
      midRoller.move_velocity(0);
      topRoller.move_velocity(0);
    }
    else if(intakeMode == 8){
      rightIntake.move(0);
      leftIntake.move(0);
      midRoller.move(0);
      topRoller.move(MAX_SCORE_SPEED);
    }
    else if(intakeMode == 9){
      rightIntake.move(-127);
      leftIntake.move(127);
      midRoller.move(0);
      topRoller.move(MAX_SCORE_SPEED);
    }
    else if(intakeMode == 10){
      rightIntake.move(127);
      leftIntake.move(-127);
      midRoller.move(-127);
      topRoller.move_velocity(0);
    }
    else if(intakeMode == 11){
      rightIntake.move_velocity(0);
      leftIntake.move_velocity(0);
      midRoller.move(MAX_SCORE_SPEED);
      topRoller.move_velocity(MAX_SCORE_SPEED);
    }
    else if(intakeMode == 12){
      if(line.get_value() >= LINE_LIMIT){
        midRoller.move(45);
        topRoller.move(60);
        rightIntake.move_velocity(0);
        leftIntake.move_velocity(0);
      }
      else{
        midRoller.move(80);
        topRoller.move(-127);
        rightIntake.move_velocity(0);
        leftIntake.move_velocity(0);
      }
    }
    else if(intakeMode == 13){
      rightIntake.move_velocity(0);
      leftIntake.move_velocity(0);
      midRoller.move(20);
      topRoller.move(MAX_SCORE_SPEED);
      pros::delay(250);
      while(intakeMode == 13){
        rightIntake.move(0);
        leftIntake.move(0);
        midRoller.move(MAX_SCORE_SPEED);
        topRoller.move(MAX_SCORE_SPEED);
        pros::delay(200);
        while(intakeMode == 13){
          rightIntake.move(-127);
          leftIntake.move(127);
          midRoller.move(MAX_SCORE_SPEED);
          topRoller.move(MAX_SCORE_SPEED);
        }
      }
    }
    else {
      continue;
    }
    pros::delay(20);
  }
}

void initIntakes(){
  setIntakeBrakes(MOTOR_BRAKE_HOLD);
  pros::Task intake_task(intakeTask);
}

void intakeOp(){
  intakeMode = -1;
  if(mid_line.get_value() >= LINE_LIMIT){
    if(master.get_digital(DIGITAL_L1)){
      if(master.get_digital(DIGITAL_L2)){
        midRoller.move(100);
        topRoller.move(127);
      }
      else if(line.get_value() >= LINE_LIMIT && !(master.get_digital(DIGITAL_L1) && master.get_digital(DIGITAL_L2))){
        midRoller.move(45);
        topRoller.move(60);
      }
      else{
        midRoller.move(80);
        topRoller.move(-127);
      }
    }
    else if(master.get_digital(DIGITAL_L2)){
      midRoller.move(-127);
      topRoller.move(-127);
    }
    else{
      midRoller.move_velocity(0);
      topRoller.move_velocity(0);
    }

    if(master.get_digital(DIGITAL_R1)){
      if(master.get_digital(DIGITAL_R2)){
        rightIntake.move(127);
        leftIntake.move(127);
      }
      else{
      rightIntake.move(-127);
      leftIntake.move(127);
      }
    }
    else if(master.get_digital(DIGITAL_R2)){
      rightIntake.move(127);
      leftIntake.move(-127);
    }
    else{
      rightIntake.move_velocity(0);
      leftIntake.move_velocity(0);
    }
  }
  else{
    if(master.get_digital(DIGITAL_L1)){
      if(master.get_digital(DIGITAL_L2)){
        midRoller.move(100);
        topRoller.move(127);
      }
      else if(line.get_value() >= LINE_LIMIT && !(master.get_digital(DIGITAL_L1) && master.get_digital(DIGITAL_L2))){
        midRoller.move(60);
        topRoller.move(60);
      }
      else{
        midRoller.move(80);
        topRoller.move(-127);
      }
    }
    else if(master.get_digital(DIGITAL_L2)){
      midRoller.move(-127);
      topRoller.move(-127);
    }
    else{
      midRoller.move_velocity(0);
      topRoller.move_velocity(0);
    }

    if(master.get_digital(DIGITAL_R1)){
      if(master.get_digital(DIGITAL_R2)){
        rightIntake.move(127);
        leftIntake.move(127);
      }
      else{
      rightIntake.move(-127);
      leftIntake.move(127);
      }
    }
    else if(master.get_digital(DIGITAL_R2)){
      rightIntake.move(127);
      leftIntake.move(-127);
    }
    else{
      rightIntake.move_velocity(0);
      leftIntake.move_velocity(0);
    }
  }
}
