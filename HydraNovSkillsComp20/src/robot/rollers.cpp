#include "main.h"

pros::Motor rightIntake(RIGHT_INTAKE);
pros::Motor leftIntake(LEFT_INTAKE);
pros::Motor midRoller(MID_ROLLER);
pros::Motor topRoller(TOP_ROLLER);

pros::ADIAnalogIn line(LINE_SENSOR);
pros::ADIAnalogIn mid_line(MID_LINE_SENSOR);
pros::Controller masterRollers(CONTROLLER_MASTER);

pros::Optical optical(OPTICAL_PORT);
int color = -1;
int hue = 0;

#define PURPLE 320
#define ORANGE 30
#define GREEN 100

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
  * 14 = Intakes until it sees a red ball, at which point it spins the intakes out and stops intaking.
  * 15 = Mid and Top rollers in, Intakes until it sees a red ball, at which point it spins the intakes out and stops intaking.
  * 16 = Front rollers stopped, rollers rolling up to indexer
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
    else if(intakeMode == 15){
        hue = optical.get_hue();
        if (hue > PURPLE || hue < ORANGE){
          color = 0;
        }
        else if (hue > GREEN && hue < PURPLE){
          color = 1;
        }
        else {
          color = -1;
        }

        if (color == 1){
          if(line.get_value() >= LINE_LIMIT){
            midRoller.move(MAX_SCORE_SPEED);
            topRoller.move(MAX_SCORE_SPEED);
            rightIntake.move(-60);
            leftIntake.move(60);
          }
          else{
            midRoller.move(MAX_SCORE_SPEED);
            topRoller.move(MAX_SCORE_SPEED);
            rightIntake.move(-60);
            leftIntake.move(60);
          }
        }
        if (color == -1){
          if(line.get_value() >= LINE_LIMIT){
            midRoller.move(MAX_SCORE_SPEED);
            topRoller.move(MAX_SCORE_SPEED);
            rightIntake.move(-127);
            leftIntake.move(127);
          }
          else{
            midRoller.move(MAX_SCORE_SPEED);
            topRoller.move(MAX_SCORE_SPEED);
            rightIntake.move(-127);
            leftIntake.move(127);
          }
        }
        else if (color == 0){
          rightIntake.move(127);
          leftIntake.move(-127);
          if(line.get_value() >= LINE_LIMIT){
            midRoller.move(MAX_SCORE_SPEED);
            topRoller.move(MAX_SCORE_SPEED);
          }
          else{
            midRoller.move(MAX_SCORE_SPEED);
            topRoller.move(MAX_SCORE_SPEED);
          }
          pros::delay(100);
          intakeMode = 11;
        }
    }
    else if(intakeMode == 14){
        hue = optical.get_hue();
        if (hue > PURPLE || hue < ORANGE){
          color = 0;
        }
        else if (hue > GREEN && hue < PURPLE){
          color = 1;
        }
        else {
          color = -1;
        }

        if (color == 1){
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
        if (color == -1){
          if(line.get_value() >= LINE_LIMIT){
            midRoller.move(45);
            topRoller.move(60);
            rightIntake.move(-60);
            leftIntake.move(60);
          }
          else{
            midRoller.move(80);
            topRoller.move(-127);
            rightIntake.move(-60);
            leftIntake.move(60);
          }
        }
        else if (color == 0){
          rightIntake.move(127);
          leftIntake.move(-127);
          if(line.get_value() >= LINE_LIMIT){
            midRoller.move(45);
            topRoller.move(60);
          }
          else{
            midRoller.move(80);
            topRoller.move(-127);
          }
          pros::delay(100);
          intakeMode = 16;
        }
    }
    else if(intakeMode == 16){
      if(line.get_value() >= LINE_LIMIT){
        midRoller.move(45);
        topRoller.move(60);
        rightIntake.move(0);
        leftIntake.move(0);
      }
      else{
        midRoller.move(80);
        topRoller.move(-127);
        rightIntake.move(0);
        leftIntake.move(0);
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
  optical.set_led_pwm(100);
  pros::Task intake_task(intakeTask);
}

void intakeOp(){
  intakeMode = -1;
  if(mid_line.get_value() >= LINE_LIMIT){
    if(masterRollers.get_digital(DIGITAL_L1)){
      if(masterRollers.get_digital(DIGITAL_L2)){
        midRoller.move(100);
        topRoller.move(127);
      }
      else if(line.get_value() >= LINE_LIMIT && !(masterRollers.get_digital(DIGITAL_L1) && masterRollers.get_digital(DIGITAL_L2))){
        midRoller.move(45);
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
  else{
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
}
