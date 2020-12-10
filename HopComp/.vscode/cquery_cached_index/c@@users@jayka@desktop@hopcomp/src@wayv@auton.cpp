#include "main.h"

void runAuton(int auton){
  switch(auton){
    case 0: //Skills
      skills();
    break;
    case 1: //Red Right
      redRight();
    break;
    case 2: //Red Left
      redLeft();
    break;
    case 3: //Do Nothing
      nothing();
    break;
    case -1: //Blue Right
      blueRight();
    break;
    case -2: //Blue Left
      blueLeft();
    break;
    case -3: //Do Nothing
      nothing();
    break;
  }
}

void skills(){
  //Init
  reset();
  setDriveBrakes(MOTOR_BRAKE_COAST);

  //Deploy Score Goal 1
  setState(5);
  pros::delay(200);

  //Go to first ball
  setState(3);
  drive(790, 1500, 1);
  brake();
  reset();

  //Move to corner goal
  rotateImu(-80, 1300, 0.8);
  driveSetState(510, 0, 2000, 0.55);

  //Score Goal 2
  setState(11);
  pros::delay(1000);
  setState(0);
  reset();

  //Back out of goal
  drive(-750, 2000, 0.7);
  rotateImu(73, 1530, 0.8);
  setState(3);
  reset();

  //Intake next ball
  drive(760, 3000, 1);
  reset();

  //Move to goal
  rotateImu(-30, 1300, 0.7);
  reset();
  drive(730, 1500, 0.7);
  setState(0);

  //Score Goal 3
  setState(11);
  pros::delay(700);
  setState(0);
  reset();

  //Back out of goal
  drive(-820, 2000, 0.8);

  //Move to goal
  rotateImu(20, 1000, 1);
  setState(3);
  reset();
  driveSetState(1500, 0, 2000, 0.9);

  //Score Goal 4
  setState(11);
  pros::delay(1000);
  setState(0);
  reset();

  //Back out of goal
  driveSetState(-460, 11, 3000, 0.9);

  //Move to next ball
  rotateImu(185, 1500, 0.7);
  drive(1030, 3000, 0.8);
  reset();

  //Move to goal
  rotateImu(27, 1500, 0.7);
  setState(0);
  drive(700, 3000, 0.7);

  //Score Goal 5
  setState(11);
  pros::delay(500);
  setState(0);
  reset();

  //Back out of goal
  drive(-600, 2000, 0.7);

  //Move to goal
  rotateImu(134, 2000, 0.8);
  reset();
  setState(3);
  drive(1500, 2500, 0.7);

  //Score Goal 6
  setState(6);
  pros::delay(500);
  setState(0);
  reset();

  //Back out of goal
  drive(-350, 2000, 1);
  setState(5);
  pros::delay(500);
  setState(0);

  //Move to goal
  rotateImu(-110, 2000, 0.9);
  reset();
  setState(3);
  drive(2150, 2500, 1);
  rotateImu(-185, 2000, 0.6);
  driveSetState(850, 2, 2000, 0.6);

  //Score Goal 7
  setState(6);
  pros::delay(500);
  setState(0);
  reset();

  //Back out of goal
  drive(-750, 2000, 0.7);
  setState(5);
  pros::delay(500);
  setState(0);
  rotateImu(-13.5, 1500, 1);
  setState(3);
  reset();

  //Get next ball
  drive(1200, 3000, 0.9);
  reset();

  //Drive to goal and descore
  rotateImu(92, 1500, 1);
  driveSetState(900, 2, 2000, 0.8);
  reset();

  //Score Goal 8
  setState(6);
  pros::delay(1000);
  setState(0);
  reset();

  //End
  brake();
}

void redRight(){
  reset();
  setDriveBrakes(MOTOR_BRAKE_COAST);

  //Deploy Score Goal 1
  setState(5);
  pros::delay(200);

  //Go to first ball
  drive(760, 1500, 1);
  brake();
  reset();

  //Move to corner goal
  rotateImu(80, 1300, 0.8);
  driveSetState(560, 3, 2000, 0.55);

  //Score Goal 2
  setState(11);
  pros::delay(500);
  setState(0);
  reset();

  //Back out of goal
  drive(-750, 2000, 0.7);

  //End
  brake();
}

void redLeft(){
  reset();
  setDriveBrakes(MOTOR_BRAKE_COAST);

  //Deploy Score Goal 1
  setState(5);
  pros::delay(200);

  //Go to first ball
  drive(760, 1500, 1);
  brake();
  reset();

  //Move to corner goal
  rotateImu(-80, 1300, 0.8);
  driveSetState(560, 3, 2000, 0.55);

  //Score Goal 2
  setState(11);
  pros::delay(500);
  setState(0);
  reset();

  //Back out of goal
  drive(-750, 2000, 0.7);

  //End
  brake();
}

void blueRight(){
  reset();
  setDriveBrakes(MOTOR_BRAKE_COAST);

  //Deploy Score Goal 1
  setState(5);
  pros::delay(200);

  //Go to first ball
  drive(760, 1500, 1);
  brake();
  reset();

  //Move to corner goal
  rotateImu(80, 1300, 0.8);
  driveSetState(560, 3, 2000, 0.55);

  //Score Goal 2
  setState(11);
  pros::delay(500);
  setState(0);
  reset();

  //Back out of goal
  drive(-750, 2000, 0.7);

  //End
  brake();
}

void blueLeft(){
  reset();
  setDriveBrakes(MOTOR_BRAKE_COAST);

  //Deploy Score Goal 1
  setState(5);
  pros::delay(200);

  //Go to first ball
  drive(760, 1500, 1);
  brake();
  reset();

  //Move to corner goal
  rotateImu(-80, 1300, 0.8);
  driveSetState(560, 3, 2000, 0.55);

  //Score Goal 2
  setState(11);
  pros::delay(500);
  setState(0);
  reset();

  //Back out of goal
  drive(-750, 2000, 0.7);

  //End
  brake();
}

void nothing(){
  pros::delay(15000);
}
