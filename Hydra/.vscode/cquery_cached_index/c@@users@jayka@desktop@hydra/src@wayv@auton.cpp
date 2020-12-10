#include "main.h"

void runAuton(int auton){
  switch(auton){
    case 0: //Skills
      blueLeft();
      //skills();
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
  pros::delay(100);

  //Go to first ball
  setState(3);
  drive(926, 2000, 0.9);
  setState(0);
  stop();
  reset();

  //Move to corner goal
  rotateLeft(-81.5, 2000, 1);
  drive(1300, 5000, 0.55);

  //Score Goal 2
  setState(11);
  pros::delay(500);
  setState(0);
  reset();

  //Back out of goal
  drive(-750, 2000, 0.7);
  rotate(75, 1500, 0.9);
  setState(3);
  reset();

  //Intake next ball
  drive(895, 3000, 0.9);
  reset();

  //Move to goal
  rotate(-38, 1300, 0.7);
  reset();
  drive(1000, 3000, 0.7);

  //Score Goal 3
  setState(6);
  pros::delay(400);
  setState(0);
  reset();

  //Back out of goal
  drive(-920, 2000, 0.8);

  //Move to goal
  rotate(31.5, 1000, 1);
  setState(3);
  reset();
  driveSetState(1780, 0, 2000, 0.9);

  //Score Goal 4
  setState(11);
  pros::delay(500);
  setState(0);
  reset();

  //Back out of goal
  driveSetState(-460, 3, 3000, 0.9);

  //Move to next ball
  rotate(172.4, 1500, 0.7);
  drive(1250, 3000, 0.8);
  reset();

  //Move to goal
  rotate(53, 1500, 0.7);
  setState(0);
  drive(1200, 3000, 0.7);

  //Score Goal 5
  setState(11);
  pros::delay(500);
  setState(0);
  reset();

  //Back out of goal
  drive(-600, 2000, 0.7);

  //Move to goal
  rotate(132, 2000, 0.9);
  reset();
  setState(3);
  drive(1700, 3000, 0.7);

  //Score Goal 6
  setState(6);
  pros::delay(450);
  setState(0);
  reset();

  //Back out of goal
  drive(-350, 2000, 1);
  setState(5);
  pros::delay(500);
  setState(0);

  //Move to goal
  rotate(-110, 2000, 0.9);
  reset();
  setState(3);
  drive(2150, 2500, 1);
  rotate(-188, 2000, 0.6);
  driveSetState(1000, 2, 2000, 0.9);

  //Score Goal 7
  setState(6);
  pros::delay(450);
  setState(0);
  reset();

  //Back out of goal
  drive(-750, 2000, 0.7);
  setState(5);
  pros::delay(500);
  setState(0);
  rotate(-12.8, 1500, 1);
  setState(3);
  reset();

  //Get next ball
  drive(1425, 3000, 0.9);
  reset();

  //Drive to goal and descore
  rotate(112, 1500, 1);
  drive(900, 2000, 1);
  setState(2);
  reset();
  setState(2);

  //Score Goal 8
  setState(6);
  pros::delay(1000);

  //End
  stop();

}

void redRight(){

}

void redLeft(){

}

void blueRight(){

}

void blueLeft(){
  reset();
  setDriveBrakes(MOTOR_BRAKE_COAST);

  //Deploy Score Goal
  setState(5);
  pros::delay(100);

  //Go to first ball
  setState(0);
  drive(775, 2000, 1);
  setState(3);
  reset();

  //Move to corner goal
  rotate(-90, 1500, 1);
  reset();
  drive(1300, 3000, 0.55);

  //Score Goal 2
  setState(6);
  pros::delay(500);
  setState(0);
  reset();

  //Back out of goal
  drive(-1150, 2500, 0.7);
  rotate(90, 1500, 0.9);
  setState(3);
  reset();

  //Move to center goal
  drive(775, 2000, 1);
  reset();

  //Score Mid
  setState(11);
  pros::delay(500);
  setState(0);
  reset();
}

void nothing(){
  pros::delay(15000);
}
