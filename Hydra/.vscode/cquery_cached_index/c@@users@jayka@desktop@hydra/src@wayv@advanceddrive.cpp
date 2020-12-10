#include "main.h"

PID aDistancePID (CUSTOM_DKP, CUSTOM_DKD);
PID aAnglePID (CUSTOM_AKP, CUSTOM_AKD);

pros::Motor aLF (abs(LEFT_FRONT), MOTOR_GEARSET_18, true, MOTOR_ENCODER_COUNTS);
pros::Motor aLB (abs(LEFT_BACK), MOTOR_GEARSET_18, true, MOTOR_ENCODER_COUNTS);
pros::Motor aRF (abs(RIGHT_FRONT), MOTOR_GEARSET_18, false, MOTOR_ENCODER_COUNTS);
pros::Motor aRB (abs(RIGHT_BACK), MOTOR_GEARSET_18, false, MOTOR_ENCODER_COUNTS);

void moveToPoint(float targetX, float targetY){
  bool atPoint = false;
  float targetAngle = 0, currentPosition;
  float linearPower, turnPower = 0;
  float legY, legX;
  int sign;

  while (!atPoint) {
    updatePosition();

    //Pythagorean theorem –– distance between points
    legY = pow(targetY-getY(),2);
    legX = pow(targetX-getX(),2);
    currentPosition = sqrt(legY + legX);

    //Power based on linear distance to point
    linearPower = -aDistancePID.calculate(0, currentPosition);
    sign = linearPower/fabs(linearPower);

    if(fabs(linearPower) > 127){
      linearPower = 127*sign;
    }

    //Find angle between points based on legs of triangle
    targetAngle = atan2f((targetX-getX()), (targetY-getY()));
    //targetAngle = atan2f((targetY - getY()), (targetX - getX()));
    targetAngle *= 180/M_PI;

    //Power based on how far off angle the robot is to the point
    if(fabs(targetAngle - getAngleDegrees()) > 3){
      //std::cout << "target - current: " << targetAngle - getAngleDegrees() <<std::endl;
      turnPower = aAnglePID.calculate(targetAngle, getAngleDegrees());
      if(fabs(turnPower) > 13){
        turnPower = 13*sign;
      }
      //std::cout << "turn power: " << turnPower << " linear power: " << linearPower <<std::endl;
    }
    else {
    turnPower = 0;
    }

    //Set power
    set_drive(linearPower + turnPower, linearPower - turnPower);

    //IF AT TARGET, EXIT LOOP
    if(currentPosition < 1){
      atPoint = true;
    }
  pros::delay(20);
  }
  stop();
}

void followLine(float startX, float startY, float endX, float endY){}

void followPath(){}
