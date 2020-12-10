#include "main.h"

pros::ADIEncoder sideEnc (TRACKER_A, TRACKER_B);

pros::Motor OLF (abs(LEFT_FRONT), MOTOR_GEARSET_18, true, MOTOR_ENCODER_DEGREES);
pros::Motor OLB (abs(LEFT_BACK), MOTOR_GEARSET_18, true, MOTOR_ENCODER_DEGREES);
pros::Motor ORF (abs(RIGHT_FRONT), MOTOR_GEARSET_18, false, MOTOR_ENCODER_DEGREES);
pros::Motor ORB (abs(RIGHT_BACK), MOTOR_GEARSET_18, false, MOTOR_ENCODER_DEGREES);

float Sl = 4.75; //distance from tracking center to middle of left wheel
float Sr = 5.8; //distance from tracking center to middle of right wheel
float Ss = 4.2; //distance from tracking center to middle of the tracking wheel
float wheelDiameter = SIDE_WHEEL_DIAM; //diameter of the side wheels being used for tracking
float trackingDiameter = BACK_WHEEL_DIAM; //diameter of the sideways tracking wheel

float x = 0;
float y = 0;
float angle = 0;

float lastLeftPos = 0;
float lastRightPos = 0;
float lastSidePos = 0;

float deltaTheta = 0;
float thetaNew = 0;
float thetaM = 0;

float curLeft = 0;
float curRight = 0;
float curSide = 0;

float leftAtReset = 0;
float rightAtReset = 0;
float thetaReset = 0;

float deltaLeft = 0;
float deltaRight = 0;
float deltaSide = 0;

float deltaLr = 0;
float deltaRr = 0;

float deltaX = 0;
float deltaY = 0;

float theta = 0;
float radius = 0;

void updatePosition() {
  curLeft = (OLF.get_position() + OLB.get_position()) / 2;
  curRight = (ORF.get_position() + ORB.get_position()) / 2; //step 1
  curSide = sideEnc.get_value();

  deltaLeft = (curLeft - lastLeftPos)*(M_PI/180)*(wheelDiameter/2);
  deltaRight = (curRight - lastRightPos)*(M_PI/180)*(wheelDiameter/2); //step 2
  deltaSide = (curSide - lastSidePos)*(M_PI/180)*(trackingDiameter/2);

  lastLeftPos = curLeft;
  lastRightPos = curRight; //step 3
  lastSidePos = curSide;

  deltaLr = (curLeft - leftAtReset)*(M_PI/180)*(wheelDiameter/2); //step 4
  deltaRr = (curRight - rightAtReset)*(M_PI/180)*(wheelDiameter/2);

  thetaNew = (thetaReset + (deltaLr - deltaRr)/(Sl + Sr)); //step 5

  theta = atan2f(deltaY, deltaX);
  radius = sqrt(deltaX*deltaX + deltaY*deltaY);
  theta = theta-thetaM;                          //step 10
  deltaX = radius*cos(theta);
  deltaY = radius*sin(theta);

  thetaNew+=M_PI;
  while (thetaNew <= 0) {
    thetaNew+=2*M_PI;
  }
  thetaNew = modulo(thetaNew, 2*M_PI);
  thetaNew-=M_PI;

  angle = thetaNew;
  x = x - deltaX; //step 11
  y = y + deltaY;

  pros::delay(20);

  printf("X: %f ", getX());
  printf("Y: %f ", getY());
  printf("getAngleDegrees(): %f ", getAngleDegrees());
}

float getX() {
  return x;
}

float getY() {
  return y;
}

float getAngleDegrees() {
  return angle*180/M_PI;
}
float getAngle() {
  return angle;
}

float modulo(float a, float b) {
  while (a>b) {
    a-=b;
  }
  return a;
}

void initOdom(){
  pros::Task odom_task(updatePosition);
}
