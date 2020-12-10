#include "main.h"

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
  curLeft = leftEnc.get_value();
  curRight = rightEnc.get_value();
  curSide = sideEnc.get_value();

  deltaLeft = (curLeft - lastLeftPos)*(M_PI/180)*(SIDE_WHEEL_DIAM/2);
  deltaRight = (curRight - lastRightPos)*(M_PI/180)*(SIDE_WHEEL_DIAM/2);
  deltaSide = (curSide - lastSidePos)*(M_PI/180)*(BACK_WHEEL_DIAM/2);

  lastLeftPos = curLeft;
  lastRightPos = curRight;
  lastSidePos = curSide;

  deltaLr = (curLeft - leftAtReset)*(M_PI/180)*(SIDE_WHEEL_DIAM/2);
  deltaRr = (curRight - rightAtReset)*(M_PI/180)*(SIDE_WHEEL_DIAM/2);

  thetaNew = (thetaReset + (deltaLr - deltaRr)/(SL + SR));

  theta = atan2f(deltaY, deltaX);
  radius = sqrt(deltaX*deltaX + deltaY*deltaY);
  theta = theta-thetaM;
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

  printf("X: %f ", getX());
  printf("Y: %f ", getY());
  printf("getAngleDegrees(): %f ", getAngleDegrees());

  pros::delay(10);
}

float getX() {
  return x;
}

float getY() {
  return y;
}

void setPosition(float set_x, float set_y) {
  x = set_x;
  y = set_y;
}

void setAngle(float set_angle){
  angle = set_angle;
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
