#ifndef _ODOM_HPP_
#define _ODOM_HPP_

void updatePosition();

float getX();

float getY();

void setPosition(float set_x, float set_y);

void setAngle(float set_angle);

float getAngleDegrees();

float getAngle();

float modulo(float a, float b);

void initOdom();

#endif
