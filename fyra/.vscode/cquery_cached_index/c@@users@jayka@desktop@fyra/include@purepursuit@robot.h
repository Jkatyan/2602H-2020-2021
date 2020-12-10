#pragma once

#include <cmath>

class Vector;

class Robot
{

public:
	Vector *pos;
	double velocity = 0;
	double angle = 0;
	double angularVelocity = 0;

	double leftSpeed = 0;
	double rightSpeed = 0;

	virtual ~Robot()
	{
		delete pos;
	}

	Robot();

	virtual void setLeft(double speed);

	virtual void setRight(double speed);

	virtual void update();

	virtual void show();

	virtual double getX();

	virtual double getY();

	virtual Vector *getPos();

	// returns angle in radians with north as 0
	virtual double getAngle();
};
