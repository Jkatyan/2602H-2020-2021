#pragma once

//JAVA TO C++ CONVERTER NOTE: Forward class declarations:
class Vector;

class Waypoint
{

private:
	Vector *pos;
	double targetVelocity = 0;

public:
	virtual ~Waypoint()
	{
		delete pos;
	}

	Waypoint(double x, double y);

	Waypoint(const Waypoint &waypoint);

	Waypoint(Vector *pos);

	virtual double getDistanceTo(Waypoint *other);

	virtual double getDistanceTo(double x, double y);

	virtual double getX();

	virtual double getY();

	virtual void setX(double x);

	virtual void setY(double y);

	virtual Vector *getVector();

	virtual double getTargetVelocity();

	virtual void setTargetVelocity(double targetVelocity);
};
