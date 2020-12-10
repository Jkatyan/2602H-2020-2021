#pragma once

#include <iostream>
#include <cmath>

class Vector
{

private:
	double x = 0;
	double y = 0;

	double mag = 0;
	double magSquared = 0;

	/**
	 * 2D Vector class for handling calculations
	 * All functions will not modify the current object
	 */

public:
	Vector(double x, double y);

	Vector(const Vector &copy);

	virtual double getX();

	virtual double getY();

	virtual void setX(double x);

	virtual void setY(double y);

	virtual double getMagSquared();

	virtual double getMag();

	virtual Vector *normalize();

	virtual Vector *add(Vector *vector);

	virtual Vector *sub(Vector *vector);

	virtual Vector *mult(double num);

	virtual double dot(Vector *vector);

	virtual double getDistanceTo(Vector *other);

	virtual void printInfo();

	virtual void printExtInfo();
};
