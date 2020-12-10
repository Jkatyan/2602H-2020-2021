#include "PurePursuit/Vector.h"

Vector::Vector(double x, double y)
{
	this->x = x;
	this->y = y;

	this->mag = -1;
	this->magSquared = -1;
}

Vector::Vector(const Vector &copy)
{
	this->x = copy->getX();
	this->y = copy->getY();

	this->mag = -1;
	this->magSquared = -1;
}

double Vector::getX()
{
	return this->x;
}

double Vector::getY()
{
	return this->y;
}

void Vector::setX(double x)
{
	this->x = x;
	this->magSquared = -1;
	this->mag = -1;
}

void Vector::setY(double y)
{
	this->y = y;
	this->magSquared = -1;
	this->mag = -1;
}

double Vector::getMagSquared()
{
	if (this->magSquared == -1)
	{
		this->magSquared = this->x * this->x + this->y * this->y;
	}
	return this->magSquared;
}

double Vector::getMag()
{
	if (this->mag == -1)
	{
		this->mag = std::sqrt(getMagSquared());
	}
	return this->mag;
}

Vector *Vector::normalize()
{
	double newX = this->x /= getMag();
	double newY = this->y /= getMag();

	return new Vector(newX, newY);
}

Vector *Vector::add(Vector *vector)
{
	return new Vector(this->getX() + vector->getX(), this->getY() + vector->getY());
}

Vector *Vector::sub(Vector *vector)
{
	return new Vector(this->getX() - vector->getX(), this->getY() - vector->getY());
}

Vector *Vector::mult(double num)
{
	return new Vector(this->getX() * num, this->getY() * num);
}

double Vector::dot(Vector *vector)
{
	return this->getX() * vector->getX() + this->getY() * vector->getY();
}

double Vector::getDistanceTo(Vector *other)
{
	return std::sqrt((this->getX() - other->getX()) * (this->getX() - other->getX()) + (this->getY() - other->getY()) * (this->getY() - other->getY()));
}

void Vector::printInfo()
{
	std::wcout << L"X: " << getX() << L", Y: " << getY() << std::endl;
}

void Vector::printExtInfo()
{
	std::wcout << L"X: " << getX() << L", Y: " << getY() << L", Mag: " << getMag() << std::endl;
}
