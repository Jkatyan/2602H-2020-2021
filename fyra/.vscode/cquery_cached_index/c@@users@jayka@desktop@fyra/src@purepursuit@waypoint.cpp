#include "PurePursuit/Waypoint.h"
#include "PurePursuit/Vector.h"

Waypoint::Waypoint(double x, double y) : Vector tempVar(x, y);
Waypoint(&tempVar)
{
}

Waypoint::Waypoint(const Waypoint &waypoint) : Waypoint(waypoint->getVector())
{
}

Waypoint::Waypoint(Vector *pos)
{
	this->pos = new Vector(pos);
	targetVelocity = -1.0;
}

double Waypoint::getDistanceTo(Waypoint *other)
{
	return this->pos->getDistanceTo(other->getVector());
}

double Waypoint::getDistanceTo(double x, double y)
{
	Vector tempVar(x, y);
	return this->pos->getDistanceTo(&tempVar);
}

double Waypoint::getX()
{
	return this->pos->getX();
}

double Waypoint::getY()
{
	return this->pos->getY();
}

void Waypoint::setX(double x)
{
	this->pos->setX(x);
}

void Waypoint::setY(double y)
{
	this->pos->setY(y);
}

Vector *Waypoint::getVector()
{
	return pos;
}

double Waypoint::getTargetVelocity()
{
	return this->targetVelocity;
}

void Waypoint::setTargetVelocity(double targetVelocity)
{
	this->targetVelocity = targetVelocity;
}
