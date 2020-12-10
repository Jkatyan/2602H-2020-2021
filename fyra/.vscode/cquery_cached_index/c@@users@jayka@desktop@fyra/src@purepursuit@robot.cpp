#include "PurePursuit/Robot.h"
#include "PurePursuit/Vector.h"

Robot::Robot()
{
	pos = new Vector(width / 2 / SCALE_FACTOR, height / 2 / SCALE_FACTOR);
	velocity = 0.0;
	angle = PI / 2;
	angularVelocity = 0.0;

	leftSpeed = 0.0;
	rightSpeed = 0.0;
}

void Robot::setLeft(double speed)
{
	leftSpeed = speed;
}

void Robot::setRight(double speed)
{
	rightSpeed = speed;
}

void Robot::update()
{
	this->velocity = (this->leftSpeed + this->rightSpeed) / 2.0;
	this->angularVelocity = -(this->leftSpeed - this->rightSpeed) / DRIVE_WIDTH;

	this->angle += this->angularVelocity / frameRate;
	Vector tempVar(this->velocity * std::cos(this->angle), this->velocity * std::sin(this->angle));
	this->pos = this->pos->add((&tempVar)->mult(1 / static_cast<float>(frameRate)));

	this->leftSpeed = 0;
	this->rightSpeed = 0;
}

void Robot::show()
{
	rectMode(CENTER);
	fill(120);
	stroke(0);
	pushMatrix();
		translate(static_cast<float>(this->pos->getX()) * SCALE_FACTOR, static_cast<float>(height - (this->pos->getY() * SCALE_FACTOR)));
		rotate((float) - angle + PI / 2);
		rect(0, 0, static_cast<float>(DRIVE_WIDTH) * SCALE_FACTOR, static_cast<float>(DRIVE_WIDTH) * 2.0 * SCALE_FACTOR);
	popMatrix();
}

double Robot::getX()
{
	return pos->getX();
}

double Robot::getY()
{
	return pos->getY();
}

Vector *Robot::getPos()
{
	return pos;
}

double Robot::getAngle()
{
	return angle;
}
