#include "PurePursuit/RobotUtils.h"
#include "PurePursuit/Waypoint.h"
#include "PurePursuit/Vector.h"

int <missing_class_definition>::getClosestPoint(double x, double y, int lastPoint)
{
	int index = -1;
	double closestDist = -1;

	for (int i = lastPoint; i < smoothedPoints->size(); i++)
	{
		Waypoint *waypoint = smoothedPoints->get(i);
		double checkDist = waypoint->getDistanceTo(x, y);

		if (index == -1)
		{
			index = i;
			closestDist = checkDist;
		}
		else
		{
			if (checkDist <= closestDist)
			{
				index = i;
				closestDist = checkDist;
			}
		}
	}

	return index;
}

LookAheadResult::LookAheadResult(double t, int i, Vector *lookAhead)
{
	this->t = t;
	this->i = i;
	this->lookAhead = lookAhead;
}

LookAheadResult *<missing_class_definition>::getLookAheadPoint(double x, double y, double lastT, int lastIndex)
{
	Vector *pos = new Vector(x, y);
	for (int i = lastIndex; i < smoothedPoints->size() - 1; i++)
	{
		Waypoint *a = smoothedPoints->get(i);
		Waypoint *b = smoothedPoints->get(i + 1);

		double t = getLookAheadPointT(pos, a->getVector(), b->getVector());
		if (t != -1)
		{
			// If the segment is further along or the fractional index is greater, then this is the correct point
			if (i > lastIndex || t > lastT)
			{
				Vector *d = b->getVector()->sub(a->getVector());

//JAVA TO C++ CONVERTER TODO TASK: A 'delete pos' statement was not added since pos was passed to a method or constructor. Handle memory management manually.
				return new LookAheadResult(t, i, a->getVector()->add(d->mult(t)));
			}
		}
	}

	// Just return last look ahead result
	Waypoint *a = smoothedPoints->get(lastIndex);
	Waypoint *b = smoothedPoints->get(lastIndex + 1);
	Vector *d = b->getVector()->sub(a->getVector());

//JAVA TO C++ CONVERTER TODO TASK: A 'delete pos' statement was not added since pos was passed to a method or constructor. Handle memory management manually.
	return new LookAheadResult(lastT, lastIndex, a->getVector()->add(d->mult(lastT)));
}

double <missing_class_definition>::getLookAheadPointT(Vector *pos, Vector *start, Vector *end)
{
	Vector *d = end->sub(start);
	Vector *f = start->sub(pos);

	double a = d->dot(d);
	double b = 2.0 * f->dot(d);
	double c = f->dot(f) - LOOKAHEAD * LOOKAHEAD;

	double discriminant = b * b - 4 * a * c;

	if (discriminant < 0)
	{
		return -1;
	}
	else
	{
		discriminant = std::sqrt(discriminant);
		double t1 = (-b - discriminant) / (2 * a);
		double t2 = (-b + discriminant) / (2 * a);

		if (t1 >= 0 && t1 <= 1)
		{
			return t1;
		}
		if (t2 >= 0 && t2 <= 1)
		{
			return t2;
		}
	}

	return -1;
}

double <missing_class_definition>::getCurvatureToPoint(Vector *pos, double angle, Vector *lookAhead)
{
	double a = -std::tan(angle);
	double b = 1.0;
	double c = std::tan(angle) * pos->getX() - pos->getY();
	drawLine(a, b, c);

	double x = std::abs(a * lookAhead->getX() + b * lookAhead->getY() + c) / std::sqrt(a * a + b * b);
	double l = pos->getDistanceTo(lookAhead);
	double curvature = 2 * x / l / l;

	Vector tempVar(std::cos(angle), std::sin(angle));
	Vector *otherPoint = pos->add(&tempVar);
	double side = Math::signum((otherPoint->getY() - pos->getY()) * (lookAhead->getX() - pos->getX()) - (otherPoint->getX() - pos->getX()) * (lookAhead->getY() - robot->getY()));

	return curvature * side;
}

void <missing_class_definition>::drawLine(double a, double b, double c)
{
	for (int x = 0; x <= width; x += 10)
	{
		double y = height - (-c - a * x) / b;
		fill(255, 0, 0);
		noStroke();
		ellipse(static_cast<float>(x), static_cast<float>(y), 2, 2);
	}
}

void <missing_class_definition>::followPath()
{
	if (smoothedPoints->size() != 0)
	{
		Vector *robotPos = robot->getPos();
		LookAheadResult *lookAheadResult = getLookAheadPoint(robotPos->getX(), robotPos->getY(), pathLastT, pathLastLookAheadIndex);
		pathLastT = lookAheadResult->t;
		pathLastLookAheadIndex = lookAheadResult->i;
		Vector *lookAheadPoint = lookAheadResult->lookAhead;

		double curvature = getCurvatureToPoint(robotPos, robot->getAngle(), lookAheadPoint);
		pathLastClosestIndex = getClosestPoint(robotPos->getX(), robotPos->getY(), pathLastClosestIndex);
		double targetVelocity = smoothedPoints->get(pathLastClosestIndex)->getTargetVelocity();

		double tempLeft = targetVelocity * (2.0 + curvature * DRIVE_WIDTH) / 2.0;
		double tempRight = targetVelocity * (2.0 - curvature * DRIVE_WIDTH) / 2.0;

		// println(curvature);

		if (lastCall == -1)
		{
			lastCall = millis();
		}
		double maxChange = (millis() - lastCall) / 1000.0 * MAX_ACCELERATION;
		left += constrainDouble(tempLeft - left, maxChange, -maxChange);
		right += constrainDouble(tempRight - right, maxChange, -maxChange);

		robot->setLeft(left);
		robot->setRight(right);

		lastCall = millis();

		fill(0);
		noStroke();
		ellipse(static_cast<float>(lookAheadPoint->x), static_cast<float>(height - lookAheadPoint->y), 25, 25);
		noFill();
		stroke(0);
		ellipse(static_cast<float>(robotPos->getX()), static_cast<float>(height - robotPos->getY()), static_cast<float>(LOOKAHEAD) * 2, static_cast<float>(LOOKAHEAD) * 2);
	}
}

double <missing_class_definition>::constrainDouble(double value, double max, double min)
{
	if (value < min)
	{
		return min;
	}
	if (value > max)
	{
		return max;
	}
	return value;
}
