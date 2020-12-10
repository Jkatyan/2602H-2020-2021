#pragma once

#include <cmath>

//JAVA TO C++ CONVERTER NOTE: Forward class declarations:
class Vector;

// All functions related to the robot following the path

// TODO Optimize the distance functions w/ distance squared

// Gets the index of the waypoint closest to the given coordinates and
// uses the last found point to optimize the search

int getClosestPoint(double x, double y, int lastPoint);

class LookAheadResult
{
public:
	double t = 0;
	int i = 0;
	Vector *lookAhead;

	virtual ~LookAheadResult()
	{
		delete lookAhead;
	}

	LookAheadResult(double t, int i, Vector *lookAhead);
};

// Calculate the next look ahead point and uses the last found point to ensure only forward progress is made

LookAheadResult *getLookAheadPoint(double x, double y, double lastT, int lastIndex);

double getLookAheadPointT(Vector *pos, Vector *start, Vector *end);

// angle is in radians
double getCurvatureToPoint(Vector *pos, double angle, Vector *lookAhead);

double pathLastT = 0.0;
int pathLastLookAheadIndex = 0;
int pathLastClosestIndex = 0;
double left = 0;
double right = 0;
long long lastCall = -1;

void followPath();

double constrainDouble(double value, double max, double min);
