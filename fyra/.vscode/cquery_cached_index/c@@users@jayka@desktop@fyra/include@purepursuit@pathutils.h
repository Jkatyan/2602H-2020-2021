#pragma once
#include "main.h"
#include <cmath>

// All functions related to generating the path

void injectWaypoints();

void smoothWaypoints();

void calculateDistances();

void calculateCurvatures();

// Calculates the target velocity for each point using curvature and decceleration
// Must call calculateDistances() before calling this
void calculateTargetVelocities();
