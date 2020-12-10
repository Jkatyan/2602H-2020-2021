#pragma once

// Injection
double SPACING = 25.0;

// Smoothing
double WEIGHT_SMOOTH = 0.75; // 0.75 - 0.98
double WEIGHT_DATA = 1 - WEIGHT_SMOOTH; // 0.02 - 0.25
double TOLERANCE = 0.001;

// Robot Constants
double TURNING_CONSTANT = 2.0;
double MAX_VELOCITY = 200.0; // inches per second (I think)
double MAX_ACCELERATION = 250.0; // inches per second squared (I think)

double DRIVE_WIDTH = 30.0;

// Following constants
double LOOKAHEAD = 75; // 12 - 25 inches
