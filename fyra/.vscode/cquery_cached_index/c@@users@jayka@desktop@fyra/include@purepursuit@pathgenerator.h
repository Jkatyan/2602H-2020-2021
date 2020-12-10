#pragma once

#include <vector>
#include <iostream>

class Waypoint;
class Robot;

std::vector<Waypoint*> *userPoints;
std::vector<Waypoint*> *injectedPoints;
std::vector<Waypoint*> *smoothedPoints;

std::vector<double> *distancePath;
std::vector<double> *distanceBetween; // distance between point i and point i - 1
std::vector<double> *curvatures;

bool userInjectedHidden = false;

Robot *robot;
bool following = false;
