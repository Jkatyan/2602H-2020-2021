#include "PurePursuit/PathGenerator.h"
#include "PurePursuit/Waypoint.h"
#include "PurePursuit/Robot.h"

void <missing_class_definition>::setup()
{
	size(1200, 600);
	frameRate(60);

	userPoints = std::vector<Waypoint*>();
	injectedPoints = std::vector<Waypoint*>();
	smoothedPoints = std::vector<Waypoint*>();

	distancePath = std::vector<double>();
	distanceBetween = std::vector<double>();
	curvatures = std::vector<double>();

	userInjectedHidden = false;

	robot = new Robot();
	following = false;
	Waypoint tempVar(robot->getX(), robot->getY());
	userPoints->add(&tempVar);

	// println(getCurvatureToPoint());
}

void <missing_class_definition>::draw()
{
	background(255);

	robot->update();
	robot->show();

	if (!userInjectedHidden)
	{
		for (auto waypoint : *injectedPoints)
		{
			drawWaypoint(waypoint, color(255, 0, 0));
		}

		for (auto waypoint : *userPoints)
		{
			drawWaypoint(waypoint, color(0, 0, 255));
		}
	}

	for (int i = 0; i < smoothedPoints->size(); i++)
	{
		Waypoint *waypoint = smoothedPoints->get(i);
		double curvature = curvatures->get(i);
		drawWaypoint(waypoint, color(0, 255 - (static_cast<int>(128 * curvature * 50)), 0));
	}

	if (following)
	{
		followPath();
	}
}

void <missing_class_definition>::mousePressed()
{
	Waypoint tempVar(mouseX / SCALE_FACTOR, height - (mouseY / SCALE_FACTOR));
	userPoints->add(&tempVar);
}

void <missing_class_definition>::keyPressed()
{
	if (key == L'i')
	{
		int start = millis();
		injectWaypoints();
		std::wcout << L"Injection time: " << (millis() - start) << std::endl;
	}
	if (key == L's')
	{
		int start = millis();
		smoothWaypoints();
		std::wcout << L"Smoothing time: " << (millis() - start) << std::endl;

		start = millis();
		calculateDistances();
		std::wcout << L"Distance time: " << (millis() - start) << std::endl;

		start = millis();
		calculateCurvatures();
		std::wcout << L"Curvature time: " << (millis() - start) << std::endl;

		start = millis();
		calculateTargetVelocities();
		std::wcout << L"Target velocity time: " << (millis() - start) << std::endl;
	}
	if (key == L'r')
	{
		userPoints->clear();
		injectedPoints->clear();
		smoothedPoints->clear();
	}
	if (key == L'h')
	{
		userInjectedHidden = !userInjectedHidden;
	}
	if (key == L'p')
	{
		std::wcout << L"User Points: " << std::endl;
		for (auto waypoint : *userPoints)
		{
			waypoint->getVector()->printInfo();
		}

		std::wcout << L"Injected Points: " << std::endl;
		for (auto waypoint : *injectedPoints)
		{
			waypoint->getVector()->printInfo();
		}

		std::wcout << L"Smoothed Points: " << std::endl;
		for (auto waypoint : *smoothedPoints)
		{
			waypoint->getVector()->printInfo();
		}

		std::wcout << L"Distances: " << std::endl;
		for (auto distance : *distancePath)
		{
			std::wcout << distance << std::endl;
		}

		std::wcout << L"Curvatures: " << std::endl;
		for (auto curvature : *curvatures)
		{
			std::wcout << curvature << std::endl;
		}

		std::wcout << L"Target Velocities: " << std::endl;
		for (auto waypoint : *smoothedPoints)
		{
			std::wcout << waypoint->getTargetVelocity() << std::endl;
		}
	}
	if (key == L'f')
	{
		following = true;
	}
}

void <missing_class_definition>::keyReleased()
{
	if (key == L'f')
	{
		following = false;
	}
}

void <missing_class_definition>::drawWaypoint(Waypoint *waypoint, color *waypointColor)
{
	stroke(waypointColor);
	fill(waypointColor);

	ellipse(static_cast<float>(waypoint->getX()) * SCALE_FACTOR, static_cast<float>(height - (waypoint->getY() * SCALE_FACTOR)), 10.0, 10.0);
}
