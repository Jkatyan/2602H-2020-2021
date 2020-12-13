#include "main.h"

void initialize() {
	initIntakes();
	initDrive();
}

void disabled() {}

void competition_initialize() {
	initSelector();
}

void autonomous() {
	runAuton(auton);
}

void opcontrol() {
	while (true) {
		tankDrive();
		intakeOp();
		pros::delay(20);
	}
}
