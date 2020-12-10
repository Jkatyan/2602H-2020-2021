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
	printf("Selected Auton: %d\n\n", auton);
	runAuton(auton);
}

void opcontrol() {
	setDriveBrakes(MOTOR_BRAKE_COAST);
	while (true) {
    tankDrive();
    intakeOp();
		pros::delay(10);
	}
}
