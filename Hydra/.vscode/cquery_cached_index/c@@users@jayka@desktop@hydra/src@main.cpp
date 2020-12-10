#include "main.h"

pros::Controller user (CONTROLLER_MASTER);

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

    if (user.get_digital(DIGITAL_B)){
      runAuton(auton);
    }

		pros::delay(20);
	}
}
