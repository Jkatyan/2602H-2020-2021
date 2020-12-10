#include "main.h"

void initialize() {
  initOdom();
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
  while (true){
    if (master.get_digital(DIGITAL_B)){
      runAuton(auton);
    }
    pros::delay(10);
  }
}
