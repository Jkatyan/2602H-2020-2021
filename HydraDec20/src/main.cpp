#include "main.h"

pros::Controller master(pros::E_CONTROLLER_MASTER);

//Motors
pros::Motor LF (19, true);
pros::Motor LB (20, true);
pros::Motor RF (11);
pros::Motor RB (12);

pros::Motor left_mtr(14);
pros::Motor right_mtr(17);

pros::Motor rollerA(16);
pros::Motor rollerB(15, true);

//Sensors
pros::ADIAnalogIn frontLine ('F');
pros::ADIAnalogIn middleLine ('H');
pros::ADIAnalogIn topLine ('G');
pros::ADIAnalogIn maxLine ('D');

double wheelDia = 5.417;
double width = 11.52;

int intakeMode;
int conveyorMode;

bool bottom, middle, top, max;

void autoIndexing(){
	(frontLine.get_value()<=2700)?bottom = true:bottom = false;
	(middleLine.get_value()<=400)?middle = true:middle = false;
	(topLine.get_value()<=2870)?top = true:top = false;
	(maxLine.get_value()<=2870)?max = true:max = false;

	if(!top && !middle && bottom){
		rollerA.move(100);
		rollerB.move(100);
	}
	else if (middle && !top && !bottom){
		rollerA.move(0);
		rollerB.move(0);
	}
	else if (middle && bottom && !top){
		rollerA.move(100);
		rollerB.move(100);
	}
	else if (top){
		rollerA.move(0);
		rollerB.move(0);
	}
	else{
		rollerA.move(0);
		rollerB.move(0);
	}
}

void setStateIntakes(int state){
	intakeMode = state;
}

void setStateConveyor(int state){
	conveyorMode = state;
}

int conveyorTask(){
	while(true){
		switch(conveyorMode){
			case 0:
				rollerA.move(0);
				rollerB.move(0);
			break;
			case 1:
				rollerA.move(127);
				rollerB.move(127);
			break;
			case 2:
				rollerA.move(-127);
				rollerB.move(-127);
			break;
			case 3:
				autoIndexing();
			break;
		}
		pros::delay(20);
	}
}

int intakeTask(){
	while(true){
		switch(intakeMode){
			case 0:
				right_mtr.move_velocity(0);
				left_mtr.move_velocity(0);
			break;
			case 1:
				left_mtr.move(127);
				right_mtr.move(-127);
			break;
			case 2:
				left_mtr.move(-127);
				right_mtr.move(127);
			break;
		}
		pros::delay(20);
	}
}

using namespace okapi;
std::shared_ptr<ChassisController> drive =
	ChassisControllerBuilder()
	  .withMotors({19, 20}, {11, 12})
	  .withDimensions(AbstractMotor::gearset::green, {{5.42_in, 11.52_in}, imev5GreenTPR})
		.withOdometry()
	  .buildOdometry();

std::shared_ptr<AsyncMotionProfileController> profileController =
  AsyncMotionProfileControllerBuilder()
    .withLimits({
      0.9, // Maximum linear velocity of the Chassis in m/s
      2.0, // Maximum linear acceleration of the Chassis in m/s/s
      4.0 // Maximum linear jerk of the Chassis in m/s/s/s
    })
    .withOutput(drive)
    .buildMotionProfileController();

void setDrive(int l, int r){
	LF.move(l);
	LB.move(l);
	RF.move(r);
	RB.move(r);
}

void align(int delay, int power){
	setDrive(power, power);
	pros::delay(delay);
	setDrive(0, 0);
}

void alignRight(int delay, int power){
	setDrive(0, power);
	pros::delay(delay);
	setDrive(0, 0);
}

void alignLeft(int delay, int power){
	setDrive(power, 0);
	pros::delay(delay);
	setDrive(0, 0);
}

void reset(){
  LF.set_zero_position(LF.get_position());
  LB.set_zero_position(LB.get_position());
  RF.set_zero_position(RF.get_position());
  RB.set_zero_position(RB.get_position());
  LF.tare_position();
  LB.tare_position();
  RF.tare_position();
  RB.tare_position();
}

void whileVelocityNotZero(){
  while(LF.get_actual_velocity() <= 30){
    pros::delay(20);
  }
  while(LF.get_actual_velocity() > 25){
    pros::delay(20);
  }
}

void setChassisVel(int max){
  drive->setMaxVelocity(max);
}

void stopAutoDrive(){
  profileController->flipDisable(true);
}

void startAutoDrive(){
  profileController->flipDisable(false);
}

QLength toInch(double length) {
    okapi::QLength inchLength = length * okapi::inch;
    return inchLength;
}

QLength toFeet(double length) {
    okapi::QLength feetLength = length * okapi::foot;
    return feetLength;
}

QAngle toDegrees(double angle){
  okapi::QAngle degreeAngle = angle * okapi::degree;
  return degreeAngle;
}

QAngle radiansToDegrees(double radians) {
    okapi::QAngle radianAngle = radians * okapi::radian;
    okapi::QAngle degreeAngle = radianAngle.convert(degree) * okapi::degree;
    return degreeAngle;
}

// Generates and Runs a path
void pathGen(okapi::QLength forward,okapi::QLength side,okapi::QAngle angle, bool reverse){
  profileController->generatePath({
    {0_ft, 0_ft, 0_deg},
    {forward, side, angle}},
    "pathGen");
  profileController->setTarget("pathGen", reverse);
  profileController->waitUntilSettled();
}

// Generates a path
void gen(std::string name, okapi::QLength forward,okapi::QLength side,okapi::QAngle angle){
  profileController->generatePath({
    {0_ft, 0_ft, 0_deg},
    {forward, side, angle}},
    name);
}

// Runs a path
void run(std::string name, bool reverse){
  profileController->setTarget(name, reverse);
}

// Removes a path once the run is complete
void finishRun(std::string name){
  profileController->waitUntilSettled();
  profileController->removePath(name);
}

void turn(okapi::QAngle angle, int maxVel){
  drive->setMaxVelocity(maxVel);
  drive->turnAngle(angle);
  drive->setMaxVelocity(200);
}

void move(okapi::QLength distance){
  drive->moveDistance(distance);
}

void moveAsync(okapi::QLength distance){
  drive->moveDistanceAsync(distance);
}

void waitForSettle(){
  drive->waitUntilSettled();
}

void profilePosTurn(double angle, bool reset, int velLimit){

  int maxVel = velLimit;
  double target = 2 * (angle*width)/(wheelDia);
  double Rtarget = 0;
  double Ltarget = 0;

  if(reset){
    LF.move_relative(target, maxVel);
    LB.move_relative(target, maxVel);
    RF.move_relative(-target, maxVel);
    RB.move_relative(-target, maxVel);
  }
  else{
    double goalDiff = 2 * target;
    target = (goalDiff - (LF.get_position() - RF.get_position()) / 2);

    Ltarget = LF.get_position() + target;
    Rtarget = RF.get_position() - target;
    LF.move_absolute(Ltarget, maxVel);
    LB.move_absolute(Ltarget, maxVel);
    RF.move_absolute(Rtarget, maxVel);
    RB.move_absolute(Rtarget, maxVel);
  }

  int onTargetTime = 0;

  while(true){
    double error = LF.get_target_position() - LF.get_position();
    if(abs(error) < 25 && abs(LF.get_actual_velocity()) < 1 && abs(RF.get_actual_velocity()) < 1){
      break;
    }
    if(abs(LF.get_actual_velocity()) > 3 || abs(RF.get_actual_velocity()) > 3){
      onTargetTime = 0;
    }
    else{
      onTargetTime += 20;
    }
    if(onTargetTime > 150){
      break;
    }
    pros::delay(20);
  }
  LF.modify_profiled_velocity(200);
  LB.modify_profiled_velocity(200);
  RF.modify_profiled_velocity(200);
  RB.modify_profiled_velocity(200);
}

void initialize() {
	pros::lcd::initialize();
	pros::lcd::set_text(1, "Hello PROS User!");

	left_mtr.set_brake_mode(MOTOR_BRAKE_HOLD);
	right_mtr.set_brake_mode(MOTOR_BRAKE_HOLD);

	rollerA.set_brake_mode(MOTOR_BRAKE_HOLD);
	rollerB.set_brake_mode(MOTOR_BRAKE_HOLD);

	pros::Task intake_task (intakeTask);
	pros::Task conveyor_task (conveyorTask);

	gen("A", 39_in, 0_in, 0_deg);
	gen("B", 21_in, 0_in, 0_deg);
	gen("C", 3_ft, 0_in, 0_deg);
	gen("D", 8.5_ft, -1_ft, -45_deg);

}

void disabled() {}
void competition_initialize() {}

void autonomous() {

	setStateConveyor(2); //Deploy
	pros::delay(200);
	setStateConveyor(0);

	run("A", false); //Ball 1
	finishRun("A");

	profilePosTurn(-82, true, 150);

	setStateIntakes(1);
	setStateConveyor(3);
	run("B", false);
	finishRun("B");

	setStateIntakes(0); //Score
	setStateConveyor(1);
	pros::delay(300);
	setStateConveyor(3);

	run("C", true); //Back Out
	profilePosTurn(-130, true, 150);

	setStateIntakes(1);
	run("D", false);
	finishRun("D");

	setStateIntakes(0); //Score
	setStateConveyor(1);
	pros::delay(300);
	setStateConveyor(3);

	run("C", true); //Back Out
	setStateIntakes(0);
	setStateConveyor(0);
	
}

void opcontrol() {
	LF.set_brake_mode(MOTOR_BRAKE_COAST);
	LB.set_brake_mode(MOTOR_BRAKE_COAST);
	RF.set_brake_mode(MOTOR_BRAKE_COAST);
	RB.set_brake_mode(MOTOR_BRAKE_COAST);

	left_mtr.set_brake_mode(MOTOR_BRAKE_HOLD);
	right_mtr.set_brake_mode(MOTOR_BRAKE_HOLD);

	while (true) {
		pros::lcd::print(2, "A Torque: %f", rollerA.get_torque());
		pros::lcd::print(3, "B Torque: %f", rollerB.get_torque());
		if (master.get_digital(DIGITAL_R1)){
			setStateIntakes(1);
		}
		else if (master.get_digital(DIGITAL_R2)){
			setStateIntakes(2);
		}
		else {
			setStateIntakes(0);
		}

		if (master.get_digital(DIGITAL_L1)){
			setStateConveyor(1);
		}
		else if (master.get_digital(DIGITAL_L2)){
			setStateConveyor(2);
		}
		else {
			setStateConveyor(3);
		}

		if(master.get_digital(DIGITAL_R1) && master.get_digital(DIGITAL_R2)){
			RF.move(master.get_analog(ANALOG_RIGHT_Y) * 0.5);
			RB.move(master.get_analog(ANALOG_RIGHT_Y) * 0.5);
			LF.move(master.get_analog(ANALOG_LEFT_Y) * 0.5);
			LB.move(master.get_analog(ANALOG_LEFT_Y) * 0.5);
		}
		else{
			RF.move(master.get_analog(ANALOG_RIGHT_Y));
			RB.move(master.get_analog(ANALOG_RIGHT_Y));
			LF.move(master.get_analog(ANALOG_LEFT_Y));
			LB.move(master.get_analog(ANALOG_LEFT_Y));
		}

		if(master.get_digital(DIGITAL_B)){
			autonomous();
			pros::delay(60000);
		}

		pros::delay(20);
	}
}
