#include "main.h"
using namespace okapi;

//Declaration
const int FRONT_LEFT = 1;
const int BACK_LEFT = 2;
const int FRONT_RIGHT = -3;
const int BACK_RIGHT = -4;

const int LIFT_PORT = 16;
const int INTAKE_RIGHT = 21;
const int INTAKE_LEFT = -8;
const int TILTER_PORT = -19;

Controller master;
ADIButton lift_button_one('H');
ADIButton lift_button_two('G');


Motor tilter(TILTER_PORT);
Motor lift(LIFT_PORT);

MotorGroup intake({INTAKE_RIGHT, INTAKE_LEFT});

ControllerButton intakeIn(ControllerDigital::L1);
ControllerButton intakeOut(ControllerDigital::L2);
ControllerButton liftUp(ControllerDigital::R1);
ControllerButton liftDown(ControllerDigital::R2);


auto drive = ChassisControllerFactory::create(
	{FRONT_LEFT,BACK_LEFT},
	{FRONT_RIGHT,BACK_RIGHT},
	AbstractMotor::gearset::green,
	{4_in, 12_in}
	//CHANGE ROBOT WIDTH AFTER MEASURING
);

auto profileControllerFast = AsyncControllerFactory::motionProfile(
  0.5,  // Maximum linear velocity of the Chassis in m/s
  1.5,  // Maximum linear acceleration of the Chassis in m/s/s
  8.0, // Maximum linear jerk of the Chassis in m/s/s/s
  drive // Chassis Controller
);

auto profileControllerSlow = AsyncControllerFactory::motionProfile(
  1.0,  // Maximum linear velocity of the Chassis in m/s
  2.0,  // Maximum linear acceleration of the Chassis in m/s/s
  8.0, // Maximum linear jerk of the Chassis in m/s/s/s
  drive // Chassis Controller
);

bool lift_down_disabled=false;
bool intaking = false;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// OP CONTROL CODE
void drive_op(){
	drive.arcade(master.getAnalog(ControllerAnalog::leftY),
						 -master.getAnalog(ControllerAnalog::leftX));
}

void tilt_op(){
	tilter.moveVoltage(master.getAnalog(ControllerAnalog::rightY)*12000);
}

void lift_op(){
	if(lift_button_one.isPressed() && lift_button_two.isPressed()){
		lift_down_disabled=true;
	}

	if(!lift_down_disabled){
		if(liftDown.isPressed()){
			lift.moveVoltage(-8000);
		}
	}
	if(!lift_down_disabled && liftDown.isPressed()){
		lift.moveVoltage(-8000);
	} else if(liftUp.isPressed()){
		lift.moveVoltage(12000);
	} else {
		lift.moveVoltage(0);
	}
}

void intake_op(){
	if(intakeIn.changedToPressed()){
		intaking = !intaking;
	}

	if(intaking){
		intake.moveVoltage(12000);
	} else {
		if(intakeOut.isPressed()){
			intake.moveVoltage(-8000);
		} else {
			intake.moveVoltage(0);
		}
	}
}

void initialize() {
	pros::lcd::initialize();
	pros::lcd::set_text(1, "Avery 1");
}

void disabled() {}

void competition_initialize() {
	profileControllerSlow.generatePath({Point{0_ft, 0_ft, 0_deg}, Point{2.75_ft, 0_ft, 0_deg}}, "A");
	profileControllerFast.generatePath({Point{0_ft, 0_ft, 0_deg}, Point{1.5_ft, 0_ft, 0_deg}}, "B");
	profileControllerSlow.generatePath({Point{0_ft, 0_ft, 0_deg}, Point{1.0_ft, 0_ft, 0_deg}}, "C");
	profileControllerSlow.generatePath({Point{0_ft, 0_ft, 0_deg}, Point{0.15_ft, 0_ft, 0_deg}}, "D");
	profileControllerFast.generatePath({Point{0_ft,0_ft,0_deg}, Point{1.0_ft,0_ft,0_deg}},"E" );
}

void autonomous() {
	profileControllerSlow.setTarget("A");
	intake.moveVoltage(12000);
  profileControllerSlow.waitUntilSettled();
	profileControllerFast.setTarget("B", true);
	intake.moveVoltage(2000);
  profileControllerFast.waitUntilSettled();
	intake.moveVoltage(0);
	drive.setMaxVelocity(40);
	drive.turnAngle(245);
	drive.setMaxVelocity(200);
	profileControllerSlow.setTarget("C");
  profileControllerSlow.waitUntilSettled();
	intake.moveVoltage(-4000);
	pros::delay(500);
	intake.moveVoltage(0);
	tilter.moveVoltage(12000);
	pros::delay(1750);
	tilter.moveVoltage(0);
	profileControllerSlow.setTarget("D");
	profileControllerSlow.waitUntilSettled();
	profileControllerFast.setTarget("E", true);
	profileControllerFast.waitUntilSettled();
}

void opcontrol() {
	lift.setBrakeMode(AbstractMotor::brakeMode::hold);
	intake.setBrakeMode(AbstractMotor::brakeMode::brake);
	while (true){
		drive_op();
		tilt_op();
		lift_op();
		intake_op();

		pros::delay(10);
	}
}
