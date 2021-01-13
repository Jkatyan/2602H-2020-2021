#include "main.h"

//Autonomous:
/*
* 0: Skills

* 1: Red Left
* 2: Red Left 2
* 3: Red Left 3

* 4: Blue Right
* 5: Blue Right 2
* 6: Blue Right 3

*/
const int auton = 0;

//Motor Ports
#define LEFT_FRONT 19
#define LEFT_BACK 20
#define RIGHT_FRONT 11
#define RIGHT_BACK 12
#define LEFT_INTAKE 14
#define RIGHT_INTAKE 17
#define MID_ROLLER 15
#define TOP_ROLLER 16

//Sensors
#define TRACKER_A 1
#define TRACKER_B 2
#define IMU_PORT 1
#define OPTICAL_PORT 18
#define VISION_PORT 21

//CONSTANTS
#define DRIVEP 0.185
#define DRIVED 0.04
#define DRIVEF 9

#define TURNP 0.917
#define TURND 0

#define ANGLEP 3

#define RC 1 //Right Chassis Speed
#define LC 1 //Left Chassis Speed

//INITIALIZE
PID drivePID;
PID turnPID;
PID anglePID;

//CREATE DEVICES & SENSORS
pros::Controller master (CONTROLLER_MASTER);

pros::Motor LF (LEFT_FRONT, true);
pros::Motor LB (LEFT_BACK, true);
pros::Motor RF (RIGHT_FRONT);
pros::Motor RB (RIGHT_BACK);
pros::Motor leftIntake (LEFT_INTAKE);
pros::Motor rightIntake (RIGHT_INTAKE, true);
pros::Motor midRoller (MID_ROLLER, true);
pros::Motor topRoller (TOP_ROLLER, true);

pros::ADIEncoder enc (TRACKER_A, TRACKER_B);
pros::Optical optical(OPTICAL_PORT);
pros::Imu imu (IMU_PORT);
pros::Vision vision (VISION_PORT);

pros::vision_object_s_t obj;
pros::vision_signature_s_t RED_SIG = {1, {1, 0, 0}, 3.000000, 6335, 7645, 6990, -541, 431, -56, 4598058, 0};
pros::vision_signature_s_t BLUE_SIG;
pros::vision_signature_s_t FLAG_SIG;


//HELPER FUNCTIONS
void brake(){
	LF.move_velocity(0);
	LB.move_velocity(0);
	RF.move_velocity(0);
	RB.move_velocity(0);
}

void set_drive(float left, float right){
	if (left == 0 && right == 0){
		brake();
	}
	LF.move(left * LC);
	LB.move(left * LC);
	RF.move(right * RC);
	RB.move(right * RC);
}

void set_drive_brakes(pros::motor_brake_mode_e_t mode){
	LF.set_brake_mode(mode);
	LB.set_brake_mode(mode);
	RF.set_brake_mode(mode);
	RB.set_brake_mode(mode);
}

void set_intake_brakes(pros::motor_brake_mode_e_t mode){
	leftIntake.set_brake_mode(mode);
	rightIntake.set_brake_mode(mode);
	midRoller.set_brake_mode(mode);
	topRoller.set_brake_mode(mode);
}

void reset_drive(){
	RF.tare_position();
	RB.tare_position();
	LF.tare_position();
	LB.tare_position();
	enc.reset();
}

double rollAngle180(double angle){
	return angle - 360.0 * std::floor((angle + 180) / 360.0);
}

double getRotation(){
	return imu.get_rotation() * 1.00166;
}

bool isStopped(){
	if (!RF.is_stopped() || !RB.is_stopped() || !LF.is_stopped() || !LB.is_stopped()){
		return false;
	}
	return true;
}

bool crossedHalf(float distance, float target){
	if(distance < target){
		return (target/2 > distance) ? false : true;
	}
	else{
		return (target/2 < distance) ? false : true;
	}
	return false;
}

//ROLLER FUNCTIONS
int intakeMode = 0;
void setState(int state){
	intakeMode = state;
	/*
	* 0: Everything Stopped
	* 1: Indexing, Intakes In
	* 2: Indexing, Intakes Out
	* 3: Indexing, Intakes Stopped
	* 4: Reverse Everything
	* 5: Score, Intakes In
	* 6: Score, Intakes Out
	* 7: Score, Intakes Stopped
	*/
}

void set_intakes(int intakes){
	rightIntake.move(intakes);
	leftIntake.move(intakes);
}

void set_conveyor(int mid, int top){
	midRoller.move(mid);
	topRoller.move(top);
}

void set_rollers(int intakes, int mid, int top){
	rightIntake.move(intakes);
	leftIntake.move(intakes);
	midRoller.move(mid);
	topRoller.move(top);
}

int intakeTask(){
	while(true){
		switch(intakeMode){
			case 0:
				set_rollers(0, 0, 0);
			break;
			case 1:
				set_rollers(127, 127, -60);
			break;
			case 2:
				set_rollers(-127, 127, -60);
			break;
			case 3:
				set_rollers(0, 127, -60);
			break;
			case 4:
				set_rollers(-127, -127, -127);
			break;
			case 5:
				set_rollers(127, 127, 127);
			break;
			case 6:
				set_rollers(-127, 127, 127);
			break;
			case 7:
				set_rollers(0, 127, 127);
			break;
		}
		pros::delay(20);
	}
}

void rollerOp(){
	intakeMode = -1;
	if(master.get_digital(DIGITAL_L1)){
		if(master.get_digital(DIGITAL_L2)){ //Scoring
			set_conveyor(127, 127);
		}
		else{ //Indexing
			set_conveyor(127, -60);
		}
	}
	else if(master.get_digital(DIGITAL_L2)){ //Reverse
		set_conveyor(-127, -127);
	}
	else{ //Stopped
		set_conveyor(0, 0);
	}
	if(master.get_digital(DIGITAL_R1)){
		set_intakes(127);
	}
	else if(master.get_digital(DIGITAL_R2)){
		set_intakes(-127);
	}
	else{
		set_intakes(0);
	}
}

//VISION FUNCTIONS
bool detectRedBall() {

}

bool detectBlueBall() {

}

bool detectFlag() {

}

//DRIVE FUNCTIONS
void driveOp(){
	set_drive(master.get_analog(ANALOG_LEFT_Y), master.get_analog(ANALOG_RIGHT_Y));
}

void drive(int target, float angle, int time, float correctionStrength, float speed){
  int atTarget = 0;
  float driveEnc = -enc.get_value();
	int direction = 1;
  int startTime = pros::millis();

	if (driveEnc > target){
		direction = -1;
	}

  while ((atTarget != 1) || (pros::millis()-startTime) < time) {
	  driveEnc = -enc.get_value();
	  float driveVal = pidCalculate(drivePID, target, driveEnc)*speed;
		float angleVal = pidCalculate(anglePID, angle, getRotation())*correctionStrength;

	  float rightVal = driveVal - angleVal;
	  float leftVal = driveVal + angleVal;

		if (direction == 1){
			set_drive(leftVal + DRIVEF, rightVal + DRIVEF);
			if(driveEnc >= target || ((pros::millis()-startTime) == time)){
		     atTarget = 1;
			}
		}
		else{
			set_drive(leftVal - DRIVEF, rightVal - DRIVEF);
			if(driveEnc <= target || ((pros::millis()-startTime) == time)){
		     atTarget = 1;
			}
		}
    pros::delay(20);
  }
}

void driveSetState(int target, int state, float angle, int time, float correctionStrength, float speed){
  int atTarget = 0;
  float driveEnc = -enc.get_value();
	int direction = 1;
  int startTime = pros::millis();

	if (driveEnc > target){
		direction = -1;
	}

  while ((atTarget != 1) || (pros::millis()-startTime) < time) {
	  driveEnc = -enc.get_value();
	  float driveVal = pidCalculate(drivePID, target, driveEnc)*speed;
		float angleVal = pidCalculate(anglePID, angle, getRotation())*correctionStrength;

	  float rightVal = driveVal - angleVal;
	  float leftVal = driveVal + angleVal;

		if(crossedHalf((target - driveEnc), target)){
			setState(state);
		}

		if (direction == 1){
			set_drive(leftVal + DRIVEF, rightVal + DRIVEF);
			if(driveEnc >= target || ((pros::millis()-startTime) == time)){
		     atTarget = 1;
			}
		}
		else{
			set_drive(leftVal - DRIVEF, rightVal - DRIVEF);
			if(driveEnc <= target || ((pros::millis()-startTime) == time)){
		     atTarget = 1;
			}
		}
    pros::delay(20);
  }
}

void track(int target, int sig, int time, float speed){
  int atTarget = 0;
  float driveEnc = -enc.get_value();
	int direction = 1;
  int startTime = pros::millis();

	if (driveEnc > target){
		direction = -1;
	}

  while ((atTarget != 1) || (pros::millis()-startTime) < time) {
		int angleVal = 0;
		if(vision.get_object_count() > 0){
			obj = vision.get_by_size(0); // Get largest object visible
			if(obj.signature == sig){
				if(abs(obj.x_middle_coord) <= 18){
					angleVal = 0;
				}
				else if(abs(obj.x_middle_coord) <= 25){
					angleVal = 20*speed;
				}
				else if(abs(obj.x_middle_coord) <= 30){
					angleVal = 25*speed;
				}
				else if(abs(obj.x_middle_coord) <= 35){
					angleVal = 30*speed;
				}
				else if(abs(obj.x_middle_coord) <= 40){
					angleVal = 35*speed;
				}
				else if(abs(obj.x_middle_coord) > 40){
					angleVal = 50*speed;
				}
				if(obj.x_middle_coord < 0){
					angleVal *= -1;
				}
			}
		}
	  driveEnc = -enc.get_value();
	  float driveVal = pidCalculate(drivePID, target, driveEnc)*speed;

	  float rightVal = driveVal - angleVal;
	  float leftVal = driveVal + angleVal;

		if (direction == 1){
			set_drive(leftVal + DRIVEF, rightVal + DRIVEF);
			if(driveEnc >= target || ((pros::millis()-startTime) == time)){
		     atTarget = 1;
			}
		}
		else{
			set_drive(leftVal - DRIVEF, rightVal - DRIVEF);
			if(driveEnc <= target || ((pros::millis()-startTime) == time)){
		     atTarget = 1;
			}
		}
    pros::delay(20);
  }
}

void driveTrack(int target, float angle, int sig, int time, float correctionStrength, float speed){
  int atTarget = 0;
  float driveEnc = -enc.get_value();
	int direction = 1;
  int startTime = pros::millis();
	float angleVal = 0.0

	if (driveEnc > target){
		direction = -1;
	}

  while ((atTarget != 1) || (pros::millis()-startTime) < time) {
	  driveEnc = -enc.get_value();
	  float driveVal = pidCalculate(drivePID, target, driveEnc)*speed;
		if(crossedHalf((target - driveEnc), target)){
			angleVal = 0.0;
			if(vision.get_object_count() > 0){
				obj = vision.get_by_size(0); // Get largest object visible
				if(obj.signature == sig){
					if(abs(obj.x_middle_coord) <= 18){
						angleVal = 0;
					}
					else if(abs(obj.x_middle_coord) <= 25){
						angleVal = 20*speed;
					}
					else if(abs(obj.x_middle_coord) <= 30){
						angleVal = 25*speed;
					}
					else if(abs(obj.x_middle_coord) <= 35){
						angleVal = 30*speed;
					}
					else if(abs(obj.x_middle_coord) <= 40){
						angleVal = 35*speed;
					}
					else if(abs(obj.x_middle_coord) > 40){
						angleVal = 50*speed;
					}
					if(obj.x_middle_coord < 0){
						angleVal *= -1;
					}
				}
			}
		}
		else{
			angleVal = pidCalculate(anglePID, angle, getRotation())*correctionStrength;
		}

	  float rightVal = driveVal - angleVal;
	  float leftVal = driveVal + angleVal;

		if (direction == 1){
			set_drive(leftVal + DRIVEF, rightVal + DRIVEF);
			if(driveEnc >= target || ((pros::millis()-startTime) == time)){
		     atTarget = 1;
			}
		}
		else{
			set_drive(leftVal - DRIVEF, rightVal - DRIVEF);
			if(driveEnc <= target || ((pros::millis()-startTime) == time)){
		     atTarget = 1;
			}
		}
    pros::delay(20);
  }
}

void trackDrive(int target, float angle, int sig, int time, float correctionStrength, float speed){
  int atTarget = 0;
  float driveEnc = -enc.get_value();
	int direction = 1;
  int startTime = pros::millis();
	float angleVal = 0.0

	if (driveEnc > target){
		direction = -1;
	}

  while ((atTarget != 1) || (pros::millis()-startTime) < time) {
		if(vision.get_object_count() > 0){
			obj = vision.get_by_size(0); // Get largest object visible
			if(obj.signature == sig){
				if(abs(obj.x_middle_coord) <= 18){
					angleVal = 0.0;
				}
				else if(abs(obj.x_middle_coord) <= 25){
					angleVal = 20*speed;
				}
				else if(abs(obj.x_middle_coord) <= 30){
					angleVal = 25*speed;
				}
				else if(abs(obj.x_middle_coord) <= 35){
					angleVal = 30*speed;
				}
				else if(abs(obj.x_middle_coord) <= 40){
					angleVal = 35*speed;
				}
				else if(abs(obj.x_middle_coord) > 40){
					angleVal = 50*speed;
				}
				if(obj.x_middle_coord < 0){
					angleVal *= -1;
				}
			}
		}
		else{
			angleVal = pidCalculate(anglePID, angle, getRotation())*correctionStrength;
		}
		
	  driveEnc = -enc.get_value();
	  float driveVal = pidCalculate(drivePID, target, driveEnc)*speed;

	  float rightVal = driveVal - angleVal;
	  float leftVal = driveVal + angleVal;

		if (direction == 1){
			set_drive(leftVal + DRIVEF, rightVal + DRIVEF);
			if(driveEnc >= target || ((pros::millis()-startTime) == time)){
		     atTarget = 1;
			}
		}
		else{
			set_drive(leftVal - DRIVEF, rightVal - DRIVEF);
			if(driveEnc <= target || ((pros::millis()-startTime) == time)){
		     atTarget = 1;
			}
		}
    pros::delay(20);
  }
}

void rotate(float target, int time, float speed){
	int atTarget = 0;
  float driveEnc = getRotation();
  float distance = 0.0;
	int direction = 1;
  int startTime = pros::millis();

	if (driveEnc > target){
		direction = -1;
	}

	while ((atTarget != 1) || (pros::millis()-startTime) < time) {
	  driveEnc = getRotation();
	  distance = target - driveEnc;

	  float val = pidCalculate(turnPID, target, driveEnc)*speed;
	  float rightVal = val;
	  float leftVal = val;

		if (direction == 1){
			set_drive(leftVal, -rightVal);
			if(driveEnc >= target || ((pros::millis()-startTime) == time)){
		     atTarget = 1;
			}
		}
		else{
			set_drive(leftVal, -rightVal);
			if(driveEnc <= target || ((pros::millis()-startTime) == time)){
		     atTarget = 1;
			}
		}
    pros::delay(20);
  }
	reset_drive();
}

//INITIALIZE
void initRollers(){
	set_intake_brakes(MOTOR_BRAKE_HOLD);
	optical.set_led_pwm(100);
	pros::Task intake_task(intakeTask);
}

void initDrive(){
	imu.reset();
	pros::delay(2500);
	set_drive_brakes(MOTOR_BRAKE_COAST);
}

void initialize() {
	pros::lcd::initialize();
	pros::lcd::set_text(1, "Hello PROS User!");

	//Tasks
	initDrive();
	initRollers();

	//SENSORS
	vision.set_signature(1, &RED_SIG);
	vision.clear_led();
	vision.set_zero_point(pros::E_VISION_ZERO_CENTER);
	enc.reset();
}

void disabled() {}

void competition_initialize() {}

void autonomous() {
	drivePID = pidInit (DRIVEP, 0, DRIVED, 0, 100.0, 5, 15);
	turnPID = pidInit (TURNP, 0, TURND, 0, 10.0, 99999, 99999);
	anglePID = pidInit (ANGLEP, 0, 0, 0, 10.0, 99999, 99999);

	drive(2000, 0, 5000, 1, 1);
}

void opcontrol() {
	intakeMode = -1;
	while (true) {
		driveOp();
		rollerOp();
		pros::lcd::print(2, "%d", -enc.get_value());
		pros::lcd::print(3, "%lf", imu.get_rotation());

		//Vision Testing
		if(vision.get_object_count() > 0){
			obj = vision.get_by_size(0); // Get largest object visible
			if(obj.signature == 1){
				pros::lcd::print(4, "Red: %d", obj.x_middle_coord);
			}
			else if(obj.signature == 2){
				pros::lcd::print(4, "Blue: %d", obj.x_middle_coord);
			}
			else if(obj.signature == 3){
				pros::lcd::print(4, "Flag: %d", obj.x_middle_coord);
			}
			else{
				pros::lcd::print(4, "No Object Detected");
			}
		}

		if(master.get_digital(DIGITAL_B)){
			autonomous();
			pros::delay(60000);
		}
		pros::delay(20);
	}
}
