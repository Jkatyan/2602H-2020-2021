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
#define LINE_PORT 4
#define TRACKER_A 1
#define TRACKER_B 2
#define IMU_PORT 1
#define OPTICAL_PORT 18
#define VISION_PORT 21

//CONSTANTS
#define DRIVEP 0.09
#define DRIVED 0.01
#define DRIVEF 9

#define TURNP 0.67
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

pros::ADIAnalogIn line (LINE_PORT);
pros::ADIEncoder enc (TRACKER_A, TRACKER_B);
pros::Optical optical(OPTICAL_PORT);
pros::Imu imu (IMU_PORT);
pros::Vision vision (VISION_PORT);

pros::vision_object_s_t obj;
pros::vision_signature_s_t RED_SIG = {1, {1, 0, 0}, 3.000000, 6335, 7645, 6990, -541, 431, -56, 4598058, 0};
pros::vision_signature_s_t BLUE_SIG = {2, {1, 0, 0}, 3.000000, -3491, -2017, -2754, 8813, 13279, 11046, 2570882, 0};
pros::vision_signature_s_t FLAG_SIG = {3, {1, 0, 0}, 3.000000, -4491, -2803, -3646, -2821, 1, -1410, 6787717, 0};


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
}

void set_intakes(int intakes){
	if(intakes == 0){
		rightIntake.move_velocity(0);
		leftIntake.move_velocity(0);
	}
	else{
		rightIntake.move(intakes);
		leftIntake.move(intakes);
	}
}

void set_conveyor(int mid, int top){
	if(mid == 0){
		midRoller.move_velocity(0);
	}
	else{
		midRoller.move(mid);
	}
	if(top == 0){
		topRoller.move_velocity(0);
	}
	else{
		topRoller.move(top);
	}
}

void set_rollers(int intakes, int mid, int top){
	set_intakes(intakes);
	set_conveyor(mid, top);
}

int intakeTask(){
	while(true){
		switch(intakeMode){
			case 0:
				set_rollers(0, 0, 0);
			break;
			case 1:
				set_rollers(127, 127, -127);
			break;
			case 2:
				set_rollers(-127, 127, -127);
			break;
			case 3:
				set_rollers(0, 127, -127);
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
			case 8:
				set_rollers(-127, 0, 0);
			break;
			case -1:
				continue;
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

//DRIVE FUNCTIONS
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
			if(driveEnc >= target || ((pros::millis()-startTime) >= time)){
		     atTarget = 1;
			}
		}
		else{
			set_drive(leftVal - DRIVEF, rightVal - DRIVEF);
			if(driveEnc <= target || ((pros::millis()-startTime) >= time)){
		     atTarget = 1;
			}
		}
		pros::lcd::print(2, "%d", -enc.get_value());
		pros::lcd::print(3, "%lf", imu.get_rotation());
    pros::delay(20);
  }
	pros::lcd::print(2, "%d", -enc.get_value());
	pros::lcd::print(3, "%lf", imu.get_rotation());
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
			if(driveEnc >= target || ((pros::millis()-startTime) >= time)){
		     atTarget = 1;
			}
		}
		else{
			set_drive(leftVal - DRIVEF, rightVal - DRIVEF);
			if(driveEnc <= target || ((pros::millis()-startTime) >= time)){
		     atTarget = 1;
			}
		}
		pros::lcd::print(2, "%d", -enc.get_value());
		pros::lcd::print(3, "%lf", imu.get_rotation());
    pros::delay(20);
  }
	pros::lcd::print(2, "%d", -enc.get_value());
	pros::lcd::print(3, "%lf", imu.get_rotation());
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
		double angleVal = 0;
		if(vision.get_object_count() > 0){
			obj = vision.get_by_size(0); // Get largest object visible
			if(obj.signature == sig){
				if(abs(obj.x_middle_coord) <= 10){
					angleVal = 0;
				}
				else if(abs(obj.x_middle_coord) <= 15){
					angleVal = 15*speed;
				}
				else if(abs(obj.x_middle_coord) <= 20){
					angleVal = 20*speed;
				}
				else if(abs(obj.x_middle_coord) <= 25){
					angleVal = 30*speed;
				}
				else if(abs(obj.x_middle_coord) <= 30){
					angleVal = 40*speed;
				}
				else if(abs(obj.x_middle_coord) <= 35){
					angleVal = 45*speed;
				}
				else if(abs(obj.x_middle_coord) <= 40){
					angleVal = 50*speed;
				}
				else if(abs(obj.x_middle_coord) > 40){
					angleVal = 60*speed;
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
			if(driveEnc >= target || ((pros::millis()-startTime) >= time)){
		     atTarget = 1;
			}
		}
		else{
			set_drive(leftVal - DRIVEF, rightVal - DRIVEF);
			if(driveEnc <= target || ((pros::millis()-startTime) >= time)){
		     atTarget = 1;
			}
		}
    pros::delay(20);
		pros::lcd::print(2, "%d", -enc.get_value());
		pros::lcd::print(3, "%lf", imu.get_rotation());
  }
	pros::lcd::print(2, "%d", -enc.get_value());
	pros::lcd::print(3, "%lf", imu.get_rotation());
}

double fastTrack(int sig, float speed){
		double angleVal = 0;
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
	  return angleVal;
}

void driveTrack(int target, float angle, int sig, int time, float correctionStrength, float speed){
  int atTarget = 0;
  float driveEnc = -enc.get_value();
	int direction = 1;
  int startTime = pros::millis();
	float angleVal = 0.0;

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
		pros::lcd::print(2, "%d", -enc.get_value());
		pros::lcd::print(3, "%lf", imu.get_rotation());
    pros::delay(20);
  }
	pros::lcd::print(2, "%d", -enc.get_value());
	pros::lcd::print(3, "%lf", imu.get_rotation());
}

void trackDrive(int target, int sig, int trackTime, int time, float correctionStrength, float speed){
  int atTarget = 0;
  float driveEnc = -enc.get_value();
	int direction = 1;
  int startTime = pros::millis();
	float angleVal = 0.0;

	if (driveEnc > target){
		direction = -1;
	}

  while ((atTarget != 1) || (pros::millis()-startTime) < time) {
		if((pros::millis()-startTime) < trackTime){
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
				angleVal = 0.0;
			}
		}
		else{
			angleVal = 0.0;
		}

	  driveEnc = -enc.get_value();
	  float driveVal = pidCalculate(drivePID, target, driveEnc)*speed;

	  float rightVal = driveVal - angleVal;
	  float leftVal = driveVal + angleVal;

		if (direction == 1){
			set_drive(leftVal + DRIVEF, rightVal + DRIVEF);
			if(driveEnc >= target || ((pros::millis()-startTime) >= time)){
		     atTarget = 1;
			}
		}
		else{
			set_drive(leftVal - DRIVEF, rightVal - DRIVEF);
			if(driveEnc <= target || ((pros::millis()-startTime) >= time)){
		     atTarget = 1;
			}
		}
		pros::lcd::print(2, "%d", -enc.get_value());
		pros::lcd::print(3, "%lf", imu.get_rotation());
    pros::delay(20);
  }
	pros::lcd::print(2, "%d", -enc.get_value());
	pros::lcd::print(3, "%lf", imu.get_rotation());
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

	while ((atTarget != 1) || ((pros::millis()-startTime) < time)) {
	  driveEnc = getRotation();
	  distance = target - driveEnc;

	  float val = pidCalculate(turnPID, target, driveEnc)*speed;
	  float rightVal = val;
	  float leftVal = val;

		if (direction == 1){
			set_drive(leftVal + DRIVEF, -rightVal - DRIVEF);
			if(driveEnc >= target || ((pros::millis()-startTime) >= time)){
		     atTarget = 1;
			}
		}
		else{
			set_drive(leftVal - DRIVEF, -rightVal + DRIVEF);
			if(driveEnc <= target || ((pros::millis()-startTime) >= time)){
		     atTarget = 1;
			}
		}
		pros::lcd::print(2, "%d", -enc.get_value());
		pros::lcd::print(3, "%lf", imu.get_rotation());
    pros::delay(20);
  }
	pros::lcd::print(2, "%d", -enc.get_value());
	pros::lcd::print(3, "%lf", imu.get_rotation());
	reset_drive();
}

void rotateTrack(float target, int sig, int time, float speed){
  int atTarget = 0;
  float driveEnc = getRotation();
	int direction = 1;
  int startTime = pros::millis();
	float val = 0.0;
	float angleVal = 0.0;

	if (driveEnc > target){
		direction = -1;
	}

  while ((atTarget != 1) || (pros::millis()-startTime) < time) {
		if(vision.get_object_count() > 0){
			obj = vision.get_by_size(0); // Get largest object visible
			if(obj.signature == sig){
				if(abs(obj.x_middle_coord) <= 18){
					angleVal = 0;
				}
				else if(abs(obj.x_middle_coord) <= 25){
					angleVal = 40*speed;
				}
				else if(abs(obj.x_middle_coord) <= 30){
					angleVal = 70*speed;
				}
				else if(abs(obj.x_middle_coord) <= 35){
					angleVal = 90*speed;
				}
				else if(abs(obj.x_middle_coord) <= 40){
					angleVal = 110*speed;
				}
				else if(abs(obj.x_middle_coord) > 40){
					angleVal = 120*speed;
				}
				if(obj.x_middle_coord < 0){
					angleVal *= -1;
				}
			}
		}
		else{
			val = pidCalculate(turnPID, target, driveEnc)*speed;
		}

		if (direction == 1){
			set_drive(val, -val);
			if(driveEnc >= target || ((pros::millis()-startTime) == time)){
		     atTarget = 1;
			}
		}
		else{
			set_drive(val, -val);
			if(driveEnc <= target || ((pros::millis()-startTime) == time)){
		     atTarget = 1;
			}
		}
		pros::lcd::print(2, "%d", -enc.get_value());
		pros::lcd::print(3, "%lf", imu.get_rotation());
    pros::delay(20);
  }
	pros::lcd::print(2, "%d", -enc.get_value());
	pros::lcd::print(3, "%lf", imu.get_rotation());
}

void driveOp(){
	if(master.get_digital(DIGITAL_R1) && master.get_digital(DIGITAL_R2)){
		set_drive(master.get_analog(ANALOG_LEFT_Y) + fastTrack(1, 1), master.get_analog(ANALOG_RIGHT_Y) - fastTrack(1, 1));
	}
	else{
		set_drive(master.get_analog(ANALOG_LEFT_Y), master.get_analog(ANALOG_RIGHT_Y));
	}
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
	vision.set_signature(2, &BLUE_SIG);
	vision.set_signature(3, &FLAG_SIG);
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

/*
	█████████
	█░░░░░░░█
	█░░░░░░░█
	█░░░░░░░█
	█░█░░░░░█
	█░░░░░░░█
	█░░░░░░░█
	█░░░░░░░█
	█████████
	PART 1
*/

	reset_drive(); //Start + goal 1
	setState(4);
	pros::delay(300);

/*
	█████████
	█░░░░░░░█
	█░█░░░░░█
	█░░░░░░░█
	█░█░░░░░█
	█░░░░░░░█
	█░░░░░░░█
	█░░░░░░░█
	█████████
	PART 2
*/

	setState(1);
	drive(1500, -150, 1100, 0.1, 1);
	//track(2200, 1, 1500, 1); //Ball 1 and 2
	reset_drive();

	//drive(-900, -90, 1450, 1, 1); // Back up
	rotate(-150, 700, 1); //Go to goal 2
	track(1300, 2, 1325, 1);

	setState(5); //score
	pros::delay(575);
	setState(3);

/*
	█████████
	█░░░░░░░█
	█░█░█░░░█
	█░░░░░░░█
	█░█░░░░░█
	█░░░░░░░█
	█░░░░░░░█
	█░░░░░░░█
	█████████
	PART 3
*/

	driveSetState(-930, 4, -120, 1800, 1, 1); //back out

	rotate(-345, 1250, 1); //face next ball
	setState(1);

	trackDrive(1300, 1, 400, 1200, 1, 1);

	rotate(-452, 1050, 1); //go to goal 3
	track(1625, 1, 1800, 1);

	setState(5); //score
	pros::delay(375);
	setState(3);

/*
	█████████
	█░░░░░░░█
	█░█░█░█░█
	█░░░░░░░█
	█░█░░░░░█
	█░░░░░░░█
	█░░░░░░░█
	█░░░░░░░█
	█████████
	PART 4
*/

	drive(150, -450, 1200, 1, 1); //back out
	setState(4);
	pros::delay(275);
	setState(0);

	rotate(-387, 1000, 1); //ball
	setState(1);
	trackDrive(2900, 2, 1700, 2300, 1, 1);

	setState(5); //score goal 4
	pros::delay(600);

/*
	█████████
	█░░░░░░░█
	█░█░█░█░█
	█░░░░░░░█
	█░█░░░█░█
	█░░░░░░░█
	█░░░░░░░█
	█░░░░░░░█
	█████████
	PART 5
*/

	drive(1500, -375, 2000, 1, 1); //back out and release
	setState(0);

	rotate(-272, 1500, 1); //next ball
	setState(1);
	trackDrive(1900, 1, 500, 1400, 1, 1);

	rotate(-360, 1500, 1); //turn to goal
	track(1750, 2, 2800, 1);

	setState(5); //score goal 5
	pros::delay(475);

/*
	█████████
	█░░░░░░░█
	█░█░█░█░█
	█░░░░░░░█
	█░█░░░█░█
	█░░░░░░░█
	█░░░░░█░█
	█░░░░░░░█
	█████████
	PART 6
*/

	drive(325, -360, 2000, 1, 1); //back out
	setState(0);

	rotate(-297, 1500, 1); //go to goal 6
	setState(1);
	track(2850, 2, 3000, 1);

	setState(5); //score goal 6
	pros::delay(500);
	setState(3);

/*
	█████████
	█░░░░░░░█
	█░█░█░█░█
	█░░░░░░░█
	█░█░░░█░█
	█░░░░░░░█
	█░░░█░█░█
	█░░░░░░░█
	█████████
	PART 7
*/

	driveSetState(1300, 4, -300, 2000, 1, 1); //back out
	setState(4); //score goal 6
	pros::delay(500);
	setState(0);

	rotate(-180, 1500, 1); //next ball
	setState(1);
	trackDrive(1800, 1, 700, 1300, 1, 1);

	rotate(-271, 1500, 1); //turn to and score goal
	track(1500, 2, 2600, 1);

	setState(5); //score goal 7
	pros::delay(500);
	setState(3);

/*
	█████████
	█░░░░░░░█
	█░█░█░█░█
	█░░░░░░░█
	█░█░░░█░█
	█░░░░░░░█
	█░█░█░█░█
	█░░░░░░░█
	█████████
	PART 8
*/

	drive(100, -270, 1500, 1, 1); //back out

	setState(4); //release
	pros::delay(300);
	setState(0);

	rotate(-210, 1000, 1); //get ball
	setState(1);
	track(2970, 2, 2200, 1);

	setState(5); //score goal 8
	pros::delay(550);
	setState(3);

/*
	█████████
	█░░░░░░░█
	█░█░█░█░█
	█░░░░░░░█
	█░█░█░█░█
	█░░░░░░░█
	█░█░█░█░█
	█░░░░░░░█
	█████████
	PART 9
*/

	reset_drive();
	drive(-1200, -210, 2000, 1, 1); //back out and release
	setState(4);
	pros::delay(400);
	setState(0);

	rotate(-90, 1000, 1); //ball
	setState(1);
	trackDrive(2775, 1, 1900, 2300, 1, 1);

	rotate(46, 1000, 1); //descore goal
	setState(8);
	track(1200, 2, 1000, 1);
	pros::delay(2000);


	setState(5); //score
	drive(2760, 46, 200, 1, 1); //back out and release
	pros::delay(300);

	drive(0, 46, 2000, 1, 1); //back out and release

	pros::lcd::print(2, "%d", -enc.get_value());
	pros::lcd::print(3, "%lf", imu.get_rotation());
}

void opcontrol() {
	intakeMode = 0;
	pros::vision_signature_s_t sig = vision.get_signature(3);
	pros::Vision::print_signature(sig);
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
