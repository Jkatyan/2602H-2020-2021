#include "main.h"

#define RFPORT 20
#define RBPORT 19
#define LFPORT 11
#define LBPORT 12
#define IMUPORT 13

pros::Motor RD(RFPORT);
pros::Motor RD2(RBPORT, true);
pros::Motor LD(LFPORT);
pros::Motor LD2(LBPORT, true);
pros::Imu imu(IMUPORT);

class Chassis

/*

The following must be in your program in order to use the Chassis class:

Global Scope:
	#define _right_front_port_ <port>
	#define _right_back_port_ <port>
	#define _left_front_port_ <port>
	#define _left_back_port_ <port>
  #define _imu_port_ <port>

	pros::Motor RD(_right_front_port_);
	pros::Motor RD2(_right_back_port_);
	pros::Motor LD(_left_front_port_);
	pros::Motor LD2(_left_back_port_);
  pros::Imu imu(_imu_port_);

*/

{
public:
  double wheelDiam, ticksPerRotation;
	double lEnc, rEnc;
	double xPos, yPos, angle;
//initialization
  void setChassis(double wheelDiam, double ticksPerRotation){
			this -> wheelDiam = wheelDiam;
      this -> ticksPerRotation = ticksPerRotation;
			this -> lEnc = 0.0;
			this -> rEnc = 0.0;
  }

	void initializeChassis(double xPos, double yPos){
		angle = 0.0;
		this -> xPos = xPos;
		this -> yPos = yPos;

    pros::delay(2000);
	}
//Odometry
	void updatePosition(){
		double deltaLeft = ((LD.get_position() - lEnc) / ticksPerRotation) * M_PI * wheelDiam;
		double deltaRight = ((RD.get_position() - rEnc) / ticksPerRotation) * M_PI * wheelDiam;
		yPos  += (((deltaLeft + deltaRight) / 2.0)) * cos(imu.get_heading() * M_PI / 180);
		xPos  += (((deltaLeft + deltaRight) / 2.0)) * sin(imu.get_heading() * M_PI / 180);
		lEnc = LD.get_position();
		rEnc = RD.get_position();
	}

	double getAngle(){
		return imu.get_heading();
	}

	double getX(){
		return xPos;
	}

	double getY(){
		return yPos;
	}

	void setX(double x){
    xPos = x;
  }

  void setY(double y){
    yPos = y;
  }

};

Chassis drive;

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */

void initialize() {
	drive.setChassis(3.25, 540);
	drive.initializeChassis(0.0, 0.0);
	pros::lcd::initialize();
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */

void opcontrol() {
	while(1){
		drive.updatePosition();
		pros::delay(20);
	}
}
