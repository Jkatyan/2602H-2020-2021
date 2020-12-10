#include "main.h"

//* custom drive
PID distancePID (CUSTOM_DKP, CUSTOM_DKD);
PID anglePID (CUSTOM_AKP, CUSTOM_AKD);

pros::Motor LF (abs(LEFT_FRONT), MOTOR_GEARSET_18, true, MOTOR_ENCODER_COUNTS);
pros::Motor LB (abs(LEFT_BACK), MOTOR_GEARSET_18, true, MOTOR_ENCODER_COUNTS);
pros::Motor RF (abs(RIGHT_FRONT), MOTOR_GEARSET_18, false, MOTOR_ENCODER_COUNTS);
pros::Motor RB (abs(RIGHT_BACK), MOTOR_GEARSET_18, false, MOTOR_ENCODER_COUNTS);

pros::Controller screen (CONTROLLER_MASTER);

pros::Imu imu (IMU_PORT);

void left_drive(int power) {
	LF.move(power * LC);
  LB.move(power * LC);
}

void right_drive(int power) {
	RF.move(power * RC);
  RB.move(power * RC);
}

void set_drive(int left, int right){
	if(left == 0 && right != 0){
		LF.move_velocity(0);
	  LB.move_velocity(0);
		right_drive(right);
	}
	else if(left != 0 && right == 0){
		RF.move_velocity(0);
	  RB.move_velocity(0);
		left_drive(left);
	}
	else{
  left_drive(left);
  right_drive(right);
	}
}

void stop(){
  LF.move_velocity(0);
  LB.move_velocity(0);
  RF.move_velocity(0);
  RB.move_velocity(0);
}

double distance_PID(double target, double distance){
  return distancePID.calculate(target, distance);
}

double angle_PID(double target, double distance){
  return anglePID.calculate(target, distance);
}

int left_pos(){
  return (LF.get_position() + LB.get_position()) / 2;
}

int right_pos(){
  return (RF.get_position() + RB.get_position()) / 2;
}

void setDriveBrakes (pros::motor_brake_mode_e_t mode){
  LF.set_brake_mode(mode);
  LB.set_brake_mode(mode);
  RF.set_brake_mode(mode);
  RB.set_brake_mode(mode);
}

void reset(){
  LF.tare_position();
  LB.tare_position();
  RF.tare_position();
  RB.tare_position();
  distancePID.reset();
  anglePID.reset();
}

bool isStopped(){
  while (!RF.is_stopped() || !RB.is_stopped() || !LF.is_stopped() || !LB.is_stopped()){
    return false;
    pros::delay(20);
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

void drive(double target, int time, float speed){

	double lastDistance = 0;
	double distance;
	int reps = 0;
  bool atTarget = false;
	int startTime = pros::millis();

  while(!atTarget && (pros::millis()-startTime) < time){

    distance = (left_pos() + right_pos()) / 2.0;

    double distancePower = distance_PID(target, distance);

		double leftPower = slewRateCalculate(distancePower);
    double rightPower = slewRateCalculate(distancePower);

    set_drive(leftPower * speed, rightPower * speed);

		if(lastDistance == distance){
			reps++;
		}
    if(distance == target || reps >= 15){
      atTarget = true;
    }

		lastDistance = distance;
    pros::delay(20);
  }
	//screen.print(0, 0, "Distance: %lf\n\n", distance);
}

void driveSetState(double target, int state, int time, float speed){

	double lastDistance = 0;
	double distance;
	int reps = 0;
  bool atTarget = false;
	int startTime = pros::millis();

  while(!atTarget && (pros::millis()-startTime) < time){

    distance = (left_pos() + right_pos()) / 2.0;

    double distancePower = distance_PID(target, distance);

		double leftPower = distancePower;
    double rightPower = distancePower;

    set_drive(leftPower * speed, rightPower * speed);

		if(lastDistance == distance){
			reps++;
		}

		if(crossedHalf(distance, target)){
			setState(state);
		}

		if(distance == target || reps >= 15){
      atTarget = true;
    }

		lastDistance = distance;
    pros::delay(20);
  }
	printf("Distance: %lf\n\n", distance);
  stop();
}

void driveAngle(double target, double heading, int time, float speed){

	double lastDistance = 0;
	double distance;
	int reps = 0;
  bool atTarget = false;
	int startTime = pros::millis();

  while(!atTarget && (pros::millis()-startTime) < time){

    distance = (left_pos() + right_pos()) / 2.0;
    double angle = imu.get_yaw();

    double distancePower = distance_PID(target, distance);
    double anglePower = angle_PID(heading, angle);

		double leftPower = distancePower + anglePower;
    double rightPower = distancePower - anglePower;

    set_drive(leftPower * speed, rightPower * speed);

		if(lastDistance == distance){
			reps++;
		}

		if(distance == target || reps >= 15){
      atTarget = true;
    }

		lastDistance = distance;
    pros::delay(20);
  }
	printf("Distance: %lf\n\n", distance);
  stop();
}

void fastDrive(double target, int speed){
	if (target < 0){
		speed = -speed;
	}
	reset();
	set_drive(speed, speed);

	if (target > 0){
		while (((left_pos() + right_pos()) / 2.0) < target){
			pros::delay(20);
		}
	}
	else{
		while (((left_pos() + right_pos()) / 2.0) > target){
			pros::delay(20);
		}
	}
}

void rotate(double target, int time, float speed){

	double lastAngle = imu.get_heading();
	int reps = 0;
	double angle;
  bool atTarget = 0;
  int startTime = pros::millis();

  while (!atTarget && (pros::millis()-startTime) < time) {
		angle = imu.get_yaw();
		double anglePower = angle_PID(target, angle) * speed;
    int distance = target - angle;

	 // anglePower = ((fabs(angle-target)>180)? -1 : 1) * anglePower;
	  //anglePower = slewRateCalculate(anglePower);

	  set_drive(anglePower, -anglePower);

		if(angle == lastAngle){
			reps++;
		}

	  if(angle == target || reps >= 15){
	     atTarget = 1;
	  }

		lastAngle = angle;
    pros::delay(20);
  }
	printf("Angle: %lf\n\n", angle);
  stop();
	reset();
}

void rotateRight(double target, int time, float speed){

	double lastAngle = imu.get_heading();
	int reps = 0;
  bool atTarget = 0;
  int startTime = pros::millis();

  while (!atTarget && (pros::millis()-startTime) < time) {
		double angle = imu.get_yaw();
		double anglePower = angle_PID(target, angle) * speed;
    int distance = target - angle;

	//  anglePower = ((fabs(angle-target)>180)? -1 : 1) * anglePower;
	  //anglePower = slewRateCalculate(anglePower);

	  set_drive(0, -anglePower);

		if(angle == lastAngle){
			reps++;
		}

	  if(angle == target || reps >= 15){
	     atTarget = 1;
	  }

		lastAngle = angle;
    pros::delay(20);
  }
  stop();
	reset();
}

void rotateLeft(double target, int time, float speed){

	double lastAngle = imu.get_heading();
	int reps = 0;
  bool atTarget = 0;
  int startTime = pros::millis();

  while (!atTarget && (pros::millis()-startTime) < time) {
		double angle = imu.get_yaw();
		double anglePower = angle_PID(target, angle) * speed;
    int distance = target - angle;

	  //anglePower = ((fabs(angle-target)>180)? -1 : 1) * anglePower;
	  //anglePower = slewRateCalculate(anglePower);

	  set_drive(anglePower, 0);

		if(angle == lastAngle){
			reps++;
		}

	  if(angle == target || reps >= 15){
	     atTarget = 1;
	  }

		lastAngle = angle;
    pros::delay(20);
  }
  stop();
	reset();
}


//* okapi drive
okapi::Controller controller;

//Okapi Controllers
MotorGroup leftMotors = {LEFT_FRONT, LEFT_BACK};
MotorGroup rightMotors = {RIGHT_FRONT, RIGHT_BACK};

std::shared_ptr<OdomChassisController> chassis = ChassisControllerBuilder()
    .withMotors(leftMotors, rightMotors)
    .withGains(
        {DRIVE_KP, 0, DRIVE_KD}, // Distance controller gains
        {TURN_KP, 0, TURN_KD}, // Turn controller gains
        {ANGLE_KP, 0, 0}  // Angle controller gains (helps drive straight)
    )
    .withDimensions(AbstractMotor::gearset::green, {{WHEEL_DIAM, WHEEL_TRACK}, imev5GreenTPR})
    .withOdometry() // use the same scales as the chassis (above)
    .buildOdometry(); // build an odometry chassis

auto profileController = AsyncMotionProfileControllerBuilder()
    .withLimits({
      VEL,  // Maximum linear velocity of the Chassis in m/s
      LIN,  // Maximum linear acceleration of the Chassis in m/s/s
      JER // Maximum linear jerk of the Chassis in m/s/s/s
    })
    .withOutput(chassis)
    .buildMotionProfileController();

void okapi_waitUntilSettled(){
  chassis->waitUntilSettled();
}

void okapi_resetPos(){
  chassis->setState({0_ft,0_ft,0_deg});
}

void okapi_setPos(QLength x, QLength y, QAngle deg){
  chassis->setState({x,y,deg});
}

void okapi_drive(QLength dist, int rpm){
  chassis->setMaxVelocity(rpm);
  chassis->moveDistance(dist);
}

void okapi_driveAsync(QLength dist, int rpm){
  chassis->setMaxVelocity(rpm);
  chassis->moveDistanceAsync(dist);
}

void okapi_driveToPoint(QLength x, QLength y, int rpm){
  chassis->setMaxVelocity(rpm);
  chassis->driveToPoint({y, x});
}

void okapi_driveToPoint(QLength x, QLength y, QAngle angle, int rpm){
  chassis->setMaxVelocity(rpm);
  chassis->driveToPoint({y, x});
  chassis->turnToAngle(angle);
}

void okapi_turn(QAngle deg, int rpm){
  chassis->setMaxVelocity(rpm);
  chassis->turnAngle(deg);
}

void okapi_turnToAngle(QAngle deg, int rpm){
  chassis->setMaxVelocity(rpm);
  chassis->turnToAngle(deg);
}

void okapi_arc(QLength x, QLength y, QAngle deg, bool reversed, int rpm){

  profileController->generatePath({{0_ft,0_ft,0_deg}, {y, x, deg}}, "A");
  (reversed)?profileController->setTarget("A", true, true):
             profileController->setTarget("A");
             profileController->waitUntilSettled();
             profileController->removePath("A"); //remove path once motion is complete.
}

void initDrive(){
	imu.reset();
  pros::delay(2000);
  setDriveBrakes(MOTOR_BRAKE_COAST);
}

void tankDrive(){
  chassis->getModel()->tank(controller.getAnalog(ControllerAnalog::leftY) * LC,
                            controller.getAnalog(ControllerAnalog::rightY) * RC);
}

void arcadeDrive(){
  chassis->getModel()->arcade(controller.getAnalog(ControllerAnalog::leftY) * LC,
                              controller.getAnalog(ControllerAnalog::leftX) * RC);
}
