#include "main.h"
#include "autonomous.hpp"

/*GYRO*/ pros::ADIGyro gyro (GYROPORT, GC);

PID drivePID; //Regular Drive PID
PID gyroDrivePID; //Gyro OP

float lastSlewTime;
float maxAccel = 0.16;
float lastSlewRate;

lv_obj_t * myLabelGyro;char bufG[100];
lv_obj_t * myLabelEnc;char bufE[100];
lv_obj_t * myLabelTarget;char bufT[100];

int preset1 = 0; //Red Preset
int preset2 = 0; //Blue Preset

void enablePreset(int preset){
  if(preset == 1){
    preset1 = 1;
    preset2 = 0;
  }
  else if(preset == 2){
    preset2 = 1;
    preset1 = 0;
  }
}
void rawDriveTarget(int side, float angle, int target, int time, int turn, float speed, bool slow, bool slew, float rightCorrection, float leftCorrection){
  angle = angle*side;
  int atTarget = 0;
  int driveEnc = 0;
  int distance = 0;
  int startTime = pros::millis();

  	while ((atTarget != 1) && (pros::millis()-startTime) < time) {
      if(turn == 0){
         driveEnc = ((abs((lB.get_position() + lF.get_position())/2))+(abs((rB.get_position() + rF.get_position()))/2))/2;
      }
      else{
         driveEnc = (((lB.get_position() + lF.get_position())/2)+((rB.get_position() + rF.get_position())/2))/2;
    }
    distance = target - driveEnc;
sprintf(bufG, "Gyro Value: %lf", gyro.get_value()/10.0);
lv_label_set_text(myLabelGyro, bufG);

sprintf(bufE, "Drive Encoder Value: %i", driveEnc);
lv_label_set_text(myLabelEnc, bufE);

sprintf(bufT, "Distance From Target: %i", distance);
lv_label_set_text(myLabelTarget, bufT);
//********** CALCULATIONS

      if (!slow){
      float val = pidCalculate(drivePID, target, -driveEnc)*speed;
      val = (slew)? slewRateCalculate(val): val;
      int rightVal = val - pidCalculate(gyroDrivePID, angle, gyro.get_value()/10.0);
      int leftVal = val + pidCalculate(gyroDrivePID, angle, gyro.get_value()/10.0);

      switch(turn){
        case 2:
        rF.move(rightCorrection*rightVal);
        rB.move(rightCorrection*rightVal);
        lF.move(leftCorrection*-leftVal);
        lB.move(leftCorrection*-leftVal);
        if(driveEnc == target){
          atTarget = 1;
          sprintf(bufT, "Distance From Target: %i", distance);
          lv_label_set_text(myLabelTarget, bufT);
        }
        break;
        case 0:
        rF.move(rightCorrection*rightVal);
        rB.move(rightCorrection*rightVal);
        lF.move(leftCorrection*leftVal);
        lB.move(leftCorrection*leftVal);
        if(driveEnc == target){
          atTarget = 1;
          sprintf(bufT, "Distance From Target: %i", distance);
          lv_label_set_text(myLabelTarget, bufT);
        }
        break;
        case 1:
        rF.move(rightCorrection*-rightVal);
        rB.move(rightCorrection*-rightVal);
        lF.move(leftCorrection*leftVal);
        lB.move(leftCorrection*leftVal);
        if(driveEnc == target){
          atTarget = 1;
          sprintf(bufT, "Distance From Target: %i", distance);
          lv_label_set_text(myLabelTarget, bufT);
        }
        break;
      }
      }
      else{
//********** CALCULATIONS

      float val = pidCalculate(drivePID, target, driveEnc)*speed;
      val = (slew)? slewRateCalculate(val): val;
      int rightVal = val - pidCalculate(gyroDrivePID, angle, gyro.get_value()/10.0);
      int leftVal = val + pidCalculate(gyroDrivePID, angle, gyro.get_value()/10.0);

      switch(turn){
        case 2:
        rF.move(rightCorrection*rightVal);
        rB.move(rightCorrection*rightVal);
        lF.move(leftCorrection*-leftVal);
        lB.move(leftCorrection*-leftVal);
        if(driveEnc == target){
          atTarget = 1;
          sprintf(bufT, "Distance From Target: %i", distance);
          lv_label_set_text(myLabelTarget, bufT);
        }
        break;
        case 0:
        rF.move(rightCorrection*rightVal);
        rB.move(rightCorrection*rightVal);
        lF.move(leftCorrection*leftVal);
        lB.move(leftCorrection*leftVal);
        if(driveEnc == target){
          atTarget = 1;
          sprintf(bufT, "Distance From Target: %i", distance);
          lv_label_set_text(myLabelTarget, bufT);
        }
        break;
        case 1:
        rF.move(rightCorrection*-rightVal);
        rB.move(rightCorrection*-rightVal);
        lF.move(leftCorrection*leftVal);
        lB.move(leftCorrection*leftVal);
        if(driveEnc == target){
          atTarget = 1;
          sprintf(bufT, "Distance From Target: %i", distance);
          lv_label_set_text(myLabelTarget, bufT);
        }
        break;
      }
      }
      pros::delay(15);
    } //While Loop

    distance = target - driveEnc;
    lF.set_zero_position(driveEnc-target);
    lB.set_zero_position(driveEnc-target);
    rF.set_zero_position(driveEnc-target);
    rB.set_zero_position(driveEnc-target);
    stop();

}
void motorTarget(int port, float p, float i, float d, int special, int target, int time, float speed, float accel, bool slew){
  PID taskPID = pidInit(p, i, d, 100.5, 4, 100000, 100000);
  pros::Motor motor(port);
  int atTarget = 0;
  int encoder = 0;
  int startTime = pros::millis();
	while ((atTarget != 1) && (pros::millis()-startTime) < time) {
  switch(special){case 0:encoder = motor.get_position();break;/*case 1:encoder = traypot.get_value();break;*/default:break;}
  sprintf(bufE, "Encoder Value: %i", encoder);
  lv_label_set_text(myLabelEnc, bufE);

  float val = pidCalculate(taskPID, target, encoder)*speed;
  val = (slew)? motorSlew(val, accel): val;
  motor.move(val);
  if(encoder == target){
    atTarget = 1;
  }
  pros::delay(15);
  }
  motor.move(0);

}
void rawTargetTurn(int target, int time, int turn, float speed, bool slow, bool slew, float rightCorrection, float leftCorrection){
  //angle = angle*side;
  int atTarget = 0;
  int driveEnc = 0;
  int distance = 0;
  int startTime = pros::millis();
    /*if(turn != 0){
      reset();
      stop();
    }*/
    while ((atTarget != 1) && (pros::millis()-startTime) < time) {
      if(turn != 0){
         driveEnc = ((abs((lB.get_position() + lF.get_position())/2))+(abs((rB.get_position() + rF.get_position()))/2))/2;
      }
      else{
         driveEnc = (((lB.get_position() + lF.get_position())/2)+((rB.get_position() + rF.get_position())/2))/2;
    }
    distance = target - driveEnc;
lv_label_set_text(myLabelGyro, bufG);

sprintf(bufE, "Drive Encoder Value: %i", driveEnc);
lv_label_set_text(myLabelEnc, bufE);

sprintf(bufT, "Distance From Target: %i", distance);
lv_label_set_text(myLabelTarget, bufT);
//********** CALCULATIONS

      if (!slow){
      float val = pidCalculate(drivePID, target, driveEnc)*speed;
      val = (slew)? slewRateCalculate(val): val;
      int rightVal = val;// - pidCalculate(gyroDrivePID, angle, gyro.get_value()/10.0);
      int leftVal = val;// + pidCalculate(gyroDrivePID, angle, gyro.get_value()/10.0);

      switch(turn){
        case 2:
        rF.move(rightCorrection*rightVal);
        rB.move(rightCorrection*rightVal);
        lF.move(leftCorrection*-leftVal);
        lB.move(leftCorrection*-leftVal);
        if(driveEnc == target){
          atTarget = 1;
          sprintf(bufT, "Distance From Target: %i", distance);
          lv_label_set_text(myLabelTarget, bufT);
        }
        break;
        case 0:
        rF.move(rightCorrection*rightVal);
        rB.move(rightCorrection*rightVal);
        lF.move(leftCorrection*leftVal);
        lB.move(leftCorrection*leftVal);
        if(driveEnc == target){
          atTarget = 1;
          sprintf(bufT, "Distance From Target: %i", distance);
          lv_label_set_text(myLabelTarget, bufT);
        }
        break;
        case 1:
        rF.move(rightCorrection*-rightVal);
        rB.move(rightCorrection*-rightVal);
        lF.move(leftCorrection*leftVal);
        lB.move(leftCorrection*leftVal);
        if(driveEnc == target){
          atTarget = 1;
          sprintf(bufT, "Distance From Target: %i", distance);
          lv_label_set_text(myLabelTarget, bufT);
        }
        break;
      }
      }
      else{
//********** CALCULATIONS

      float val = pidCalculate(drivePID, target, driveEnc)*speed;
      val = (slew)? slewRateCalculate(val): val;
      int rightVal = val;// - pidCalculate(gyroDrivePID, angle, gyro.get_value()/10.0);
      int leftVal = val;// + pidCalculate(gyroDrivePID, angle, gyro.get_value()/10.0);

      switch(turn){
        case 2:
        rF.move(rightCorrection*rightVal);
        rB.move(rightCorrection*rightVal);
        lF.move(leftCorrection*-leftVal);
        lB.move(leftCorrection*-leftVal);
        if(driveEnc == target){
          atTarget = 1;
          sprintf(bufT, "Distance From Target: %i", distance);
          lv_label_set_text(myLabelTarget, bufT);
        }
        break;
        case 0:
        rF.move(rightCorrection*rightVal);
        rB.move(rightCorrection*rightVal);
        lF.move(leftCorrection*leftVal);
        lB.move(leftCorrection*leftVal);
        if(driveEnc == target){
          atTarget = 1;
          sprintf(bufT, "Distance From Target: %i", distance);
          lv_label_set_text(myLabelTarget, bufT);
        }
        break;
        case 1:
        rF.move(rightCorrection*-rightVal);
        rB.move(rightCorrection*-rightVal);
        lF.move(leftCorrection*leftVal);
        lB.move(leftCorrection*leftVal);
        if(driveEnc == target){
          atTarget = 1;
          sprintf(bufT, "Distance From Target: %i", distance);
          lv_label_set_text(myLabelTarget, bufT);
        }
        break;
      }
      }
      pros::delay(15);
    } //While Loop
    /*if(turn != 0){
      reset();
      stop();
    }
    else{*/
    stop();
  //}
}
void gyroTurn(int target, int accuracy, int time, float speed) {
	int startTime = pros::millis();
	bool gyroAtTarget = false;
	int repsAtTarget = 0;
	//go into the loop that will repeat to update motor values and break when at target
	while (!gyroAtTarget  && (pros::millis()-startTime) < time) {
		// calculate the desired motor value based on the sensor value relative to the target
		float drive = pidCalculate(gyroDrivePID, target, gyro.get_value()/10.0)*speed;
		drive = ((fabs(gyro.get_value()/10.0-target)>180)? -1 : 1)*drive;
    lF.move(drive);
    lB.move(drive);
    rF.move(-drive);
    rB.move(-drive);
		//if the sensor value is within the desired range of the target
		if (fabs(gyro.get_value()/10.0-target) < accuracy) {
			//if the sensor value is within the range for multiple iterations of the loop where each loop is approximately 20ms
			if (repsAtTarget > 15) {
				//break out of the loop
				gyroAtTarget = true;
			}
			else {
				repsAtTarget++;
			}
		}
		else {
			repsAtTarget = 0;
		}
		pros::delay(15);
	}
	stop();
}
int driveTarget(int target, float angle, int time, float speed){
  if(preset1 == 1){
    rawDriveTarget (p1.side , angle, target, time, 0, speed, p1.slow , p1.slew , p1.rightCorrection , p1.leftCorrection);
    return 0;
  }
  else if(preset2 == 1){
    rawDriveTarget (p2.side , angle, target, time, 0, speed, p2.slow , p2.slew , p2.rightCorrection , p2.leftCorrection);
    return 0;
  }
  else{
    return 1;
  }} //Preset Code
int targetTurn(int target, int time, int turn, float speed){
  if(preset1 == 1){
    rawTargetTurn (target, time, turn, speed, p1.slow , p1.slew , p1.rightCorrection , p1.leftCorrection);
    return 0;
  }
  else if(preset2 == 1){
    rawTargetTurn (target, time, turn, speed, p2.slow , p2.slew , p2.rightCorrection , p2.leftCorrection);
    return 0;
  }
  else{
    return 1;
  }} //Preset Code
void stop(){
  lB.set_zero_position(lB.get_position());
  lF.set_zero_position(lF.get_position());
  rB.set_zero_position(rB.get_position());
  rF.set_zero_position(rB.get_position());
  lB.move(0);
  lF.move(0);
  rB.move(0);
  rF.move(0);} //Sets zero position / stops motors.
void reset(){
  stop();
  lB.tare_position();
  lF.tare_position();
  rB.tare_position();
  rF.tare_position();
  lB.move(0);
  lF.move(0);
  rB.move(0);
  rF.move(0);} //Resets Drive encoder values, Encoder error, and stops motors.
float slewRateCalculate (float desiredRate) {

		float deltaTime = pros::millis()-lastSlewTime;
		float desiredAccel = (desiredRate -lastSlewRate)/deltaTime;
		float addedRate;
		float newRate;

		if (fabs(desiredAccel) < maxAccel || (desiredAccel<0 && desiredRate>0) || (desiredAccel>0 && desiredRate<0)) {
		    addedRate = desiredAccel*deltaTime;
		    newRate = addedRate+lastSlewRate;
		}
		else {
		    addedRate = ((desiredAccel>0)? 1: -1)*maxAccel*deltaTime;
        newRate = addedRate+lastSlewRate;
		}
	  lastSlewTime = lastSlewTime+deltaTime;
	  lastSlewRate = newRate;

		float returnVal = newRate;
		return returnVal;} //1D Motion Profiler
float motorSlew (float desiredRate, float maxAccelTray) {

    		float deltaTime = pros::millis()-lastSlewTime;
    		float desiredAccel = (desiredRate -lastSlewRate)/deltaTime;
    		float addedRate;
    		float newRate;

    		if (fabs(desiredAccel) < maxAccelTray || (desiredAccel<0 && desiredRate>0) || (desiredAccel>0 && desiredRate<0)) {
    		    addedRate = desiredAccel*deltaTime;
    		    newRate = addedRate+lastSlewRate;
    		}
    		else {
    		    addedRate = ((desiredAccel>0)? 1: -1)*maxAccelTray*deltaTime;
            newRate = addedRate+lastSlewRate;
    		}
    	  lastSlewTime = lastSlewTime+deltaTime;
    	  lastSlewRate = newRate;

    		float returnVal = newRate;
    		return returnVal;}
/*void moveToPoint(float targetX, float targetY) {
          PID turnPID = pidInit (TURNKP, TURNKI, TURNKD, 0, 100.0,5,15);
        	bool atPoint = false;
        	float targetAngle =0;
        	float power =0;
        	float turnPower =0;
        	lastSlewTime = pros::millis();

        	while (!atPoint) {
        		updatePosition();

        		power = -pidCalculate(drivePID, 0, sqrt(pow(targetY-getY(),2) + pow(targetX-getX(),2)));
        		power = slewRateCalculate(power);

        		targetAngle = atan2f((targetY-getY()),(targetX-getX()))-M_PI/2;
        		if (targetAngle >= M_PI) {
        	    targetAngle-=2*M_PI;
        	  }
        	  else if (targetAngle <= -M_PI) {
        	    targetAngle+=2*M_PI;
        	  }

        		turnPower = ((fabs(targetAngle-getAngle())>M_PI)? -1: 1)*pidCalculate(turnPID, targetAngle, getAngle());

        		lF.move((power + turnPower)*LC);
            lB.move((power + turnPower)*LC);
            rF.move((power - turnPower)*RC);
            rB.move((power - turnPower)*RC);

        		pros::lcd::print(0, "X: %f", getX());
        		pros::lcd::print(1, "Y: %f", getY());
        		pros::lcd::print(2, "Angle: %f", getAngleDegrees());

        		pros::lcd::print(7, "Target Angle: %f", targetAngle*180/M_PI);


        		if (sqrt(pow(targetY-getY(),2) + pow(targetX-getX(),2)) < 3 || master.get_digital(DIGITAL_B)) {
        			atPoint = true;
        		}
        		pros::delay(10);
        	}
        }*/
