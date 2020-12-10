#include "misc/autonSel.h"
#include "misc/PID.hpp"
#include "autonomous.hpp"
#include "misc/gyro.hpp"

lv_obj_t * myLabelGyro;
char bufG[100];

lv_obj_t * myLabelEnc;
char bufE[100];

lv_obj_t * myLabelTarget;
char bufT[100];

Gyro gyro;

int preset1 = 0; //Red Preset
int preset2 = 0; //Blue Preset

pros::Controller master(pros::E_CONTROLLER_MASTER);
pros::Controller partner(pros::E_CONTROLLER_PARTNER);
pros::Motor lB(L2, pros::LBG, LBREV, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor lF(L1, pros::LFG, LFREV, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor rB(R2, pros::RBG, RBREV, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor rF(R1, pros::RFG, RFREV, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor mogo(MOGO, pros::MOGOG, MOGOREV, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor lift(LIFT, pros::LIFTG, LIFTREV, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor inta(INTA, pros::INTAG, INTAREV, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor intb(INTB, pros::INTBG, INTBREV, pros::E_MOTOR_ENCODER_DEGREES);

void autonomous() {

  //gyro_init(gyro, GYROPORT, 'f');

  if(setAuton == 0){}
  else{
  lv_obj_clean(lv_scr_act());

  myLabelGyro = lv_label_create(lv_scr_act(), NULL);
  lv_label_set_text(myLabelGyro, "Gyro Value:");
  lv_obj_align(myLabelGyro, NULL, LV_ALIGN_CENTER, 0, 0);

  myLabelEnc = lv_label_create(lv_scr_act(), NULL);
  lv_label_set_text(myLabelEnc, "Drive Encoder Value:");
  lv_obj_align(myLabelEnc, NULL, LV_ALIGN_CENTER, 0, 50);

  myLabelTarget = lv_label_create(lv_scr_act(), NULL);
  lv_label_set_text(myLabelTarget, "Distance From Target:");
  lv_obj_align(myLabelTarget, NULL, LV_ALIGN_CENTER, 0, -50);

  stop();
  reset();
  lastSlewTime = pros::millis();

drivePID = pidInit (0.2, 0.01, 0.1, 0, 100.0,5,15);
slowDrivePID = pidInit (0.21, 0, 0.1, 0, 100.0,5,15);
gyroDrivePID = pidInit(1.2, 0, 0.6, 0, 40,999999,9999999);
gyroPID = pidInit(2.3,  0, 0.38, 0, 10,99999,999999);

pros::delay(20);

switch(setAuton){
  case 1: //Red 1 1
  enablePreset(1);

  break;
  case 2: //Red 1 2
  enablePreset(1);

  break;
  case 3: //Red 2 1
  enablePreset(1);

  break;
  case 4: //Red 2 2
  enablePreset(1);

  break;
  case 5: //Red 3 1
  enablePreset(1);

  break;
  case 6: //Red 3 2
  enablePreset(1);

  break;
  case 7: //Red 4 1
  enablePreset(1);

  break;
  case 8: //Red 4 2
  enablePreset(1);

  break;
  case 9: //Blue 1 1
  enablePreset(2);

  break;
  case 10: //Blue 1 2
  enablePreset(2);

  break;
  case 11: //Blue 2 1
  enablePreset(2);

  break;
  case 12: //Blue 2 2
  enablePreset(2);

  break;
  case 13: //Blue 3 1
  enablePreset(2);

  break;
  case 14: //Blue 3 2
  enablePreset(2);

  break;
  case 15: //Blue 4 1
  enablePreset(2);

  break;
  case 16: //Blue 4 2
  enablePreset(2);

  break;
  case 17: //Programming Skills
  enablePreset(1);
  driveTarget(-1500, -133.5, 30000, 1);
  break;
}
}
}//Void Auton

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
    /*if(turn != 0){
      reset();
      stop();
    }*/
  	while ((atTarget != 1) && (pros::millis()-startTime) < time) {
      if(turn == 0){
         driveEnc = ((abs((lB.get_position() + lF.get_position())/2))+(abs((rB.get_position() + rF.get_position()))/2))/2;
      }
      else{
         driveEnc = (((lB.get_position() + lF.get_position())/2)+((rB.get_position() + rF.get_position())/2))/2;
    }
    distance = target - driveEnc;
sprintf(bufG, "Gyro Value: %lf", gyro_get_rate(gyro)/10.0);
lv_label_set_text(myLabelGyro, bufG);

sprintf(bufE, "Drive Encoder Value: %i", driveEnc);
lv_label_set_text(myLabelEnc, bufE);

sprintf(bufT, "Distance From Target: %i", distance);
lv_label_set_text(myLabelTarget, bufT);
//********** CALCULATIONS

      if (!slow){
      float val = pidCalculate(drivePID, target, -driveEnc)*speed;
      val = (slew)? slewRateCalculate(val): val;
      int rightVal = val - pidCalculate(gyroDrivePID, angle, gyro_get_rate(gyro)/10.0);
      int leftVal = val + pidCalculate(gyroDrivePID, angle, gyro_get_rate(gyro)/10.0);

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

      float val = pidCalculate(slowDrivePID, target, driveEnc)*speed;
      val = (slew)? slewRateCalculate(val): val;
      int rightVal = val - pidCalculate(gyroDrivePID, angle, gyro_get_rate(gyro)/10.0);
      int leftVal = val + pidCalculate(gyroDrivePID, angle, gyro_get_rate(gyro)/10.0);

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
    distance = target - driveEnc;
    lF.set_zero_position(driveEnc-target);
    lB.set_zero_position(driveEnc-target);
    rF.set_zero_position(driveEnc-target);
    rB.set_zero_position(driveEnc-target);
    stop();
  //}
} //Drive Target

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
sprintf(bufG, "[!] Gyro Value: %lf", gyro_get_rate(gyro)/10.0);
lv_label_set_text(myLabelGyro, bufG);

sprintf(bufE, "Drive Encoder Value: %i", driveEnc);
lv_label_set_text(myLabelEnc, bufE);

sprintf(bufT, "Distance From Target: %i", distance);
lv_label_set_text(myLabelTarget, bufT);
//********** CALCULATIONS

      if (!slow){
      float val = pidCalculate(drivePID, target, driveEnc)*speed;
      val = (slew)? slewRateCalculate(val): val;
      int rightVal = val;// - pidCalculate(gyroDrivePID, angle, gyro_get_rate(gyro)/10.0);
      int leftVal = val;// + pidCalculate(gyroDrivePID, angle, gyro_get_rate(gyro)/10.0);

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

      float val = pidCalculate(slowDrivePID, target, driveEnc)*speed;
      val = (slew)? slewRateCalculate(val): val;
      int rightVal = val;// - pidCalculate(gyroDrivePID, angle, gyro_get_rate(gyro)/10.0);
      int leftVal = val;// + pidCalculate(gyroDrivePID, angle, gyro_get_rate(gyro)/10.0);

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
} //Drive Target w/out Gyro

void gyroTurn(int target, int accuracy, int time, float speed) {
	int startTime = pros::millis();
	bool gyroAtTarget = false;
	int repsAtTarget = 0;
	//go into the loop that will repeat to update motor values and break when at target
	while (!gyroAtTarget  && (pros::millis()-startTime) < time) {
		// calculate the desired motor value based on the sensor value relative to the target
		float drive = pidCalculate(gyroPID, target, gyro_get_rate(gyro)/10.0)*speed;
		drive = ((fabs(gyro_get_rate(gyro)/10.0-target)>180)? -1 : 1)*drive;
    lF.move(drive);
    lB.move(drive);
    rF.move(-drive);
    rB.move(-drive);
		//if the sensor value is within the desired range of the target
		if (fabs(gyro_get_rate(gyro)/10.0-target) < accuracy) {
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
} //Gyro Turn

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
  }
} //Preset Code

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
  }
} //Preset Code

void stop(){
  lB.set_zero_position(lB.get_position());
  lF.set_zero_position(lF.get_position());
  rB.set_zero_position(rB.get_position());
  rF.set_zero_position(rB.get_position());
  lB.move(0);
  lF.move(0);
  rB.move(0);
  rF.move(0);
} //Sets zero position / stops motors.
void reset(){
  stop();
  lB.tare_position();
  lF.tare_position();
  rB.tare_position();
  rF.tare_position();
  lB.move(0);
  lF.move(0);
  rB.move(0);
  rF.move(0);
} //Resets Drive encoder values, Encoder error, and stops motors.
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
		return returnVal;
} //1D Motion Profiler
