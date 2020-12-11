#include "main.h"

float lastSlewTime;
float maxAccel = MAX_ACCEL;
float lastSlewRate = 0;

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
}
