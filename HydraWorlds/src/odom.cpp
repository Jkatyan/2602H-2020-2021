#include "main.h"

float x,y,angle,diff = 0;
bool trackenabled = true;

//Math
double radToDeg(double rad){
  return rad/M_PI * 180.0;
}

double degToRad(double deg){
  return deg * M_PI / 180.0;
}

//IMU Filter
double filter(const double& currentVal, const double& lastVal){
  double filteredVal = currentVal - lastVal;
  if(fabs(filteredVal) < 0.01){
    filteredVal = 0;
  }
  return filteredVal;
}

//Tracking
void track(){
  //Reset Sensors
  rEncoder.reset();
  sEncoder.reset();

  //Robot Constants
  float Sr = -8.00, // distance from tracking center to middle of right wheel
        Ss = -4.125, // distance from tracking center to middle of the tracking wheel
        wheelDiameter = 2.75, // diameter of the side wheels being used for tracking
        trackingDiameter = 2.75, // diameter of the sideways tracking wheel
        scaleFactor = 1.0, // gear ratio, slop, etc. adjustment for right wheel
        scaleFactorS = 1.0, // gear ratio, slop, etc. adjustment for side wheel

  //Theta Values
        deltaTheta = 0, // change in angle between refresh cycles
        alpha = 0, // average angle robot took over interval (deltaTheta/2)
        theta = 0, // average angle between average angle and final
        radius = 0, // radius of the arc traveled by bot
        thetaFiltered = 0, // theta value that has been filtered
        thetaNew = 0, // current angle after filter
        thetaAvg = 0, // average theta of refresh
        curRotation = 0, // current raw angle
        lastRotation = 0, // previous interval raw angle

  //Encoder Values
        curRight = 0, // current rotations of right wheel
        curSide = 0, // current rotations of side wheel
        lastRight = 0, // previous interval rotations of right wheel
        lastSide = 0, // previous interval rotations of side wheel
        deltaRight = 0, // distance traveled by right wheel over interval in inches
        deltaSide = 0, // distance traveled by side wheel over interval in inches
        rightRadius = 0, // supporting value in calculation of rightCord
        rightCord = 0, // translation of right wheel
        sideRadius = 0, // supporting value in calculation of sideCord
        sideCord = 0, // translation of side wheel

        iL, iR, iB,
        lL, lR, lB,
        cL = 1, cR =1, cB = 1,
        Li = imuA.get_rotation(),
        Ri = imuB.get_rotation();

  while(trackenabled){

    //Robot angle calculations
    iL = (imuA.get_rotation() - Li - diff) * cL;
    iR = (imuB.get_rotation() - Ri - diff) * cR;

    curRotation = iR;
    if(iL > iR){
      curRotation = iL;
    }

    thetaFiltered += filter(curRotation, lastRotation);
    lastRotation = curRotation;
    thetaNew = degToRad(thetaFiltered);
    deltaTheta = thetaNew - angle;

    //Encoder position calculations
    curRight = rEncoder.get_value() * M_PI/360 * scaleFactor;
    curSide = sEncoder.get_value() * M_PI/360 * scaleFactorS;
    deltaRight = (curRight - lastRight)*(wheelDiameter);
    deltaSide = (curSide - lastSide)*(trackingDiameter);
    lastRight = curRight;
    lastSide = curSide;

    //Calculating wheel translations
    if(deltaTheta != 0){
      sideCord = (2*sin(deltaTheta/2))*(deltaSide/deltaTheta + Ss); //step 8
      rightCord = (2*sin(deltaTheta/2))*(deltaRight/deltaTheta + Sr);
    }
    else{
       rightCord = deltaRight;
       sideCord = deltaSide;
    }

    //Calculating average angle over interval
    thetaAvg = angle + deltaTheta/2;

    //Adding change in each polar axis to cartesian plane
    x += rightCord * sin(thetaAvg);
    y += rightCord * cos(thetaAvg);
    x += sideCord * -cos(thetaAvg);
    y += sideCord *  sin(thetaAvg);

    //Calculating values in this inverval
    angle += deltaTheta;

    pros::delay(10);
  }
}

//Tasks
void stopTracking(){
  if(tracking != nullptr){
    tracking->remove();
    delete tracking;
    tracking = nullptr;
  }
}
