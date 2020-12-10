#ifndef GYRO_CPP
#define GYRO_CPP

#include "main.h"
#include "motors.hpp"
#include "misc/GYRO.hpp"
/*
task gyroValCalc {

	prevTime = time1[T1];
	wait1Msec(1);
	currentTime = time1[T1];
	gyroValDiff = 0;
	gyroRate = gyro_get_rate(mainGyro);
	timeDiff = currentTime - prevTime;
	timeDiffSeconds = timeDiff / 1000;
	while(true) {
		//writeDebugStreamLine("%d", gyro_get_rate(mainGyro) );
		gyroscopeValue = SensorValue[in1];
		currentTime = time1[T1];
		gyroRate = gyro_get_rate(mainGyro);
		timeDiff = currentTime - prevTime;
		timeDiffSeconds = timeDiff / 1000;
		gyroValDiff = gyroRate * timeDiffSeconds;

		gyroValue += gyroValDiff;

		finalGyroValue = gyroValue * mainGyro.gyro_offset_constant;
		if(finalGyroValue < 0) {
			finalGyroValue = 360 + finalGyroValue; //add because gyroVal is currently negative to reverse that value
			gyroValue = finalGyroValue / mainGyro.gyro_offset_constant;
		}
		else if(finalGyroValue >= 360) {
			finalGyroValue = 360 - finalGyroValue;
			gyroValue = finalGyroValue / mainGyro.gyro_offset_constant;
		}
		prevTime = time1[T1];
		wait1Msec(1);
	}*/

#endif
