#ifndef GYRO_CPP
#define GYRO_CPP

#include "main.h"
#include "gyro.hpp"

float stdDeviation;
float average;
float volts;
char flipped;

//ignore data within n standard deviations of no motion average
#define GYRO_STD_DEVS 5

#define GYRO_OVERSAMPLE 2

//points or time in mSec that the gyro calibrates for
#define GYRO_CALIBRATION_POINTS 1500

float calibration_buffer [GYRO_CALIBRATION_POINTS];

float gyro_get_rate (Gyro gyro);

void
gyro_calibrate (Gyro gyro){
	float raw_average = 0.0;
	float std_deviation = 0.0;

	pros::ADIAnalogIn sensor (gyro.port_number);

	//calculate average gyro reading with no motion
	for(int i = 0; i < GYRO_CALIBRATION_POINTS; ++i){
		float raw = sensor.get_value();
		raw_average += raw;
		calibration_buffer [i] = raw;
		pros::delay (1);
	}
	raw_average /= GYRO_CALIBRATION_POINTS;
	gyro.config.avg = raw_average;
	average = gyro.config.avg;
	//calcuate the standard devation, or the average distance
	//from the average on the data read
	for (int i = 0; i < GYRO_CALIBRATION_POINTS; ++i)
		std_deviation += fabs (raw_average - calibration_buffer [i]);
	std_deviation /= (float) GYRO_CALIBRATION_POINTS;

	gyro.config.std_deviation = std_deviation;
	stdDeviation = gyro.config.std_deviation;
	gyro.config.volts_per_degree_per_second = 0.0011 * 1.515;
	volts = gyro.config.volts_per_degree_per_second;
}

void
gyro_init (Gyro gyro, int port_number, char gyro_flipped) {
	gyro.port_number = port_number;
	gyro.config.gyro_flipped = gyro_flipped;
	flipped = gyro.config.gyro_flipped;
	gyro_calibrate (gyro);
}

float
gyro_get_rate (Gyro gyro){

	pros::ADIAnalogIn sensor (gyro.port_number);

	float gyro_read = 0.0;

	#if defined (GYRO_OVERSAMPLE)
		if (GYRO_OVERSAMPLE > 0) {
			int sample_sum = 0;
			int n_samples = pow (4, GYRO_OVERSAMPLE);

			for (int i = 0; i < n_samples; ++i)
				sample_sum += sensor.get_value();
			gyro_read = (float) sample_sum / (float) n_samples;
		}
		else
			gyro_read = sensor.get_value();
	#else
		gyro_read = sensor.get_value();
	#endif

	//Difference from zero-rate value or the average calibration read
	float difference = gyro_read - average;

	//Difference fro zero-rate value, in volts
	float gyro_voltage = difference * 5.0 / 4095.0;

	if (fabs (difference) > GYRO_STD_DEVS * stdDeviation)
		if (flipped)
			return -1 * gyro_voltage / volts;
		else
			return gyro_voltage / volts;
	return 0;
}
#endif
