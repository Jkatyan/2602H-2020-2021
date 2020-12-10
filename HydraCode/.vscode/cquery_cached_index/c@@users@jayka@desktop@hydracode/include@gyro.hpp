#ifndef GYRO_HPP
#define GYRO_HPP

struct gyro_config{
	float std_deviation;
	float avg;
	float volts_per_degree_per_second;
	char gyro_flipped;
};

extern float stdDeviation;
extern float average;
extern float volts;
extern char flipped;

typedef struct {
	struct gyro_config config;
	int port_number;
} Gyro;

float gyro_get_rate (Gyro gyro);

void
gyro_calibrate (Gyro gyro);

void
gyro_init (Gyro gyro, int port_number, char gyro_flipped);

#endif
