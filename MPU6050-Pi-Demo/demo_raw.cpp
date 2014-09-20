#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#include <sys/time.h>
#include <time.h>
#include <math.h>
#include <cstdlib>
#include <iostream>

/* complementory filter variables */
#define ACCELEROMETER_SENSITIVITY 8192.0
#define GYROSCOPE_SENSITIVITY 131.0
#define MY_PI 3.14159265359
struct timeval tv0, tv1;
unsigned long elapsed;

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 accelgyro;

int16_t ax, ay, az;
int16_t gx, gy, gz;
int16_t accData[3], gyrData[3];
float final_pitch, final_roll;

/*
 * Calculate sampling rate to use in Complementary Filter.
 */
void CalculatedT()
{
    gettimeofday(&tv1, NULL);
    elapsed = (tv1.tv_sec-tv0.tv_sec)*1000000 + tv1.tv_usec - tv0.tv_usec;
    tv0 = tv1;
}

/*
 * Complementary Filter
 * Used implemtation from Pieter-Jan
 * Original code at : http://www.pieter-jan.com/node/11
 * Thanks Pieter for the great blog post.
 */
void ComplementaryFilter(int16_t accData[3], int16_t gyrData[3], float *pitch, float *roll)
{
	float pitchAcc, rollAcc;
	float dt = (elapsed/1000000); /*ms to s*/

	// Integrate the gyroscope data -> int(angularSpeed) = angle
	*pitch += ((float)gyrData[0] / GYROSCOPE_SENSITIVITY) * dt; // Angle around the X-axis
	*roll -= ((float)gyrData[1] / GYROSCOPE_SENSITIVITY) * dt;    // Angle around the Y-axis

	// Compensate for drift with accelerometer data if !bullshit
	// Sensitivity = -2 to 2 G at 16Bit -> 2G = 32768 && 0.5G = 8192
	int forceMagnitudeApprox = abs(accData[0]) + abs(accData[1]) + abs(accData[2]);
	if (forceMagnitudeApprox > 8192 && forceMagnitudeApprox < 32768)
	{
		// Turning around the X axis results in a vector on the Y-axis
		pitchAcc = atan2f((float)accData[1], (float)accData[2]) * 180 / MY_PI;
		*pitch = *pitch * 0.98 + pitchAcc * 0.02;

		// Turning around the Y axis results in a vector on the X-axis
		rollAcc = atan2f((float)accData[0], (float)accData[2]) * 180 / MY_PI;
		*roll = *roll * 0.98 + rollAcc * 0.02;
	}
}


void setup() {
    // initialize device
    printf("Initializing I2C devices...\n");
    accelgyro.initialize();
    gettimeofday(&tv0, NULL);

    // verify connection
    printf("Testing device connections...\n");
    printf(accelgyro.testConnection() ? "MPU6050 connection successful\n" : "MPU6050 connection failed\n");
}

void loop() {
    // read raw accel/gyro measurements from device
    accelgyro.getMotion6(&accData[0], &accData[1], &accData[2], &gyrData[0], &gyrData[1], &gyrData[2]);
    CalculatedT();
    ComplementaryFilter(accData, gyrData, &final_pitch, &final_roll);
    printf("pitch = %6f  roll %6f dt = %lu\n", final_pitch, final_roll, elapsed);
}

int main()
{
    setup();
    for (;;) {
        loop();
    }
}

