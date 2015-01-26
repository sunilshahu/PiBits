#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include <math.h>
#include <sys/time.h>
#include "I2Cdev.h"
#include "MPU6050.h"

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 accelgyro;

int gyro_sensitivity_250;
int acc_sensitivity_fs_2;
int16_t ax, ay, az;
int16_t gx, gy, gz;
double pitch = 0.0, roll = 0.0;
struct timeval start, end;
long mtime, seconds, useconds, total_useconds;
double acc_data[3], gyr_data[3];
double PI=(double)(3.1415926535);


void calculate_dt()
{
	gettimeofday(&end, NULL);
	seconds  = end.tv_sec  - start.tv_sec;
	useconds = end.tv_usec - start.tv_usec;
	if (useconds < 0) {
		useconds = 1000000 + useconds;
		seconds--;
	}	
	start = end;
	total_useconds = seconds * 1000000 + useconds;
}

void complementary_filter(double acc_data[3], double gyr_data[3], long dt, double *pitch, double *roll)
{
	double pitchAcc, rollAcc;               

	// Integrate the gyroscope data -> int(angularSpeed) = angle
	*pitch += (double)gyr_data[0] * dt / 1000000; // Angle around the X-axis
	*roll -= (double)gyr_data[1] * dt / 1000000;    // Angle around the Y-axis
	//printf("roll : %-6f    pitch : %-6f\n", *roll, *pitch);

	// Turning around the X axis results in a vector on the Y-axis
	pitchAcc = atan2((double)acc_data[1], (double)acc_data[2]) * 180 / PI;
	*pitch = *pitch * 0.98 + pitchAcc * 0.02;

	// Turning around the Y axis results in a vector on the X-axis
	rollAcc = atan2((double)acc_data[0], (double)acc_data[2]) * 180 / PI;
	*roll = *roll * 0.98 + rollAcc * 0.02;
} 

void setup() {
	// initialize device
	printf("Initializing I2C devices...\n");
	accelgyro.initialize();

	// verify connection
	printf("Testing device connections...\n");
	printf(accelgyro.testConnection() ? "MPU6050 connection successful\n" : "MPU6050 connection failed\n");
	gyro_sensitivity_250 = 131;
	acc_sensitivity_fs_2 = 16384;
	gettimeofday(&start, NULL);
}

void loop() {
	// read raw accel/gyro measurements from device
	accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
	calculate_dt();

	acc_data[0] = ((double)ax)/acc_sensitivity_fs_2;
	acc_data[1] = ((double)ay)/acc_sensitivity_fs_2;
	acc_data[2] = ((double)az)/acc_sensitivity_fs_2;
	gyr_data[0] = ((double)gx)/gyro_sensitivity_250;
	gyr_data[1] = ((double)gy)/gyro_sensitivity_250;
	gyr_data[2] = ((double)gz)/gyro_sensitivity_250;
	complementary_filter(acc_data, gyr_data, total_useconds, &pitch, &roll);
	printf("diff: %ld    roll : %-6f    pitch : %-6f\n", total_useconds, roll, pitch);
}

int main()
{
	setup();
	for (;;) {
		//usleep(40000);
		loop();
	}
}

