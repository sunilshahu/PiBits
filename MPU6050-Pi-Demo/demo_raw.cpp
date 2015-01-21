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
double double_ax;
double double_ay;
double double_az;
double double_gx;
double double_gy;
double double_gz;
int16_t ax, ay, az;
int16_t gx, gy, gz;
double pitch = 0.0, roll = 0.0;
double pitch_cmp, roll_cmp;
double pitch_gyro;
double roll_gyro;
double pitch_accel;
double roll_accel;
struct timeval start, end;
long mtime, seconds, useconds, total_useconds;

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
	double PI=(double)(3.1415);
	// read raw accel/gyro measurements from device
	accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
	gettimeofday(&end, NULL);
	seconds  = end.tv_sec  - start.tv_sec;
	useconds = end.tv_usec - start.tv_usec;
	start = end;

	total_useconds = seconds * 1000000 + useconds;

	double_ax = ((double)ax)/acc_sensitivity_fs_2;
	double_ay = ((double)ay)/acc_sensitivity_fs_2;
	double_az = ((double)az)/acc_sensitivity_fs_2;
	double_gx = ((double)gx)/gyro_sensitivity_250;
	double_gy = ((double)gy)/gyro_sensitivity_250;
	double_gz = ((double)gz)/gyro_sensitivity_250;
	
	pitch_gyro = (double_gy*total_useconds)/1000000;
	roll_gyro = (double_gx*total_useconds)/1000000;

	/* accel works good, but pitch<->roll*/
	pitch_accel = atan(double_ay/sqrt(double_ax*double_ax + double_az*double_az))*180.0/PI;
	roll_accel = atan(double_ax/double_az)*180.0/PI;

	pitch = 0.98*(pitch + pitch_gyro) + 0.02*pitch_accel;
	roll = 0.98*(roll + roll_gyro) + 0.02*roll_accel;
	printf("diff: %ld ,roll    : %-6f pitch    : %-6f\n", total_useconds, roll, pitch);
}

int main()
{
	setup();
	for (;;)
		loop();
}

