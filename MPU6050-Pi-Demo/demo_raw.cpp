#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include <math.h>
#include <sys/time.h>
#include "I2Cdev.h"
#include "MPU6050.h"

#include <stdlib.h>
#include <errno.h>
#include <string.h>
#include <wiringPi.h>
#include <softPwm.h>
#define PWM_RANGE		100


/* Motors direction and enable pins */
int enablea = 5;
int enableb = 6;
int a1 = 7;
int a2 = 0;
int b1 = 2;
int b2 = 3;

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


void InitMotors()
{
	// Set pins as outputs
	pinMode(a1, OUTPUT);
	pinMode(a2, OUTPUT);  
	pinMode(b1, OUTPUT);  
	pinMode(b2, OUTPUT); 

	// Set direction to none direction
	digitalWrite(a1,HIGH); 
	digitalWrite(a2,HIGH); 
	digitalWrite(b1,HIGH); 
	digitalWrite(b2,HIGH); 
	softPwmCreate (enablea , 0, PWM_RANGE);
	softPwmCreate (enableb , 0, PWM_RANGE);
}

void MotorControl(double out)
{
	unsigned int vel = abs(out);    // Absolute value of velocity

	// Sets direction
	if (out > 0) {              // forward
		digitalWrite(a1,HIGH);
		digitalWrite(a2,LOW);
		digitalWrite(b1,LOW);
		digitalWrite(b2,HIGH);
	} else {                     // backward
		digitalWrite(a1,LOW);
		digitalWrite(a2,HIGH);
		digitalWrite(b1,HIGH);
		digitalWrite(b2,LOW);
	}

	// Checks velocity fits the max ouptut range
	if (vel<0)
		vel=0;
	if (vel > 100)
		vel=100;

	// Writes the PWM 
	softPwmWrite (enablea, vel) ;
	softPwmWrite (enableb, vel) ;

}

void motor_direction_test()
{
	for (;;) {
		MotorControl(60);
		usleep(100000);
		MotorControl(-60);
		usleep(100000);
	}
}

void motor_speed_test()
{
	int j = 0, i = 1;
	for (;;) {
		j += i;
		printf("speed = %d\n", j*20);
		MotorControl(j*20);
		sleep(3);
		if (abs(j) == 5) {
			i = -i;
		}
	}
}


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

void mpu6050_setup() {
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

void mpu6050_loop() {
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
	//mpu6050_setup();
	wiringPiSetup()  ;
	InitMotors();

	for (;;) {
		motor_speed_test();
		//mpu6050_loop();
	}
}
