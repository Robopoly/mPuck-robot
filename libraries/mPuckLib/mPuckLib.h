#ifndef MPUCKLIB_H
#define MPUCKLIB_H

#include <avr/io.h>

#define ANGLE_CONSTANT		2	// in a tenth of radiant ((whelRadius/wheelSpace) * (2pi / 18) * 10)
#define DISTANCE_CONSTANT	6	// in mm ((whelRadius/2) * (2pi / 18) * 10)

#define KP_RHO		1
#define KP_ALPHA	1
#define KP_BETA		1

#define MOTOR_LIMIT 20

const int sinComp[32] = {0,20,39,56,72,84,93,99,100,97,91,81,68,52,33,14,-6,-26,
						-44,-61,-76,-87,-95,-99,-100,-96,-88,-77,-63,-46,-28,-8};	// steps of 0.2 radians result is *100

const int cosComp[32] = {100,98,92,83,70,54,36,17,-3,-23,-42,-59,-74,-86,-94,-99,
						-100,-97,-90,-79,-65,-49,-31,-11,9,28,47,63,78,88,96,100};	// steps of 0.2 radians result is *100

void readDistanceIR(int* irSensor);
int getLeftFloor(void);
int getRightFloor(void);
int getSwitch(void);
int getPotentiometer(void);
void braitenbergSpeed(int* irSensor, int* speedLeft,int* speedRight);
void resetEncoders(unsigned int* encLeft, unsigned int* encRight);
void odometry(unsigned int encLeft, unsigned int encRight, int32_t* posX, int32_t* posY, int* theta);
void controller(int32_t posX, int32_t posY, int theta, int32_t destinationX, int32_t destinationY, int destinationTheta, int* speedRight, int* speedLeft);
#endif