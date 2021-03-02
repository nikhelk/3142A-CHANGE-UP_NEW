#pragma once
#include "Config/chassis-config.h"


/// WE WOULD LIKE TO THANK 5225A FOR SHARING AND EXPLAINING THEIR ODOM SYSTEM AND CODE
struct pointVals
{
	double length;
	double theta;
};

typedef struct _pos
{
	double a;
	double y;
	double x;
	int leftLst;
	int rightLst;
	int backLst;
	double angleLst;
} sPos; // Position of the robot

void computeDistanceAndAngleToPoint(const long x, const long y, pointVals *out);
int trackPosition();


//POSITION TRACKING

extern double test2;
#define WHEEL_DIAMETER_IN_LR 2.75	// 2.843
#define WHEEL_DIAMETER_IN_S 2.75 // 2.843

// The distance between the tracking wheels and the centre of the robot in inches
#define L_DISTANCE_IN 6.1675
#define R_DISTANCE_IN 6.1675
#define S_DISTANCE_IN 6.8075

// The number of tick per rotation of the tracking wheel
#define TICKS_PER_ROTATION 360.0

// Used internally by trackPosition
#define SPIN_TO_IN_LR (WHEEL_DIAMETER_IN_LR * M_PI / TICKS_PER_ROTATION)
#define SPIN_TO_IN_S (WHEEL_DIAMETER_IN_S * M_PI / TICKS_PER_ROTATION)

/** 
 * enum positionVals
 * Stores odometry values
 */

enum positionVals
{
	ODOM_X,
	ODOM_Y,
	ODOM_THETA,
};

extern double positionArray[3];

/**
 * sets the desired initial odometry pose
 * @param x initial X
 * @param y initial Y
 * @param a initial Theta
 */

void setOdomOrigin(double x, double y, double a);


void printPosition();
int trackPositionGyro();

extern float thetaDegrees;