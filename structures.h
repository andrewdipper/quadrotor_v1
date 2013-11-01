/*
Copyright (C) 2013  Andrew Pratt
View the README
*/

#pragma once
#include "quat.h"



struct sensor_read{
	short x;
	short y;
	short z;
};

struct sensor_val{
	float x;
	float y;
	float z;
};

//not used (old)
struct position{
	struct sensor_read *acc;
	struct sensor_read *gyr;
	struct sensor_val *facc;
	struct sensor_val *fgyr;
	float axz;
	float ayz;
	float faxz;
	float fayz;
	float gx;  //rotation in deg/sec on axis
	float gy;
	float gz;
	float dt;	//time

};

//used for correcting gyro with accelerometer and magnetometer
struct correct_weight{
	float magcorr;
	float acccorr;
};


//stores state of quadrotor at current time
struct state{
	float dt;
	float corrdt;
	short read[3][3];
	float val[3][3];
	int igyr[3];
	struct quaternion *gbquat;   	//rotation to align global coordinate system with body coordinate system  
					//converting a vector's coordinates from global to body coordinates requires the OPPOSITE rotation
	struct quaternion *bgquat;  
	struct quaternion *uquat;  		//update quaternion

	struct vector *corrv;			//acc and mag correction vectors	
	struct correct_weight *corrweight;	//weight for adjustment of uquat from acc and mag correction 

	struct vector *intcorrect;  		//used for integration of error

	struct vector *correct;			//correction vector

	struct quaternion *gdquat;

	float yaw;	
	float accum_yaw;
};
	
//stores inputs from RC controller
struct controller{
	short controlflag;
	short throttle;
	short pitch;
	short roll;
	short yaw;
	short aux1;
	short aux2;
	short aux3;	
};

//not used
struct pid_w{
	float p;
	float i;
	float d;
};


//not used
struct pid{
	float px;
	float ix;
	float dx;
	float py;
	float iy;
	float dy;
	float pz;
	float iz;
	float dz;
};

struct ipid{
	int px;
	int ix;
	int dx;
	int py;
	int iy;
	int dy;
	int pz;
	int iz;
	int dz;
};


struct motorval{
	int a;
	int b;
	int c;
	int d;
};

//used for qyro bias
struct bias{
	float x;
	float y;
	float z;
};


