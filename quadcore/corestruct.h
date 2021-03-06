/*
Copyright (C) 2014  Andrew Pratt
View the README
*/
#pragma once

struct pid{
	float px;
	float py;
	float pz;
	float dx;
	float dy;
	float dz;
	float ix;
	float iy;
	float iz;
};

struct quad_state{
	float sdata[3][3];
	struct quaternion bgquat;
	struct quaternion gbquat;
	struct quaternion uquat;
	struct quaternion gtquat;
	struct vector correct_vect;
	struct vector att_correct;
	struct vector int_correct;
	float dt;
};


struct quad_static{
	struct vector gmagvect;
	float acc_scale;
	float mag_scale;
	int motor[4];
	int control[9];
	int controlflag;
	float yaw_accumulated;
	struct pid pidval;
	float throttle;
};
