/*
Copyright (C) 2013  Andrew Pratt
View the README
*/

#pragma once
#include "math.h"
#include "uart.h"

struct quaternion{
	float a;
	float b;
	float c;
	float d;
};

struct vector{
	float x;
	float y;
	float z;
};

struct ivector{
	int x;
	int y;
	int z;
};
//make the sqrt(a^2 + b^2 + c^2 + d^2) = 1
void normalize_quaternion(struct quaternion *q);

//undefined if same quat as q1 & q3 or q2 & q3
//combines two rotations into one so it is non-commutative
void mult_quaternion(struct quaternion *q1, struct quaternion *q2, struct quaternion *q3);

//make a clone of q1 in q2
void clone_quaternion(struct quaternion *q1, struct quaternion *q2);

//conjugate stored in q2	
//gives the inverse rotation
void conjugate_quaternion(struct quaternion *q1, struct quaternion *q2);

//rotates v1 by quaternion q and stores the result in v2
//the quaternion is effectively turned into a rotation matrix and then multiplied with the vector
void vmult_quaternion(struct quaternion *q, struct vector *v1, struct vector *v2);

//computes dot product of two vectors
float dot_product(struct vector *m, struct vector *n);

//computes cross product
//undefined if n || m and result point to the same structure
void cross_product(struct vector *m, struct vector *n, struct vector *result);

//make the sqrt(x^2 + y^2 + z^2) = 1
void normalize_vector(struct vector *v);


int add_vector(struct vector *m, struct vector *n, struct vector *result);

//scales vector by "scale"
void scale_vector(float scale, struct vector *v);

//print vector for debugging
void print_vect(struct vector *v);

//print quaternion for debugging
void print_quaternion(struct quaternion *q);

//print quaternion in matrix form for debugging
void print_quatmatrix(struct quaternion *q);

//uses Simultaneous Orthagonal Rotation Angle to generate a quaternion from angular velocities
void gyroquat(struct vector *v, float dt, struct quaternion *q, struct vector *corr);

//less accurate but faster estimation of quaternion given angular velocity
//assumptions as dt gets very small: 
//	cosine(angle/2) = 0
//	sin(x) = x
// perpendicular line estimation
void gyroquat2(struct vector *v, float dt, struct quaternion *q, struct vector *corr);

//make a clone of v1 in v2
void clone_vector(struct vector *v1, struct vector *v2);

//print integer vector for debugging
void print_ivect(struct ivector *v);

///Inverse of SORA to calculate simultaneous rotations about each axis
int inv_sora(struct quaternion *q, struct vector *v);




