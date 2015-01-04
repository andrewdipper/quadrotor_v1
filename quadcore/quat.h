/*
Copyright (C) 2014  Andrew Pratt
View the README
*/
#pragma once
#include "math.h"
#include "../hw/hw.h"

struct quaternion{
	float a;			//this is the angleish part
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

void normalize_quaternion(struct quaternion *q);
void mult_quaternion(struct quaternion *q1, struct quaternion *q2, struct quaternion *q3);
void clone_quaternion(struct quaternion *q1, struct quaternion *q2);
void conjugate_quaternion(struct quaternion *q1, struct quaternion *q2);
void vmult_quaternion(struct quaternion *q, struct vector *v1, struct vector *v2);
void vmult_quaternionr(struct quaternion *q, struct vector *v1, struct vector *v2);

float dot_product(struct vector *m, struct vector *n);
void cross_product(struct vector *m, struct vector *n, struct vector *result);

int normalize_vector(struct vector *v);
int add_vector(struct vector *m, struct vector *n, struct vector *result);
void scale_vector(float scale, struct vector *v);

void print_vect(struct vector *v);
void print_quaternion(struct quaternion *q);
void print_quatmatrix(struct quaternion *q);

void gyroquat(struct vector *v, float dt, struct quaternion *q, struct vector *corr);
void gyroquat2(struct vector *v, float dt, struct quaternion *q, struct vector *corr);

void clone_vector(struct vector *v1, struct vector *v2);
void print_ivect(struct ivector *v);

int inv_sora(struct quaternion *q, struct vector *v);




