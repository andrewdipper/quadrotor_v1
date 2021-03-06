/*
Copyright (C) 2014  Andrew Pratt
View the README
*/
#include "quat.h"

//make the sqrt(a^2 + b^2 + c^2 + d^2) = 1
void normalize_quaternion(struct quaternion *q){  	
	float bias;
	bias = q->a*q->a + q->b*q->b + q->c*q->c + q->d*q->d;
	bias = sqrt(bias);
	q->a /= bias;
	q->b /= bias;
	q->c /= bias;
	q->d /= bias;
}



//undefined if same quat as q1 & q3 or q2 & q3
//applies a rotation so it is non-commutative
void mult_quaternion(struct quaternion *q1, struct quaternion *q2, struct quaternion *q3){	
	q3->a = (q1->a * q2->a) - (q1->b * q2->b) - (q1->c * q2->c) - (q1->d * q2->d);
	q3->b = (q1->a * q2->b) + (q1->b * q2->a) + (q1->c * q2->d) - (q1->d * q2->c);
	q3->c = (q1->a * q2->c) - (q1->b * q2->d) + (q1->c * q2->a) + (q1->d * q2->b);
	q3->d = (q1->a * q2->d) + (q1->b * q2->c) - (q1->c * q2->b) + (q1->d * q2->a);
}


//make a clone of q1 in q2
void clone_quaternion(struct quaternion *q1, struct quaternion *q2){ 	
	q2->a = q1->a;
	q2->b = q1->b;
	q2->c = q1->c;
	q2->d = q1->d;
}

//conjugate stored in q2	
//gives the inverse rotation
void conjugate_quaternion(struct quaternion *q1, struct quaternion *q2){	
	q2->a = q1->a;
	q2->b = -q1->b;
	q2->c = -q1->c;
	q2->d = -q1->d;
}


//rotates v1 by quaternion q and stores the result in v2
//the quaternion is effectively turned into a rotation matrix and then multiplied with the vector
void vmult_quaternion(struct quaternion *q, struct vector *v1, struct vector *v2){ 
	v2->x =  v1->x*(1-2*q->c*q->c - 2*q->d*q->d)  +   v1->y*2*(q->b*q->c - q->a*q->d)       +   v1->z*2*(q->b*q->d + q->a*q->c);

	v2->y =  v1->x*2*(q->b*q->c + q->a*q->d)      +   v1->y*(1-2*q->b*q->b - 2*q->d*q->d)   +   v1->z*2*(q->c*q->d - q->a*q->b) ;

	v2->z =  v1->x*2*(q->b*q->d - q->a*q->c)      +   v1->y*2*(q->c*q->d + q->a*q->b)       +   v1->z*(1-2*q->b*q->b - 2*q->c*q->c);
}

float dot_product(struct vector *m, struct vector *n){
	float sum = 0;
	sum += m->x * n->x;
	sum += m->y * n->y;
	sum += m->z * n->z;
	return sum;
}

//undefined if n || m and result point to the same structure
void cross_product(struct vector *m, struct vector *n, struct vector *result){
	result->x = m->y*n->z - m->z*n->y; 
	result->y = m->z*n->x - m->x*n->z; 
	result->z = m->x*n->y - m->y*n->x; 

}


int add_vector(struct vector *m, struct vector *n, struct vector *result){
	result->x = m->x + n->x;
	result->y = m->y + n->y;
	result->z = m->z + n->z;
	
	return 0;
}

void scale_vector(float scale, struct vector *v){
	v->x *= scale;
	v->y *= scale;
	v->z *= scale;
}

//make a clone of q1 in q2
void clone_vector(struct vector *v1, struct vector *v2){ 	
	v2->x = v1->x;
	v2->y = v1->y;
	v2->z = v1->z;
}


int normalize_vector(struct vector *v){
	float bias;
	bias = v->x*v->x + v->y*v->y + v->z*v->z;
	if(bias == 0) return -1;
	bias = sqrt(bias);
	v->x /= bias;
	v->y /= bias;
	v->z /= bias;
	return 0;
}

//uses sora to generate a quaternion from angular velocities
void gyroquat(struct vector *v, float dt, struct quaternion *q, struct vector *corr){ //x y and z are total angles not rates
	struct vector tmp;
	
	if(corr != NULL){
		tmp.x = (v->x + corr->x) * dt;
		tmp.y = (v->y + corr->y) * dt;
		tmp.z = (v->z + corr->z) * dt;
	}else{
		tmp.x = v->x * dt;
		tmp.y = v->y * dt;
		tmp.z = v->z * dt;
	}

	float angle = sqrt(tmp.x*tmp.x + tmp.y*tmp.y + tmp.z*tmp.z);
	tmp.x /= angle;
	tmp.y /= angle;
	tmp.z /= angle;

	float sinangle = sin(angle/2.0);
	
	q->a = cos(angle/2.0);
	q->b = tmp.x * sinangle;
	q->c = tmp.y * sinangle;
	q->d = tmp.z * sinangle;

	normalize_quaternion(q);

}


//less accurate but faster estimation of quaternion given angular velocity
//assumptions as dt gets very small: 
//	cosine(angle/2) = 1
//	sin(x) = x
// perpendicular line dx thing
void gyroquat2(struct vector *v, float dt, struct quaternion *q, struct vector *corr){ 
	q->a = 1;
	q->b = (v->x + corr->x)*dt/2;
	q->c = (v->y + corr->y)*dt/2;
	q->d = (v->z + corr->z)*dt/2;

	normalize_quaternion(q);

}


///Inverse of SORA to throw into pid for correction
int inv_sora(struct quaternion *q, struct vector *v){

	float angle = acos(q->a)*2;
	float root = sqrt(1 - q->a*q->a);
	v->x = q->b * angle / root;
	v->y = q->c * angle / root;
	v->z = q->d * angle / root;
	
	return 0;
	
}


void print_vect(struct vector *v){
	outi0(v->x*10000);
	out0("    ");
	outi0(v->y*10000);
	out0("    ");
	outi0(v->z*10000);
	out0("    ");
	//out0("\n");
}


void print_ivect(struct ivector *v){
	outi0(v->x);
	out0("    ");
	outi0(v->y);
	out0("    ");
	outi0(v->z);
	out0("    ");
	out0("\n");
}


void print_quaternion(struct quaternion *q){
	outi0(q->a*10000);
	out0(",   ");
	outi0(q->b*10000);
	out0(",   ");
	outi0(q->c*10000);
	out0(",   ");
	outi0(q->d*10000);
	//out0("\n");
}

void print_quatmatrix(struct quaternion *q){

	outi0((1-2*q->c*q->c - 2*q->d*q->d)*10000); out0("   ");
	outi0((2*q->b*q->c - 2*q->d*q->a)*10000); out0("   ");
	outi0((2*q->b*q->d + 2*q->c*q->a)*10000); out0("   ");

	outi0((2*q->b*q->c + 2*q->d*q->a)*10000); out0("   ");
	outi0((1-2*q->b*q->b - 2*q->d*q->d)*10000); out0("   ");
	outi0((2*q->c*q->d - 2*q->b*q->a)*10000); out0("   ");

	outi0((2*q->b*q->d - 2*q->c*q->a)*10000); out0("   ");
	outi0((2*q->c*q->d + 2*q->b*q->a)*10000); out0("   ");
	outi0((1-2*q->b*q->b - 2*q->c*q->c)*10000); out0("  \n");


}







