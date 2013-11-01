/*
Copyright (C) 2013  Andrew Pratt
View the README
*/
#include "imu.h"





//analog devices adxl345
void accel_init(void){		
	i2c_start(I2C1);
	i2c_set(I2C1, 0x3a, 0x2d, 0x08);	//put device in measurement mode
	i2c_start(I2C1);	
	i2c_set(I2C1, 0x3a, 0x31, 0x0b);	//full resolution with range +-16 g
	i2c_start(I2C1);	
	i2c_set(I2C1, 0x3a, 0x2c, 0x0e);	//data output rate = 1600hz
}

// ITG-3200
void gyro_init(void){
	i2c_start(I2C1);
	i2c_set(I2C1, 0xd2, 0x16, 0x18);	//set to full range and 8khz sampling
	i2c_start(I2C1);
	i2c_set(I2C1, 0xd2, 0x15, 0x03);	//average 4 samples -> 2khz output data rate  
}

//HMC5883L
void mag_init(void){
	i2c_start(I2C1);
	i2c_set(I2C1, 0x3c, 0x00, 0x78);   	//average 8 samples .. 75hz output
	i2c_tx(I2C1, 0x40);			//set range to +- 1.9 Ga
	i2c_tx(I2C1, 0x00);			//continuous measurement mode

}


/*	Measures the acceleration vector (upright for gravity because the quad is accelerating up at 9.8m/s to stay at same position)
*	Right handed coordinate system
*	Axis are realigned to those of the quadrotor
*/

void accel_read(short *acc){  
	
	unsigned char temp;
	i2c_start(I2C1);

	i2c_tx(I2C1, 0x3a);			//accelerometer + W address
	i2c_tx(I2C1, 0x32);			//register address
	i2c_start(I2C1);
	i2c_tx(I2C1, 0x3b);			//accelerometer + R address
	
	i2c_rx(I2C1, &temp); 
	acc[Y] = temp;
	i2c_rx(I2C1, &temp); 
	acc[Y] |= (temp << 8);
	
	
	i2c_rx(I2C1, &temp); 
	acc[X] = temp;
	i2c_rx(I2C1, &temp); 
	acc[X] |= (temp << 8);
	acc[X] *= -1;

	i2c_rx(I2C1, &temp);
	acc[Z] = temp;
	i2c_rx(I2C1, &temp); 
	acc[Z] |= (temp << 8);
	
}

/*	Measures the angular velocity of the quadrotor
*	Uses a right handed coordinate system
*	Axis are aligned to those of the quadrotor
*/

void gyro_read(short *gyr){	
	unsigned char temp;	
	i2c_start(I2C1);

	i2c_tx(I2C1, 0xd2);		//gyro + W address
	i2c_tx(I2C1, 0x1d);		//register address
	i2c_start(I2C1);
	i2c_tx(I2C1, 0xd3);		//gyro + R address
	

	i2c_rx(I2C1, &temp);
	gyr[X] = (temp << 8);
	i2c_rx(I2C1, &temp);
	gyr[X] |= temp;

	i2c_rx(I2C1, &temp);
	gyr[Y] = (temp << 8);
	i2c_rx(I2C1, &temp);
	gyr[Y] |= temp;

	i2c_rx(I2C1, &temp);	
	gyr[Z] = (temp << 8);
	i2c_rx(I2C1, &temp);
	gyr[Z] |= temp;
	
}

/*	Measures magnetic vector
*	It should be pointing north and into the ground in northern hemisphere
*	Uses a right handed coordinate system
*	Axis are realigned to those of the quadrotor
*/	
void mag_read(short *mag){  
	unsigned char temp;
	i2c_start(I2C1);
	i2c_tx(I2C1, 0x3c);
	i2c_tx(I2C1, 0x03);
	i2c_start(I2C1);
	i2c_tx(I2C1, 0x3d);

	i2c_rx(I2C1, &temp);
	mag[X] = (temp << 8);
	i2c_rx(I2C1, &temp);
	mag[X] |= temp;
	mag[X] *= -1;  					//align with desired coordinate system

	i2c_rx(I2C1, &temp);
	mag[Z] = (temp << 8);
	i2c_rx(I2C1, &temp);
	mag[Z] |= temp;
	

	i2c_rx(I2C1, &temp);
	mag[Y] = (temp << 8);
	i2c_rx(I2C1, &temp);
	mag[Y] |= temp;
	mag[Y] *= -1;  					//align with desired coordinate system


	
}


/*	Calculates the static error of the gyroscope.  It is critical that the gyroscope doesn't move.  
*	gbias values are subtracted from subsequent measurements so the gyro measures 0 when there is
*	no rotation
*/
void calc_gbias(float *gbias){
	short read[3];
	gbias[0] = 0;
	gbias[1] = 0;
	gbias[2] = 0;
	int count = 0;
	for(count = 0; count < 1000; ++count){
		gyro_read(read);
		gbias[0] += read[0];
		gbias[1] += read[1];
		gbias[2] += read[2];
	} 
	
	gbias[0] /= 1000.0;
	gbias[1] /= 1000.0;
	gbias[2] /= 1000.0;

}


// converts X, Y, and Z int values into floats
void read_to_val(short *a, float *b){
	int i;
	for(i = 0; i < 3; ++i){
		b[i] = (float)a[i];
	}
}


