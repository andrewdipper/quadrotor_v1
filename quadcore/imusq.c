/*
Copyright (C) 2014  Andrew Pratt
View the README
*/
#include "imu.h"

//all sensors are oriented correctly

#if defined(SQUAD2S) || defined(SQUAD3S)

static struct i2c_transfer sensorblock[6];
static char sensor_buffer[3][6];


static int gyrobias[3] = {0, 0, 0};



//Accel config stuff
static char acc_cfg0[3] = {0x2c, 0x0f, 0x08};			//3200 hz
static char acc_cfg1[2] = {0x31, 0x0b};
static const char acc_waddr = 0xa6;
static const char acc_raddr = 0xa7;
static char acc_sreg = 0x32;

void accel_init(void){ 
	struct i2c_transfer init1, init2;

	init1.type = I2C_WRITE;
	init1.buflen = 3;
	init1.buffer = acc_cfg0;
	init1.address = acc_waddr;

	init2.type = I2C_WRITE;
	init2.buflen = 2;
	init2.buffer = acc_cfg1;
	init2.address = acc_waddr;

	
	i2ca0_qtrans(&init1);
	i2ca0_qtrans(&init2);

	while(init1.status != I2C_FINISHED) delay_us(1);
	while(init2.status != I2C_FINISHED) delay_us(1);
	
}

void accel_parse(char *read, int *data){			//3 int array {X,Y,Z} raw values
	int i;
	for(i = 0; i < 3; ++i){
		data[i] = (int16_t)((read[2*i] & 0xff) | ((read[2*i + 1] & 0xff) << 8)); 	//lsb first
	}

	data[0] *= -1;
	data[1] *= -1;

}





//Gyro config stuff
static char gyr_cfg0[2] = {0x20, 0xcf};
static char gyr_cfg1[2] = {0x23, 0x70};
static const char gyr_waddr = 0xd2;
static const char gyr_raddr = 0xd3;
static char gyr_sreg = 0x28 | (1 << 7);		//must have the msb in order to read multiple bytes...

void gyro_init(void){
	struct i2c_transfer init1, init2;

	init1.type = I2C_WRITE;
	init1.buflen = 2;
	init1.buffer = gyr_cfg0;
	init1.address = gyr_waddr;
	
	init2.type = I2C_WRITE;
	init2.buflen = 2;
	init2.buffer = gyr_cfg1;
	init2.address = gyr_waddr;	

	i2ca0_qtrans(&init1);
	i2ca0_qtrans(&init2);
	while(init1.status != I2C_FINISHED) delay_us(1);
	while(init2.status != I2C_FINISHED) delay_us(1);
}

void gyro_parse(char *read, int *data){			//3 int array {X,Y,Z} raw values
	int i;
	for(i = 0; i < 3; ++i){
		data[i] = (int16_t)((read[2*i + 1] & 0xff) | ((read[2*i] & 0xff) << 8)); 	//msb first
		
	}
	data[0] *= -1;
	data[1] *= -1;


	for(i = 0; i < 3; ++i){
		data[i] -= gyrobias[i]; 	
		
	}
}



void gyro_calc_bias(void){		//TODO perhaps do this as floats for better accuracy ???
	struct i2c_transfer g1, g2;
	char buf[6];
	int data[3];
	int sum[3] = {0,0,0};

	g1.type = I2C_WRITE;
	g1.buflen = 1;
	g1.buffer = &gyr_sreg;
	g1.address = gyr_waddr;

	g2.type = I2C_READ;
	g2.buflen = 6;
	g2.buffer = buf;
	g2.address = gyr_raddr;

	int i, j;


	out0("calcing bias");
	for(i = 0; i < 1000; ++i){
		outi0(i);
		i2ca0_qtrans(&g1);
		i2ca0_qtrans(&g2);
	
		while(g1.status != I2C_FINISHED || g2.status != I2C_FINISHED) delay_us(1);
		gyro_parse(buf, data);

		for(j = 0; j < 3; ++j){
			sum[j] += data[j];
		}
	}

	for(j = 0; j < 3; ++j){
		gyrobias[j] = (sum[j] + 500) / 1000;
	}


}



////////////////////////////////////////////////////////////////////////////////////////
static char mag_cfg0[4] = {0x00, 0x78, 0x40, 0x00};
static const char mag_waddr = 0x3c;
static const char mag_raddr = 0x3d;
static char mag_sreg = 0x03;

void mag_init(void){
	struct i2c_transfer init1;

	init1.type = I2C_WRITE;
	init1.buflen = 4;
	init1.buffer = mag_cfg0;
	init1.address = mag_waddr;	

	i2ca0_qtrans(&init1);
	while(init1.status != I2C_FINISHED) delay_us(1);
}

void mag_parse(char *read, int *data){
	int i;
	for(i = 0; i < 3; ++i){
		data[i] = (int16_t)((read[2*i + 1] & 0xff) | ((read[2*i] & 0xff) << 8)); 	//msb first
	}

	//realignments
	int tmp = data[1];	//flop y and z
	data[1] = data[2];
	data[2] = tmp;

	data[0] *= -1;
	data[1] *= -1;
}


//////////////////////////////////////////////////////////////////////////////////

void sensor_init(void){
	accel_init();
	gyro_init();
	mag_init();

	gyro_calc_bias();

	sensorblock[0].type = I2C_WRITE;
	sensorblock[0].buflen = 1;
	sensorblock[0].buffer = &acc_sreg;
	sensorblock[0].address = acc_waddr;

	sensorblock[1].type = I2C_READ;
	sensorblock[1].buflen = 6;
	sensorblock[1].buffer = sensor_buffer[0];
	sensorblock[1].address = acc_raddr;

	sensorblock[2].type = I2C_WRITE;
	sensorblock[2].buflen = 1;
	sensorblock[2].buffer = &gyr_sreg;
	sensorblock[2].address = gyr_waddr;

	sensorblock[3].type = I2C_READ;
	sensorblock[3].buflen = 6;
	sensorblock[3].buffer = sensor_buffer[1];
	sensorblock[3].address = gyr_raddr;

	sensorblock[4].type = I2C_WRITE;
	sensorblock[4].buflen = 1;
	sensorblock[4].buffer = &mag_sreg;
	sensorblock[4].address = mag_waddr;

	sensorblock[5].type = I2C_READ;
	sensorblock[5].buflen = 6;
	sensorblock[5].buffer = sensor_buffer[2];
	sensorblock[5].address = mag_raddr;
	
}

void sensor_queue(void){
	int i;
	for(i = 0; i < 6; ++i){
		i2ca0_qtrans(&sensorblock[i]);
	}
}

int sensor_status(void){
	int i;
	for(i = 0; i < 6; ++i){
		if(sensorblock[i].status != I2C_FINISHED) return -1;
	}
	return 0;
}

void print_sensor_status(void){
	int i;
	for(i = 0; i < 6; ++i){
		outi0(sensorblock[i].status);
		out0(" ");
	}
	out0("\n");
}
	

int sensor_parse(float data[3][3]){					//TODO make more efficient (don't need to do calculations as floats)
	int sens[3][3];
	accel_parse(sensorblock[1].buffer, sens[0]);
	gyro_parse(sensorblock[3].buffer, sens[1]);
	mag_parse(sensorblock[5].buffer, sens[2]);
	
	int i;
	for(i = 0; i < 3; ++i){						
		data[0][i] = (float)sens[0][i];
	}
	for(i = 0; i < 3; ++i){
		data[1][i] = (float)sens[1][i] * (3.14159265358979 / (14.375 * 180.0));
	}
	for(i = 0; i < 3; ++i){						
		data[2][i] = (float)sens[2][i];
	}

	if(normalize_vector((struct vector *) data[0]) != 0) 	return -1;		//normalize acceleration vector to unit					
	if(normalize_vector((struct vector *) data[2]) != 0)	return -1;		//normalize magnetometer vector to unit 
	
	


	return 0;
}

void sensor_print(void){
	int i, j;
	for(i = 1; i < 6; i+=2){
		for(j = 0; j < 6; ++j){
			outih0(sensorblock[i].buffer[j]);
			out0("   ");
		}
		out0("\n");	
	}
}

void gmagvect_compute(struct quad_state *curr, struct quad_static *stat){
	struct i2c_transfer g1, g2;
	char buf[6];
	int data[3];
	int sum[3] = {0,0,0};
	
	struct vector tmp;

	g1.type = I2C_WRITE;
	g1.buflen = 1;
	g1.buffer = &mag_sreg;
	g1.address = mag_waddr;

	g2.type = I2C_READ;
	g2.buflen = 6;
	g2.buffer = buf;
	g2.address = mag_raddr;

	int i, j;


	out0("calcing gmagvect");
	for(i = 0; i < 1000; ++i){
		outi0(i);
		i2ca0_qtrans(&g1);
		i2ca0_qtrans(&g2);
	
		while(g1.status != I2C_FINISHED || g2.status != I2C_FINISHED) delay_us(1);
		mag_parse(buf, data);

		for(j = 0; j < 3; ++j){
			sum[j] += data[j];
		}
	}

	
	for(j = 0; j < 3; ++j){
		((float *)&tmp)[j] = sum[j];
	}
	
	normalize_vector(&tmp);

	vmult_quaternion(&curr->gbquat, &tmp, &stat->gmagvect);
	//I think I need to multiply this by gbquat to move it from the body to global coordinates

}

#endif


