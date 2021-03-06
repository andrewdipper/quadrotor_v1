/*
Copyright (C) 2014  Andrew Pratt
View the README
*/
#include "aux.h"


void init_busses(void){
	uart0_init(921600);
	NVIC_ENABLE_IRQ(IRQ_UART0_STATUS);

	uart1_init(115200);
	NVIC_ENABLE_IRQ(IRQ_UART1_STATUS);				

	i2ca0_init();
	NVIC_ENABLE_IRQ(IRQ_I2C0);

	pit_module_enable();
	pit0_set(48000000);	//1sec
}

float get_timeint(void){
	int tleft = pit0_read();
	pit0_set(48000000);
	return (48000000 - tleft)/48000000.0;	
}

void init_wdogs(void){
	pit1_set_int(48000);					//watchdog for sensor reads
	NVIC_ENABLE_IRQ(IRQ_PIT_CH1);
	NVIC_SET_PRIORITY(IRQ_PIT_CH1, 32);

	pit2_set_int(9600000);					//watchdog for radio link and if sensor does not correct itself (shuts down in 1/10th of a second)
	NVIC_ENABLE_IRQ(IRQ_PIT_CH2);
	NVIC_SET_PRIORITY(IRQ_PIT_CH2, 16);
}

void init_orientation(struct quad_state *curr, struct quad_state *past, struct quad_static *stat){
	struct quad_state *swap;

	float old_scale = stat->acc_scale;
	stat->acc_scale = 50;						//increase gain for faster initialization

	sensor_queue();	
	pit0_set(48000000);

	int cnt;
	for(cnt = 0; cnt < 5000; ++cnt){
		while(sensor_status() != 0);				//wait for the sensor values to be updated
		sensor_parse(curr->sdata);
		sensor_queue();						//the redundant queue that happens at last loop does not have any effect
		
		curr->dt = get_timeint();
		filter_data(curr, past);

		clear_correction(curr);
		accel_correction(curr, past, stat);

		gyroquat2((struct vector *)curr->sdata[1], curr->dt, &curr->uquat, &curr->correct_vect);
		mult_quaternion(&past->gbquat, &curr->uquat, &curr->gbquat);
		conjugate_quaternion(&curr->gbquat, &curr->bgquat);
		
		print_quaternion(&curr->gbquat);

		swap = curr;
		curr = past;
		past = swap;
	}
	//TODO ATTEMPT TO REDUCE INITIAL YAW ERROR
	struct quaternion qtmp;
	struct vector cv;
	clone_quaternion(&curr->gbquat, &qtmp);
	if(qtmp.a < 0){	//inverse sora doesn't handle -a very well because of arccos' range
		qtmp.a *= -1;	
		qtmp.b *= -1;	
		qtmp.c *= -1;	
		qtmp.d *= -1;	
	}
	
	inv_sora(&curr->gbquat, &cv);


	//TODO end

	stat->acc_scale = old_scale;

	gmagvect_compute(past, stat);

	out0("done\n");
	print_vect(&stat->gmagvect);
}


void set_motor(struct quad_static *stat){
	int i;
	for(i = 0; i < 4; ++i){
		if(stat->motor[i] > 1200){
			stat->motor[i] = 0;
		}else if(stat->motor[i] > 1000){
			stat->motor[i] = 1000;
		}else if(stat->motor[i] < 0){
			stat->motor[i] = 0;
		}
	}

	FTM0_C5V = 3000 + stat->motor[0]*3;					//start all at a 1 ms pulse
	FTM0_C6V = 3000 + stat->motor[1]*3;
	FTM0_C0V = 3000 + stat->motor[2]*3;
	FTM0_C1V = 3000 + stat->motor[3]*3;
}

void read_control(struct quad_static *stat){
	char pbuf[21];
	int i;
	char c;

	while(1){					//search for proper header
		c = 0x00;
		while(uart1_rxlen() >= 21 && uart1_rx(&c) == 0 && c != 0xff);
		if(c != 0xff) return;
		uart1_rx(&c);
		if(c == 0xff) break;
	}

	for(i = 0; i < 19; ++i){			//copy the payload
		if(uart1_rx(&pbuf[i]) != 0) break;
	}

	if(pbuf[18] != 0xee) return;			//tail incorrect so return all data is just thrown...TODO fix this to save data ???
	
	int tmp;
	for(i = 0; i < 9; ++i){
		stat->control[i] = (pbuf[2*i] << 8) | pbuf[2*i+1];
		
	}
	
	stat->controlflag = 1;
	pit2_set_int(9600000);	

	/*
	//TODO debug
	for(i = 0; i < 9; ++i){
		outi0(stat->control[i]);
		out0(" ");
	}
	out0("*\n");
	//TODO end
	*/
}

void clear_motor(struct quad_static *stat){
	stat->motor[0] = 0;
	stat->motor[1] = 0;
	stat->motor[2] = 0;
	stat->motor[3] = 0;
}

void print_motor(struct quad_static *stat){
	outi0(stat->motor[0]);	out0("  ");
	outi0(stat->motor[1]);	out0("  ");
	outi0(stat->motor[2]);	out0("  ");
	outi0(stat->motor[3]);	out0("  ");
	out0("\n");
}

void filter_data(struct quad_state *curr, struct quad_state *past){
	int i, j;
	for(i = 0; i < 3; ++i){
		for(j = 0; j < 3; ++j){
#if defined(SQUAD2S) || defined(SQUAD3S)
			curr->sdata[i][j] = lowpass(curr->sdata[i][j], past->sdata[i][j], curr->dt, 0.03);  //default .025
#endif 
#ifdef BQUAD
			curr->sdata[i][j] = lowpass(curr->sdata[i][j], past->sdata[i][j], curr->dt, 0.03);  //default .01
#endif
		}
	}

}

void increase_motor_refresh(void){
	FTM0_MOD = 12000;
}


void kill(void){
	while(1){
		FTM0_MOD = 60000;
		FTM0_C5V = 3000; 		//kill the motors
		FTM0_C6V = 3000; 
		FTM0_C0V = 3000; 
		FTM0_C1V = 3000; 
		out0("dead");
		delay_us(100000);
	}


}




