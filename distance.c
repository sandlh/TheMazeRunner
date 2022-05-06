/*
 * distance.c
 *
 *  Created on: Apr 25, 2022
 *      Author: BDEGROOT
 */
#include "sensors/proximity.h"
#include "distance.h"

#include "ch.h"
#include "hal.h"
#include <chprintf.h>
#include <usbcfg.h>
#include <msgbus/messagebus.h>

#include <math.h>


#define SENSOR_FRONT_RIGHT 0
#define SENSOR_BACK_RIGHT 3
#define SENSOR_BACK_LEFT 4
#define SENSOR_FRONT_LEFT 7
#define SENSOR_RIGHT 2
#define SENSOR_LEFT 5

#define SAFE_DISTANCE 120

#define THREAD_PERIOD 4 //[ms]


#define FRONT_RIGHT 0
#define FRONT_LEFT 1
#define BACK_RIGHT 2
#define BACK_LEFT 3
#define RIGHT 4
#define LEFT 5

#define NB_PROX_SENSOR 6
static uint16_t sensor_value[NB_PROX_SENSOR];
static int new_speed[2];

/***************************INTERNAL FUNCTIONS************************************/
static void update_data(void);

static THD_WORKING_AREA(distance_thd_wa, 512);
static THD_FUNCTION(distance_thd, arg) {

     (void) arg;
     chRegSetThreadName(__FUNCTION__);

	 systime_t time;

	while(1){
		time = chVTGetSystemTime();

		update_data();

		//chprintf((BaseSequentialStream *)&SD3, "dist_r = %d \n", sensor_value[RIGHT]);
		chThdSleepUntilWindowed(time, time + MS2ST(THREAD_PERIOD));

	}
}


static void update_data(void){
	sensor_value[FRONT_RIGHT] = get_calibrated_prox(SENSOR_FRONT_RIGHT);
	sensor_value[FRONT_LEFT] = get_calibrated_prox(SENSOR_FRONT_LEFT);
	sensor_value[BACK_RIGHT] = get_calibrated_prox(SENSOR_BACK_RIGHT);
	sensor_value[BACK_LEFT] = get_calibrated_prox(SENSOR_BACK_LEFT);
	sensor_value[RIGHT] = get_calibrated_prox(SENSOR_RIGHT);
	sensor_value[LEFT] = get_calibrated_prox(SENSOR_LEFT);
}



/*************************END INTERNAL FUNCTIONS**********************************/


/****************************PUBLIC FUNCTIONS*************************************/
void distance_start(void){
	proximity_start(); //starts IR sensors
	calibrate_ir(); //calibrates IR sensors

	chThdCreateStatic(distance_thd_wa, sizeof(distance_thd_wa), NORMALPRIO+1, distance_thd, NULL);
}

bool is_there_obstacle(void){
	if(	(sensor_value[FRONT_RIGHT] > SAFE_DISTANCE) ||		//check if one of IR sensor value > safe_distance
		(sensor_value[FRONT_LEFT] > SAFE_DISTANCE) ||		// = check if there's obstacle right in front of one of IR sensor
		(sensor_value[BACK_RIGHT] > SAFE_DISTANCE) ||
		(sensor_value[BACK_LEFT] > SAFE_DISTANCE) ||
		(sensor_value[RIGHT] > SAFE_DISTANCE) ||
		(sensor_value[LEFT] > SAFE_DISTANCE) ||
		((sensor_value[FRONT_RIGHT]+sensor_value[FRONT_LEFT]) > SAFE_DISTANCE) ||  //check if the sum of two nearby sensor > safe_distance
		((sensor_value[FRONT_RIGHT]+sensor_value[RIGHT]) > SAFE_DISTANCE) ||		 // = check if there's obstacle between two sensors
		((sensor_value[RIGHT]+sensor_value[BACK_RIGHT]) > SAFE_DISTANCE) ||
		((sensor_value[BACK_RIGHT]+sensor_value[BACK_LEFT]) > SAFE_DISTANCE) ||
		((sensor_value[BACK_LEFT]+sensor_value[LEFT]) > SAFE_DISTANCE) ||
		((sensor_value[LEFT]+sensor_value[FRONT_LEFT]) > SAFE_DISTANCE)){
		return true;
	}
	return false;
}

uint8_t index_highest_sensor_value(void){
	uint16_t max=0;
	uint8_t max_sensor_index=0;
	for(uint8_t i=0; i<NB_PROX_SENSOR;i++){
		if(sensor_value[i]>max){
			max=sensor_value[i];
			max_sensor_index = i;
		}
	}
	return max_sensor_index;
}

void avoid_obstacle(int* speed){
	uint8_t index_sensor_max = index_highest_sensor_value();
	static int16_t error_right=0;
	static int16_t error_right_int=0;
	static int16_t error_lateral = 0;
	switch(index_sensor_max){
		case FRONT_RIGHT :
		case BACK_RIGHT :
		case RIGHT :
			error_lateral = 180 - sensor_value[RIGHT];
			error_right=sensor_value[FRONT_RIGHT]-sensor_value[BACK_RIGHT];
			error_right_int+=error_right;
			if(error_right_int>50){
				error_right_int=50;
			}
			if(error_lateral>0){
				new_speed[0]=speed[0];
				new_speed[1]=speed[0]-7*error_right-1*error_right_int;
				chprintf((BaseSequentialStream *)&SD3, "sup 0  error = %d  left = %d error_int= %d ", error_right, new_speed[1],error_right_int);
			}else{
				new_speed[0]=speed[0];
				new_speed[1]=speed[0]+8*error_lateral;
				error_right_int=0;
				chprintf((BaseSequentialStream *)&SD3, "right = %d  left = %d error_lat= %d  sensor_right= %d ", new_speed[0], new_speed[1],error_lateral,sensor_value[RIGHT]);
			}

			break;
		case FRONT_LEFT :
			//new_speed[0]=-200;
			//new_speed[1]=200;
			break;
		case BACK_LEFT :
			/*if(sensor_value[LEFT]>sensor_value[BACK_RIGHT]){
				return 600;
			}else{
				return -600;
			}*/
			break;
		case LEFT :
			break;
		default :
			break;
	}
}


uint16_t get_distance_front_right(void){
	return sensor_value[FRONT_RIGHT];
}

uint16_t get_distance_front_left(void){
	return sensor_value[FRONT_LEFT];
}

uint16_t get_distance_back_right(void){
	return sensor_value[BACK_RIGHT];
}

uint16_t get_distance_back_left(void){
	return sensor_value[BACK_LEFT];
}

uint16_t get_distance_right(void){
	return sensor_value[RIGHT];
}

uint16_t get_distance_left(void){
	return sensor_value[LEFT];
}

int get_speed_right(void){
	return new_speed[0];
}

int get_speed_left(void){
	return new_speed[1];
}


/**************************END PUBLIC FUNCTIONS***********************************/


