/*
 * distance.c
 *
 *  Created on: Apr 25, 2022
 *      Author: BDEGROOT
 */
#include "sensors/proximity.h"
#include "distance.h"
#include "orientation.h"

#include "ch.h"
#include "hal.h"
#include <chprintf.h>
#include <usbcfg.h>
#include <msgbus/messagebus.h>

#include <math.h>


#define SENSOR_FRONT_RIGHT 0 //#define to read the values from sensors
#define SENSOR_BACK_RIGHT 3
#define SENSOR_BACK_LEFT 4
#define SENSOR_FRONT_LEFT 7
#define SENSOR_RIGHT 2
#define SENSOR_LEFT 5

#define SAFE_DISTANCE 80 //security distance for wall following
#define DISTANCE_FOLLOW_WALL 120

#define ARW_MAX 50 //anti rewind max and min values for orientation PID
#define ARW_MIN -50

#define THREAD_PERIOD 4 //[ms]

#define FRONT_RIGHT 0 //#define to use the values from the sensor in the array sensor_value
#define FRONT_LEFT 1
#define BACK_RIGHT 2
#define BACK_LEFT 3
#define RIGHT 4
#define LEFT 5

#define MODE_FRONT_LEFT 0
#define MODE_FRONT_RIGHT 1
#define MODE_BACK_RIGHT 2
#define MODE_BACK_LEFT 3

#define NB_PROX_SENSOR 6

#define ZERO 0 //value to initialize parameters

static uint16_t sensor_value[NB_PROX_SENSOR];
//static int new_speed[2];
//static int8_t integrale=NULL;

static uint8_t mode_deplacement;

/***************************INTERNAL FUNCTIONS************************************/
static void update_data(void);
static int regulator_orientation(int16_t error);

static THD_WORKING_AREA(distance_thd_wa, 512);
static THD_FUNCTION(distance_thd, arg) {

     (void) arg;
     chRegSetThreadName(__FUNCTION__);

	 systime_t time;

	while(1){
		time = chVTGetSystemTime();

		update_data();

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
	mode_deplacement = get_mode_deplacement();
}

/*************************END INTERNAL FUNCTIONS**********************************/


/****************************PUBLIC FUNCTIONS*************************************/
void distance_start(void){
	proximity_start(); //starts IR sensors
	calibrate_ir(); //calibrates IR sensors

	chThdCreateStatic(distance_thd_wa, sizeof(distance_thd_wa), NORMALPRIO+1, distance_thd, NULL);
}

bool is_there_obstacle(void){
	float cos_gravity=get_cos_gravity();

	if(((sensor_value[FRONT_RIGHT] > SAFE_DISTANCE)||	//tests if obstacle towards front right and if gravity is in the same direction
		(sensor_value[RIGHT] > SAFE_DISTANCE)||
		((sensor_value[FRONT_RIGHT]+sensor_value[RIGHT]) > SAFE_DISTANCE)||
		((sensor_value[FRONT_RIGHT]+sensor_value[FRONT_LEFT])>SAFE_DISTANCE))&&
		(mode_deplacement == MODE_FRONT_RIGHT)){

		return true;

	}else if(((sensor_value[BACK_RIGHT] > SAFE_DISTANCE)||   //tests if obstacle towards back right and if gravity is in the same direction
			(sensor_value[RIGHT] > SAFE_DISTANCE)||
			((sensor_value[BACK_RIGHT]+sensor_value[RIGHT]) > SAFE_DISTANCE)||
			((sensor_value[BACK_RIGHT]+sensor_value[BACK_LEFT])>SAFE_DISTANCE))&&
			(mode_deplacement == MODE_BACK_RIGHT)){

			return true;

	}else if(((sensor_value[FRONT_LEFT] > SAFE_DISTANCE)||  //tests if obstacle towards front left and if gravity is in the same direction
			(sensor_value[LEFT] > SAFE_DISTANCE)||
			((sensor_value[FRONT_LEFT]+sensor_value[LEFT]) > SAFE_DISTANCE)||
			((sensor_value[FRONT_RIGHT]+sensor_value[FRONT_LEFT])>SAFE_DISTANCE))&&
			(mode_deplacement == MODE_FRONT_LEFT)){

			return true;

	}else if(((sensor_value[BACK_LEFT] > SAFE_DISTANCE)||	//tests if obstacle towards back left and if gravity is in the same direction
			(sensor_value[LEFT] > SAFE_DISTANCE)||
			((sensor_value[BACK_LEFT]+sensor_value[LEFT]) > SAFE_DISTANCE)||
			((sensor_value[BACK_RIGHT]+sensor_value[BACK_LEFT])>SAFE_DISTANCE))&&
			(mode_deplacement == MODE_BACK_LEFT)){

			return true;

	}else if((((sensor_value[FRONT_RIGHT])>SAFE_DISTANCE)||
			(sensor_value[RIGHT]>SAFE_DISTANCE)||
			(sensor_value[BACK_RIGHT]>SAFE_DISTANCE)||
			((sensor_value[BACK_RIGHT]+sensor_value[RIGHT]) > SAFE_DISTANCE)||
			((sensor_value[FRONT_RIGHT]+sensor_value[RIGHT]) > SAFE_DISTANCE))&&
			(cos_gravity<0.25)){
		return true;
	}else if((((sensor_value[FRONT_LEFT])>SAFE_DISTANCE)||
			(sensor_value[LEFT]>SAFE_DISTANCE)||
			(sensor_value[BACK_LEFT]>SAFE_DISTANCE)||
			((sensor_value[BACK_LEFT]+sensor_value[LEFT]) > SAFE_DISTANCE)||
			((sensor_value[FRONT_LEFT]+sensor_value[LEFT]) > SAFE_DISTANCE))&&
			(cos_gravity>(-0.25))){
		return true;
	}


	return false; //otherwise there is no obstacle to avoid
}

uint8_t index_highest_sensor_value(void){
	uint16_t max=ZERO;
	uint8_t max_sensor_index=ZERO;
	for(uint8_t i=0; i<NB_PROX_SENSOR;i++){
		if(sensor_value[i]>max){
			max=sensor_value[i];
			max_sensor_index = i;
		}
	}
	return max_sensor_index;
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

int16_t get_error_distance_to_wall_right(void){
	return (DISTANCE_FOLLOW_WALL - sensor_value[RIGHT]);
}

int16_t get_error_distance_to_wall_left(void){
	return (DISTANCE_FOLLOW_WALL - sensor_value[LEFT]);
}

int16_t get_error_orientation_right(void){
	return (sensor_value[FRONT_RIGHT]-sensor_value[BACK_RIGHT]);
}

int16_t get_error_orientation_left(void){
	return (sensor_value[FRONT_LEFT]-sensor_value[BACK_LEFT]);
}

/**************************END PUBLIC FUNCTIONS***********************************/


