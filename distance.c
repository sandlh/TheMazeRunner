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

#define SAFE_DISTANCE 100 //security distance for wall following

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

#define NULL 0 //value to initialize parameters

static uint16_t sensor_value[NB_PROX_SENSOR];
static int new_speed[2];
static int8_t integrale=NULL;

static uint8_t mode_deplacement;

/***************************INTERNAL FUNCTIONS************************************/
static void update_data(void);
static int regulator_orientation(int16_t error);
static uint8_t index_highest_sensor_value(void);

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

static int regulator_orientation(int16_t error)
{
	//PID
	uint8_t dt = THREAD_PERIOD;
	static uint8_t Kp = 4;
	static float Ki = 0.05;
	static uint8_t Kd = 20;

	static int16_t old_error = NULL;
	int16_t orientation_pid =NULL;

	static int16_t derivee = NULL;//debug

	integrale += Ki *error*dt;
	derivee = error-old_error;//debug

	if(integrale > ARW_MAX)
			integrale = ARW_MAX;
	else if (integrale < ARW_MIN )
			integrale = ARW_MIN;

	orientation_pid = Kp*error + integrale + (Kd*(error-old_error))/dt;
	old_error = error;
	//chprintf((BaseSequentialStream *)&SD3, "integral = %d  derivee = %d error=%d ", integrale, derivee, error); //debug
	return orientation_pid;
}

static uint8_t index_highest_sensor_value(void){
	uint16_t max=NULL;
	uint8_t max_sensor_index=NULL;
	for(uint8_t i=0; i<NB_PROX_SENSOR;i++){
		if(sensor_value[i]>max){
			max=sensor_value[i];
			max_sensor_index = i;
		}
	}
	return max_sensor_index;
}


/*************************END INTERNAL FUNCTIONS**********************************/


/****************************PUBLIC FUNCTIONS*************************************/
void distance_start(void){
	proximity_start(); //starts IR sensors
	calibrate_ir(); //calibrates IR sensors

	chThdCreateStatic(distance_thd_wa, sizeof(distance_thd_wa), NORMALPRIO+1, distance_thd, NULL);
}

bool is_there_obstacle(void){
	if(	((sensor_value[FRONT_RIGHT] > SAFE_DISTANCE) ||		//if obstacle on the right and gravity towards the right
		(sensor_value[BACK_RIGHT] > SAFE_DISTANCE) ||		//then there is an obstacle to avoid
		(sensor_value[RIGHT] > SAFE_DISTANCE) ||
		((sensor_value[FRONT_RIGHT]+sensor_value[RIGHT]) > SAFE_DISTANCE) ||
		((sensor_value[RIGHT]+sensor_value[BACK_RIGHT]) > SAFE_DISTANCE)) &&
		((mode_deplacement == MODE_FRONT_RIGHT)||(mode_deplacement == MODE_BACK_RIGHT))){
		//chprintf((BaseSequentialStream *)&SD3, " obstacle "); //debug
		return true;

	}else if(((sensor_value[FRONT_LEFT] > SAFE_DISTANCE) ||	//if obstacle on the left and gravity towards the left
			(sensor_value[BACK_LEFT] > SAFE_DISTANCE) ||	//then there is an obstacle to avoid
			(sensor_value[LEFT] > SAFE_DISTANCE) ||
			((sensor_value[BACK_LEFT]+sensor_value[LEFT]) > SAFE_DISTANCE) ||
			((sensor_value[LEFT]+sensor_value[FRONT_LEFT]) > SAFE_DISTANCE)) &&
			((mode_deplacement==MODE_FRONT_LEFT)||(mode_deplacement==MODE_BACK_LEFT))){
		//chprintf((BaseSequentialStream *)&SD3, " obstacle "); //debug
		return true;

	}else if(((sensor_value[FRONT_RIGHT]+sensor_value[FRONT_LEFT]) > SAFE_DISTANCE) ||   //if obstacle in front or behind
			((sensor_value[BACK_RIGHT]+sensor_value[BACK_LEFT]) > SAFE_DISTANCE)){		//then there is an obstacle to avoid
		return true;
	}

	return false; //otherwise there is no obstacle to avoid
}

void avoid_obstacle(int speed){
	uint8_t index_sensor_max = index_highest_sensor_value();
	static int16_t error_orientation = NULL;
	static int16_t error_distance_to_wall = NULL;
	static int8_t previous_state = NULL;

	switch(index_sensor_max){
		case FRONT_RIGHT :
		case BACK_RIGHT :
		case RIGHT :
			if((previous_state == FRONT_LEFT) || (previous_state == BACK_LEFT) || (previous_state == LEFT)){
				integrale = NULL; //s'il y a un changement d'état, on remet l'integrale de l'erreur à zero
			}
			error_distance_to_wall = SAFE_DISTANCE - sensor_value[RIGHT];
			error_orientation=sensor_value[FRONT_RIGHT]-sensor_value[BACK_RIGHT];

			new_speed[0]=speed+regulator_orientation(error_orientation)-error_distance_to_wall;
			new_speed[1]=speed-regulator_orientation(error_orientation)+error_distance_to_wall;
			break;
		case FRONT_LEFT :
		case BACK_LEFT :
		case LEFT :
			if((previous_state == FRONT_RIGHT) || (previous_state == BACK_RIGHT) || (previous_state == RIGHT)){
				integrale = NULL; //s'il y a un changement d'état, on remet l'integrale de l'erreur à zero
			}
			error_distance_to_wall = SAFE_DISTANCE - sensor_value[LEFT];
			error_orientation=sensor_value[FRONT_LEFT]-sensor_value[BACK_LEFT];

			new_speed[0]=speed-regulator_orientation(error_orientation)+error_distance_to_wall;
			new_speed[1]=speed+regulator_orientation(error_orientation)-error_distance_to_wall;
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


