/*
 * distance.c
 *
 *  Created on: Apr 25, 2022
 *      Author: Barbara De Groot
 */
#include "sensors/proximity.h"
#include "distance.h"
#include "orientation.h"

#include "ch.h"
#include "hal.h"
#include <msgbus/messagebus.h>

#include <math.h>

#define SAFE_DISTANCE 80 //security distance for wall detection
#define DISTANCE_FOLLOW_WALL 120 //distance that the robot is going to keep with the wall while following it
#define THRESHOLD_GRAVITY 0.25f

#define SENSOR_FRONT_RIGHT 0 //#define to read the values from sensors
#define SENSOR_BACK_RIGHT 3
#define SENSOR_BACK_LEFT 4
#define SENSOR_FRONT_LEFT 7
#define SENSOR_RIGHT 2
#define SENSOR_LEFT 5


#define NB_PROX_SENSOR 6// number of proximity sensors used
static uint16_t sensor_value[NB_PROX_SENSOR];

static uint8_t mode_deplacement;

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

		chThdSleepUntilWindowed(time, time + MS2ST(DISTANCE_THREAD_TIME));

	}
}


static void update_data(void){ //reads values from 6 of the IR proximity sensors and updates the mode_deplacement
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

	chThdCreateStatic(distance_thd_wa, sizeof(distance_thd_wa), NORMALPRIO+2, distance_thd, NULL);
}

bool is_there_obstacle(void){ //detects if there is an wall to avoid
	float cos_gravity=get_cos_gravity();

	if(((sensor_value[FRONT_RIGHT] > SAFE_DISTANCE)||	//tests if wall towards front right and if gravity is in the same direction
		(sensor_value[RIGHT] > SAFE_DISTANCE)||
		((sensor_value[FRONT_RIGHT]+sensor_value[RIGHT]) > SAFE_DISTANCE)||
		((sensor_value[FRONT_RIGHT]+sensor_value[FRONT_LEFT])>SAFE_DISTANCE))&&
		(mode_deplacement == MODE_FRONT_RIGHT)){

		return true;

	}else if(((sensor_value[BACK_RIGHT] > SAFE_DISTANCE)||   //tests if wall towards back right and if gravity is in the same direction
			(sensor_value[RIGHT] > SAFE_DISTANCE)||
			((sensor_value[BACK_RIGHT]+sensor_value[RIGHT]) > SAFE_DISTANCE)||
			((sensor_value[BACK_RIGHT]+sensor_value[BACK_LEFT])>SAFE_DISTANCE))&&
			(mode_deplacement == MODE_BACK_RIGHT)){

			return true;

	}else if(((sensor_value[FRONT_LEFT] > SAFE_DISTANCE)||  //tests if wall towards front left and if gravity is in the same direction
			(sensor_value[LEFT] > SAFE_DISTANCE)||
			((sensor_value[FRONT_LEFT]+sensor_value[LEFT]) > SAFE_DISTANCE)||
			((sensor_value[FRONT_RIGHT]+sensor_value[FRONT_LEFT])>SAFE_DISTANCE))&&
			(mode_deplacement == MODE_FRONT_LEFT)){

			return true;

	}else if(((sensor_value[BACK_LEFT] > SAFE_DISTANCE)||	//tests if wall towards back left and if gravity is in the same direction
			(sensor_value[LEFT] > SAFE_DISTANCE)||
			((sensor_value[BACK_LEFT]+sensor_value[LEFT]) > SAFE_DISTANCE)||
			((sensor_value[BACK_RIGHT]+sensor_value[BACK_LEFT])>SAFE_DISTANCE))&&
			(mode_deplacement == MODE_BACK_LEFT)){

			return true;

	}else if((((sensor_value[FRONT_RIGHT])>SAFE_DISTANCE)|| //if gravity is a bit towards the left, the robot continues a bit to follow the wall on the right
			(sensor_value[RIGHT]>SAFE_DISTANCE)||			//so that when the change from wall following to slope following occurs, the robot doesn't
			(sensor_value[BACK_RIGHT]>SAFE_DISTANCE)||		//make a curve towards the wall
			((sensor_value[BACK_RIGHT]+sensor_value[RIGHT]) > SAFE_DISTANCE)||
			((sensor_value[FRONT_RIGHT]+sensor_value[RIGHT]) > SAFE_DISTANCE))&&
			(cos_gravity<THRESHOLD_GRAVITY)){
		return true;
	}else if((((sensor_value[FRONT_LEFT])>SAFE_DISTANCE)|| 	//if gravity is a bit towards the right, the robot continues a bit to follow the wall on the left
			(sensor_value[LEFT]>SAFE_DISTANCE)||			//so that when the change from wall following to slope following occurs, the robot doesn't
			(sensor_value[BACK_LEFT]>SAFE_DISTANCE)||		//make a curve towards the wall
			((sensor_value[BACK_LEFT]+sensor_value[LEFT]) > SAFE_DISTANCE)||
			((sensor_value[FRONT_LEFT]+sensor_value[LEFT]) > SAFE_DISTANCE))&&
			(cos_gravity>(-THRESHOLD_GRAVITY))){
		return true;
	}


	return false; //otherwise there is no wall to avoid
}

uint8_t index_highest_sensor_value(void){ //finds where the wall is by finding the sensor with the maximum value
	uint16_t max=ZERO;
	uint8_t max_sensor_index=ZERO;
	for(uint8_t i=ZERO; i<NB_PROX_SENSOR;i++){
		if(sensor_value[i]>max){
			max=sensor_value[i];
			max_sensor_index = i;
		}
	}
	return max_sensor_index;
}

int16_t get_error_distance_to_wall_right(void){
	return (DISTANCE_FOLLOW_WALL - sensor_value[RIGHT]);
}

int16_t get_error_distance_to_wall_left(void){
	return (DISTANCE_FOLLOW_WALL - sensor_value[LEFT]);
}

int16_t get_error_orientation_wall_right(void){
	return (sensor_value[FRONT_RIGHT]-sensor_value[BACK_RIGHT]);
}

int16_t get_error_orientation_wall_left(void){
	return (sensor_value[FRONT_LEFT]-sensor_value[BACK_LEFT]);
}

/**************************END PUBLIC FUNCTIONS***********************************/


