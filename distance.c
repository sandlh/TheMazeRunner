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


#define FRONT_RIGHT 0
#define BACK_RIGHT 3
#define BACK_LEFT 4
#define FRONT_LEFT 7
#define RIGHT 2
#define LEFT 5

#define THREAD_PERIOD 4 //[ms]

static uint16_t distance_front_right=0;
static uint16_t distance_front_left =0;
static uint16_t distance_back_right = 0;
static uint16_t distance_back_left =0;
static uint16_t distance_right = 0;
static uint16_t distance_left =0;
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

		chprintf((BaseSequentialStream *)&SD3, "dist_f_l = %d   dist_f_r = %d  dist_l= %d  \n", distance_front_left, distance_front_right, distance_left);

		chThdSleepUntilWindowed(time, time + MS2ST(THREAD_PERIOD));

	}
}


static void update_data(void){
	distance_front_right = get_calibrated_prox(FRONT_RIGHT);
	distance_front_left = get_calibrated_prox(FRONT_LEFT);
	distance_back_right = get_calibrated_prox(BACK_RIGHT);
	distance_back_left = get_calibrated_prox(BACK_LEFT);
	distance_right = get_calibrated_prox(RIGHT);
	distance_left = get_calibrated_prox(LEFT);
}

/*************************END INTERNAL FUNCTIONS**********************************/


/****************************PUBLIC FUNCTIONS*************************************/
void distance_start(void){
	proximity_start(); //starts IR sensors
	calibrate_ir(); //calibrates IR sensors

	chThdCreateStatic(distance_thd_wa, sizeof(distance_thd_wa), NORMALPRIO+1, distance_thd, NULL);
}

uint16_t get_distance_front_right(void){
	return distance_front_right;
}

uint16_t get_distance_front_left(void){
	return distance_front_left;
}

uint16_t get_distance_back_right(void){
	return distance_back_right;
}

uint16_t get_distance_back_left(void){
	return distance_back_left;
}

uint16_t get_distance_right(void){
	return distance_right;
}

uint16_t get_distance_left(void){
	return distance_left;
}


/**************************END PUBLIC FUNCTIONS***********************************/


