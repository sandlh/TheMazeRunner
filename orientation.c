#include "orientation.h"

#include <msgbus/messagebus.h>
#include <sensors/imu.h>
#include <stdbool.h>
#include <stdlib.h>


#include <math.h>

#define THREAD_PERIOD	4//[ms]
#define THRESHOLD_X 0.5
#define THRESHOLD_Y 0.5
#define PI 3.14
#define CUTOFFREQUENCY 0.10
#define TAU 1/(2*PI*CUTOFFREQUENCY)
#define ALPHA THREAD_PERIOD/(TAU +THREAD_PERIOD)
//#define BETA
//#define GAMMA

static float error = 0;
static pente = 0;


messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);

static void update_data(void);
static float passe_bas_filter(float acc);
float get_error(void);
void orientation_start(void);
float get_pente(void);

static THD_WORKING_AREA(imu_reader_thd_wa, 512);
static THD_FUNCTION(imu_reader_thd, arg) {

     (void) arg;
     chRegSetThreadName(__FUNCTION__);

     messagebus_topic_t *imu_topic = messagebus_find_topic_blocking(&bus, "/imu");// à comprendre
     imu_msg_t imu_values;

	 systime_t time;


	while(1){
		time = chVTGetSystemTime();

		messagebus_topic_wait(imu_topic, &imu_values, sizeof(imu_values));// à comprendre

		update_data();

		chThdSleepUntilWindowed(time, time + MS2ST(THREAD_PERIOD));

	}
}

void orientation_start(void)
{
	// setup IMU
	messagebus_init(&bus, &bus_lock, &bus_condvar);
	imu_start();
	calibrate_acc();

	chThdCreateStatic(imu_reader_thd_wa, sizeof(imu_reader_thd_wa), NORMALPRIO+1, imu_reader_thd, NULL);
}
static void update_data(void) // quand je veu tourner mon acceleration en x est mon erreur dans un deuxième temps y pour la vitesse
{
	float acceleration_x = get_acc(X_AXIS); // il faudrait créer notre propre filtre
	float acceleration_y = get_acc(Y_AXIS);

	acceleration_x = passe_bas_filter(acceleration_x);
	acceleration_y = passe_bas_filter(acceleration_y);
	static float threshold=150	;

	if(abs(acceleration_y)<threshold){
		error = 0;
		//pente = acceleration_x;

		}else if (acceleration_x >0){
			error = acceleration_y;
		}else {
			error = -acceleration_y;
		}

}

static float passe_bas_filter(float acc){


	static float acc_filtered_old_value = 0;
	float acc_low_pass_filtered = 0;


	acc_low_pass_filtered = ALPHA * acc +(1-ALPHA)*acc_filtered_old_value;
	acc_filtered_old_value = acc_low_pass_filtered;
	return acc_low_pass_filtered;
}
/*
static float second_low_pass_filter(float acc){
	static float acc_filtered_old_value = 0;
	static float acc_filtered_older_value = 0;
	static float acc_old_value = 0;
	static float acc_older_value = 0;
	float acc_low_pass_filtered = 0;


	acc_low_pass_filtered = ALPHA * acc +(1-ALPHA)*acc_filtered_old_value;   à changer pour le filtre de second ordre

	acc_filtered_older_value = acc_filtered_old_value;
	acc_filtered_old_value = acc_low_pass_filtered;

	acc_older_value = acc_old_value;
	acc_old_value = acc;

	return acc_low_pass_filtered;
}*/

float get_error(void)
{
	return error;
}

float get_pente(void)
{
	return pente;
}

