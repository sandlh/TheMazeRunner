#include "orientation.h"

#include <msgbus/messagebus.h>
#include <sensors/imu.h>
#include <stdbool.h>
#include <stdlib.h>
#include <chprintf.h>
#include <usbcfg.h>

#include <math.h>



#define PI 3.14
#define CUTOFFREQUENCY 0.0015
#define TAU 1/(2*PI*CUTOFFREQUENCY)
#define ALPHA ORIENTATION_THREAD_PERIOD/(TAU +ORIENTATION_THREAD_PERIOD)


static int16_t error = 0;
static int16_t norme =0;
static int8_t mode_deplacement = 0;
static int16_t acc_y=0;


messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);

void orientation_start(void);

static void update_data(void);
static int16_t passe_bas_filter(int16_t acc);

int16_t get_error(void);
int16_t get_norme(void);
int8_t get_mode_deplacement(void);


static THD_WORKING_AREA(imu_reader_thd_wa, 512);
static THD_FUNCTION(imu_reader_thd, arg) {

     (void) arg;
     chRegSetThreadName(__FUNCTION__);
     messagebus_topic_t *imu_topic = messagebus_find_topic_blocking(&bus, "/imu");// � comprendre
     imu_msg_t imu_values;
	 systime_t time;

	while(1){

		time = chVTGetSystemTime();
		messagebus_topic_wait(imu_topic, &imu_values, sizeof(imu_values));// � comprendre
		update_data();
		chThdSleepUntilWindowed(time, time + MS2ST(ORIENTATION_THREAD_PERIOD));

	}
}

static void update_data(void) // chang�
{
	int16_t acceleration_x = (int16_t)get_acc(X_AXIS); // il faudrait cr�er notre propre filtre
	int16_t acceleration_y = (int16_t)get_acc(Y_AXIS);


	acceleration_x = passe_bas_filter(acceleration_x);
	acceleration_y = passe_bas_filter(acceleration_y);

	chprintf((BaseSequentialStream *)&SD3, "acc_x = %d \n", acceleration_x); //prints
	chprintf((BaseSequentialStream *)&SD3, "acc_y = %d \n", acceleration_y); //prints


	norme = sqrt(acceleration_x*acceleration_x + acceleration_y*acceleration_y);
	//chprintf((BaseSequentialStream *)&SD3, "norme = %d \n", norme); //prints
	error = acceleration_x;
	acc_y = acceleration_y;
	//chprintf((BaseSequentialStream *)&SD3, "error = %d \n", error);

	if ((acceleration_x*acceleration_y >= 0) && (acceleration_y >=0)){
		mode_deplacement = MODE_BACK_LEFT ;   // roue ext = roue gauche
	}else if ((acceleration_x*acceleration_y >= 0) && (acceleration_y < 0)){
		mode_deplacement = MODE_FRONT_RIGHT;
	}else if ((acceleration_x*acceleration_y < 0) && (acceleration_y >= 0)){
		mode_deplacement = MODE_BACK_RIGHT;
	}else {
		mode_deplacement = MODE_FRONT_LEFT;
	}
	//chprintf((BaseSequentialStream *)&SD3, "mode = %d \n", mode_deplacement);

}

static int16_t passe_bas_filter(int16_t acc){


	static int16_t acc_filtered_old_value = 0;
	static int16_t acc_filtered_older_value = 0;
	int16_t acc_low_pass_filtered = 0;


	acc_low_pass_filtered = ALPHA * acc +(1-ALPHA)*acc_filtered_older_value;
	acc_filtered_older_value = acc_filtered_old_value;
	acc_filtered_old_value = acc_low_pass_filtered;

	return acc_low_pass_filtered;
}

//-----------------------Fonctions non statiques ------------------------------------------------------------------------------

void orientation_start(void)
{
	// setup IMU
	messagebus_init(&bus, &bus_lock, &bus_condvar);
	imu_start();
	calibrate_acc();
	chThdCreateStatic(imu_reader_thd_wa, sizeof(imu_reader_thd_wa), NORMALPRIO, imu_reader_thd, NULL);
}

//-------- fonctions get---------

int16_t get_error(void)
{
	return error;
}

int16_t get_norme(void)
{
	return norme;
}

int8_t get_mode_deplacement(void)
{
	return mode_deplacement;
}

float get_cos_gravity(void){
	return ((float)error/(float)norme);
}

float get_sin_gravity(void){
	return ((float)acc_y/(float)norme);
}


