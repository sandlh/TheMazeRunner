#include "orientation.h"

#include <msgbus/messagebus.h>
#include <sensors/imu.h>
#include <stdbool.h>
#include <stdlib.h>
#include <chprintf.h>
#include <usbcfg.h>

#include <math.h>

// Define for low-pass filter values
#define PI 3.14
#define CUTOFFREQUENCY 0.0015
#define TAU 1/(2*PI*CUTOFFREQUENCY)
#define ALPHA ORIENTATION_THREAD_PERIOD/(TAU +ORIENTATION_THREAD_PERIOD)

static int16_t error = 0;
static int16_t norme = 0;
static int8_t mode_deplacement = 0;

messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);


void orientation_start(void);

static void update_data(void);
static int16_t passe_bas_filter(int16_t acc);

int16_t get_error(void);
int16_t get_norme(void);
int8_t get_mode_deplacement(void);

//-----------------------Internal functions ------------------------------------------------------------------------------

//Thread definition
static THD_WORKING_AREA(imu_reader_thd_wa, 512);
static THD_FUNCTION(imu_reader_thd, arg) {

     (void) arg;
     chRegSetThreadName(__FUNCTION__);
     messagebus_topic_t *imu_topic = messagebus_find_topic_blocking(&bus, "/imu");
     imu_msg_t imu_values;
	 systime_t time;

	while(1){

		time = chVTGetSystemTime();
		messagebus_topic_wait(imu_topic, &imu_values, sizeof(imu_values));

		update_data();

		chThdSleepUntilWindowed(time, time + MS2ST(ORIENTATION_THREAD_PERIOD));

	}
}

static void update_data(void) //
{
	int16_t acceleration_x = (int16_t)get_acc(X_AXIS);
	int16_t acceleration_y = (int16_t)get_acc(Y_AXIS);

	acceleration_x = passe_bas_filter(acceleration_x);
	acceleration_y = passe_bas_filter(acceleration_y);

	norme = sqrt(acceleration_x*acceleration_x + acceleration_y*acceleration_y);

	// if we want the e-puck to be in the same diirection as the slope, acceleration_x should be zero
	// instead of taking the angle as an error we took the acceleration_x

	error = acceleration_x;

	if ((acceleration_x*acceleration_y >= 0) && (acceleration_y >=0)){
		mode_deplacement = MODE_BACK_LEFT ;   // ext_wheel = right wheel , int_wheel = left wheel
	}else if ((acceleration_x*acceleration_y >= 0) && (acceleration_y < 0)){
		mode_deplacement = MODE_FRONT_RIGHT;
	}else if ((acceleration_x*acceleration_y < 0) && (acceleration_y >= 0)){
		mode_deplacement = MODE_BACK_RIGHT;
	}else {
		mode_deplacement = MODE_FRONT_LEFT;
	}

}

static int16_t passe_bas_filter(int16_t acc){

    // low pass filter

	static int16_t acc_filtered_old_value = 0;
	//we always call this function for acceleration_x and acceleration_y one after the other
	//so we take the older value for the last value of the acceleration currently being filtered
	static int16_t acc_filtered_older_value = 0;

	//equation of the low pass filter, read the rpport for more information
	int16_t acc_low_pass_filtered = ALPHA * acc +(1-ALPHA)*acc_filtered_older_value;

	//update the static variable of previous values
	acc_filtered_older_value = acc_filtered_old_value;
	acc_filtered_old_value = acc_low_pass_filtered;

	return acc_low_pass_filtered;
}

//-----------------------PUBLIC FUNCTIONS ------------------------------------------------------------------------------

void orientation_start(void)
{
	// setup IMU
	messagebus_init(&bus, &bus_lock, &bus_condvar);
	imu_start();
	calibrate_acc();
	chThdCreateStatic(imu_reader_thd_wa, sizeof(imu_reader_thd_wa), NORMALPRIO, imu_reader_thd, NULL);
}

// get functions : used to get those static variables to other modules
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


