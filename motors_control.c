#include "motors_control.h"
#include "orientation.h"
#include "distance.h"


#include <msgbus/messagebus.h>

#include <motors.h>


#include <stdbool.h>
#include <stdlib.h>
#include <chprintf.h>
#include <usbcfg.h>


#include <math.h>


#define KD_SPEED 0.15
#define KI_SPEED 3
#define KP_SPEED 1.5
#define THREAD_TIME 4 //[ms]
#define AWM_MAX  100
#define AWM_MIN  -AWM_MAX

#define NORME_MAX 5000
#define NORME_MIN 500

#define ACC_MAX 10000

#define MODE_FRONT_LEFT 0
#define MODE_FRONT_RIGHT 1
#define MODE_BACK_RIGHT 2
#define MODE_BACK_LEFT 3

#define SPEED_COEFF 5
#define SPEED_MAX 1000
#define SPEED_MIN 0

#define FRONT_RIGHT 0 //#define to use the values from the sensor in the array sensor_value
#define FRONT_LEFT 1
#define BACK_RIGHT 2
#define BACK_LEFT 3
#define RIGHT 4
#define LEFT 5

#define ARW_MAX 50 //anti rewind max and min values for orientation PID
#define ARW_MIN -50

#define SPEED_BIAS 400

#define KP_ORIENTATION 4
#define KI_ORIENTATION 1
#define KD_ORIENTATION 20

#define ZERO 0
#define SPEED_ZERO 0

static void set_motors_speed(int16_t speed_ext, float speed_int);
static float regulator_speed(int16_t error);

static int16_t speed_proportionelle(int16_t norme);
static void calculate_speeds(int16_t speed_prop);

static int16_t speed_ext = 0;
static int16_t speed_int = 0;
static int8_t integrale_pid_orientation = 0;
static int16_t regulator_orientation(int16_t error);
static void avoid_obstacle(int16_t speed_prop);

/***************************INTERNAL FUNCTIONS************************************/
static THD_WORKING_AREA(motor_control_thd_wa, 512);
static THD_FUNCTION(motor_control_thd, arg)
{
	(void) arg;
	     chRegSetThreadName(__FUNCTION__);

	     int16_t speed_prop = 0;

	     systime_t time;

	     while(true)
	     {
	    	time = chVTGetSystemTime();

	    	speed_prop = speed_proportionelle(get_norme());

	    	if(is_there_obstacle()){
		    	avoid_obstacle(speed_prop);
	    	}else{
	    	integrale_pid_orientation = 0;
			calculate_speeds(speed_prop);
	    	set_motors_speed(speed_ext, speed_int);
	    	}

			chThdSleepUntilWindowed(time, time + MS2ST(THREAD_TIME));

     }
}

static int16_t speed_proportionelle(int16_t norme)
{
	int16_t speed = 0;
	if (norme < NORME_MIN ){
		speed = SPEED_MIN;
	} else if (speed > NORME_MAX){
		speed = SPEED_MAX;
	}else {
		speed = norme/SPEED_COEFF;
	}
	return speed;
}

static void calculate_speeds(int16_t speed_prop)  // enlever les valeurs numï¿½riques

{
	int16_t error = fabs(get_error());
	float pid = regulator_speed(error);

	if (pid > ACC_MAX){
		pid = ACC_MAX;
	}
	float speed_correct =  pid/ACC_MAX; // verif valeur de ACC_MAX

	speed_ext = speed_prop * (1+speed_correct);
	speed_int = speed_prop * (1-speed_correct);

}

static float regulator_speed(int16_t error)  //voir si je peux le mettre en int
{
	//PID
		uint8_t dt = 4;
		static int16_t ancienne_erreur = 0;
		float speed_pid =0;

		float new_integrale = 0;
		static float integrale = 0;

		new_integrale = KI_SPEED * error*dt;
		integrale += new_integrale;

		if(integrale > AWM_MAX)
				integrale = AWM_MAX;
		else if (integrale < AWM_MIN )
				integrale = AWM_MIN;


		speed_pid = KP_SPEED*error + integrale + KD_SPEED*(error-ancienne_erreur)/dt;
		ancienne_erreur = error;

		return speed_pid;
	}

static void set_motors_speed(int16_t speed_ext, float speed_int)
{
	switch (get_mode_deplacement()){
			 case MODE_FRONT_RIGHT:
				 right_motor_set_speed(speed_int);
				 left_motor_set_speed(speed_ext);
				 break;
			 case MODE_BACK_RIGHT:
				 right_motor_set_speed(-speed_int);
				 left_motor_set_speed(-speed_ext);
				 break;
			 case MODE_BACK_LEFT:
				 right_motor_set_speed(-speed_ext);
				 left_motor_set_speed(-speed_int);
				 break;
			 case MODE_FRONT_LEFT:
				 right_motor_set_speed(speed_ext);
				 left_motor_set_speed(speed_int);
				 break;
		 }
}
/*************************FUNCTIONS FOR OBSTACLE CONTROL**********************************/
static int16_t regulator_orientation(int16_t error)
{
	//PID
	uint8_t dt = THREAD_TIME; //est-ce qu'on a besoin de le déclaré comme une valeur ? on peut pas juste utilisé le define ?

	static int16_t old_error = ZERO;
	int16_t orientation_pid =ZERO;

	integrale_pid_orientation += KI_ORIENTATION *error*dt;

	if(integrale_pid_orientation > ARW_MAX){
			integrale_pid_orientation = ARW_MAX;
	}else if(integrale_pid_orientation < ARW_MIN ){
			integrale_pid_orientation = ARW_MIN;
	}

	orientation_pid = KP_ORIENTATION*error + integrale_pid_orientation + (KD_ORIENTATION*(error-old_error))/dt;
	old_error = error;
	return orientation_pid;
}

static void avoid_obstacle(int16_t speed_prop){
	uint8_t index_sensor_max = index_highest_sensor_value();
	static int16_t error_orientation = ZERO;
	static int16_t error_distance_to_wall = ZERO;

	int8_t mode_deplacement = get_mode_deplacement();

	switch(index_sensor_max){
		case FRONT_RIGHT :
		case BACK_RIGHT :
		case RIGHT :
			error_distance_to_wall = get_error_distance_to_wall_right();
			error_orientation=get_error_orientation_right();
			break;
		case FRONT_LEFT :
		case BACK_LEFT :
		case LEFT :
			error_distance_to_wall = get_error_distance_to_wall_left();
			error_orientation=get_error_orientation_left();
			break;
		default :
			break;
	}

	int16_t pid_orientation=regulator_orientation(error_orientation);

	if((mode_deplacement==FRONT_RIGHT) || (mode_deplacement==FRONT_LEFT)){
		speed_ext=SPEED_BIAS+pid_orientation-error_distance_to_wall;
		speed_int=SPEED_BIAS-pid_orientation+error_distance_to_wall;
	}else{
		speed_ext=-SPEED_BIAS+pid_orientation+error_distance_to_wall;
		speed_int=-SPEED_BIAS-pid_orientation-error_distance_to_wall;
	}
	if(speed_prop==SPEED_ZERO){
		speed_ext=SPEED_ZERO;
		speed_int=SPEED_ZERO;
	}

	if((index_sensor_max==FRONT_RIGHT)||(index_sensor_max==RIGHT)||(index_sensor_max==BACK_RIGHT)){	//if wall on the right, right wheel=exterior wheel
		 right_motor_set_speed(speed_ext);															//and left wheel=interior wheel
		 left_motor_set_speed(speed_int);
	}else{									//otherwise, wall on the left -> right wheel=
		 right_motor_set_speed(speed_int);
		 left_motor_set_speed(speed_ext);
	}
}
/***********************END FUNCITONS FOR OBSTACLE CONTROL********************************/

/*************************END INTERNAL FUNCTIONS**********************************/


/****************************PUBLIC FUNCTIONS*************************************/
void motors_control_start(void)
{
	motors_init();
	orientation_start();

	chThdCreateStatic(motor_control_thd_wa, sizeof(motor_control_thd_wa), NORMALPRIO+1, motor_control_thd, NULL);
}
/****************************END PUBLIC FUNCTIONS*************************************/


