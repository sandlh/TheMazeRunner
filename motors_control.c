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

#define ZERO 0

static void set_motors_speed(int16_t speed_ext, float speed_int);
static float regulator_speed(int16_t error);

static int16_t speed_proportionelle(int16_t norme);
static void calculate_speeds(int16_t speed_prop);

static void set_motors_speed_obstacle(int16_t speed_ext, int16_t speed_int);

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
	    	//chprintf((BaseSequentialStream *)&SD3, "speed_prop = %d \n", speed_prop); //prints
	    	if(is_there_obstacle()){
		    	avoid_obstacle(speed_prop);
		    	//chprintf((BaseSequentialStream *)&SD3, " speed_int=%d speed ext=%d ", speed_int, speed_ext); //prints
		    	//set_motors_speed_obstacle(speed_ext, speed_int);
	    	}else{
	    	integrale_pid_orientation = 0;
			calculate_speeds(speed_prop);
	    	set_motors_speed(speed_ext, speed_int);
	    	//chprintf((BaseSequentialStream *)&SD3, " I"); //prints
			// right_motor_set_speed(400);
			// left_motor_set_speed(400);
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

static void calculate_speeds(int16_t speed_prop)  // enlever les valeurs num�riques

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

		//chprintf((BaseSequentialStream *)&SD3, "error = %d \n", error); //prints

		speed_pid = KP_SPEED*error + integrale + KD_SPEED*(error-ancienne_erreur)/dt;
		ancienne_erreur = error;
		//chprintf((BaseSequentialStream *)&SD3, "error2 = %d \n", ancienne_erreur); //prints

    	//chprintf((BaseSequentialStream *)&SD3, "speed_PID = %f \n", speed_pid); //prints
		return speed_pid;
	}

static void set_motors_speed(int16_t speed_ext, float speed_int) // modifi�
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
	uint8_t dt = THREAD_TIME;
	static uint8_t Kp = 4;
	static float Ki = 1;
	static uint8_t Kd = 20;

	static int16_t old_error = ZERO;
	int16_t orientation_pid =ZERO;

	static int16_t derivee = ZERO;//debug

	integrale_pid_orientation += Ki *error*dt;
	derivee = error-old_error;//debug

	if(integrale_pid_orientation > ARW_MAX){
			integrale_pid_orientation = ARW_MAX;
	}else if(integrale_pid_orientation < ARW_MIN ){
			integrale_pid_orientation = ARW_MIN;
	}

	orientation_pid = Kp*error + integrale_pid_orientation + (Kd*(error-old_error))/dt;
	old_error = error;
	chprintf((BaseSequentialStream *)&SD3, "  orientation_pid = %d   ", orientation_pid); //debug
	return orientation_pid;
}

static void avoid_obstacle(int16_t speed_prop){
	uint8_t index_sensor_max = index_highest_sensor_value();
	static int16_t error_orientation = ZERO;
	static int16_t error_distance_to_wall = ZERO;

	int8_t mode_deplacement = get_mode_deplacement();

	//int16_t error = get_error();
	//int16_t norme = get_norme();
	float cos_gravity = fabs(get_cos_gravity());
	//chprintf((BaseSequentialStream *)&SD3, "  cos = %f error=%d  norme=%d", cos_gravity, error, norme); //debug

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
	speed_ext=pid_orientation;
	speed_int=-pid_orientation;

	if((mode_deplacement==FRONT_RIGHT) || (mode_deplacement==FRONT_LEFT)){
		speed_ext=400+pid_orientation-error_distance_to_wall;
		speed_int=400-pid_orientation+error_distance_to_wall;
		//chprintf((BaseSequentialStream *)&SD3, "  dis_to_wall=%d   pid=%d   ", error_distance_to_wall,pid_orientation); //debug
	}else{
		speed_ext=-400+pid_orientation+error_distance_to_wall;
		speed_int=-400-pid_orientation-error_distance_to_wall;
	}
	if(speed_prop==0){
		speed_ext=0;
		speed_int=0;
	}
	/*if(((fabs(cos_gravity)<0.25)&&((index_sensor_max==FRONT_RIGHT)||(index_sensor_max==FRONT_LEFT)))&&
			((mode_deplacement==FRONT_RIGHT)||(mode_deplacement==FRONT_LEFT))){
		speed_int= 0;
		speed_ext=0;
		//chprintf((BaseSequentialStream *)&SD3, "  cos = %f ", cos_gravity); //debug
}else if(((fabs(cos_gravity)<0.25)&&((index_sensor_max==BACK_RIGHT)||(index_sensor_max==BACK_LEFT)))&&
			((mode_deplacement==BACK_RIGHT)||(mode_deplacement==BACK_LEFT))){
	speed_int= 0;
	speed_ext=0;
}*/

	if((index_sensor_max==FRONT_RIGHT)||(index_sensor_max==RIGHT)||(index_sensor_max==BACK_RIGHT)){
		 right_motor_set_speed(speed_ext);
		 left_motor_set_speed(speed_int);
	}else{
		 right_motor_set_speed(speed_int);
		 left_motor_set_speed(speed_ext);
	}
}

static void set_motors_speed_obstacle(int16_t speed_ext, int16_t speed_int) // modifi�
{
	switch (get_mode_deplacement()){
			 case MODE_FRONT_LEFT:
			 case MODE_BACK_LEFT:
				 right_motor_set_speed(speed_int);
				 left_motor_set_speed(speed_ext);
				 chprintf((BaseSequentialStream *)&SD3, "LEFT");
				 break;
			 case MODE_BACK_RIGHT:
			 case MODE_FRONT_RIGHT:
				 right_motor_set_speed(speed_ext);
				 left_motor_set_speed(speed_int);
				 chprintf((BaseSequentialStream *)&SD3, "RIGHT ");
				 break;
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



