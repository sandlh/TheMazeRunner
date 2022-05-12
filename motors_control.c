#include "motors_control.h"
#include "orientation.h"


#include <msgbus/messagebus.h>

#include <motors.h>


#include <stdbool.h>
#include <stdlib.h>
#include <chprintf.h>
#include <usbcfg.h>


#include <math.h>


#define KD_SPEED 15  // 0.0015
#define KI_SPEED 30   //0.003
#define KP_SPEED 10  //0.01
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

#define MAX_COEFF 1
#define ROTATION_COEFF 200

#define FRONT_RIGHT 0 //#define to use the values from the sensor in the array sensor_value
#define FRONT_LEFT 1
#define BACK_RIGHT 2
#define BACK_LEFT 3
#define RIGHT 4
#define LEFT 5

#define ARW_MAX 50 //anti rewind max and min values for orientation PID
#define ARW_MIN -50

#define NULL 0

static void set_motors_speed(int16_t speed_ext, float speed_int, int8_t mode_deplacement);
static float regulator_speed(int16_t error);

static int16_t speed_proportionelle(int16_t norme);
static float coeff_int(float pid);
void motors_control_start(void);

static int8_t integrale=0;
static int new_speed[2];
static int16_t regulator_orientation(int16_t error);
static void avoid_obstacle(void);

/***************************INTERNAL FUNCTIONS************************************/
static THD_WORKING_AREA(motor_control_thd_wa, 512);
static THD_FUNCTION(motor_control_thd, arg)
{
     (void) arg;
     chRegSetThreadName(__FUNCTION__);

     int16_t error = 0;
     int16_t norme = 0;
     int8_t mode_deplacement = 0;
     systime_t time;


     while(true)
     {
    	time = chVTGetSystemTime();
    	error = get_error();
		//chprintf((BaseSequentialStream *)&SD3, "error = %d \n", error); //prints

    	norme = get_norme();
    	mode_deplacement = get_mode_deplacement();
    	//chprintf((BaseSequentialStream *)&SD3, "mode = %d \n", mode_deplacement);

    	int16_t speed_ext = 0;
    	int16_t speed_int = 0;
    	int16_t speed_prop = 0;


    	speed_prop = speed_proportionelle(norme);

    	float pid = 0;
    	pid = regulator_speed(error);
    	float speed_correct;

    	speed_correct = coeff_int(pid);
    	//chprintf((BaseSequentialStream *)&SD3, "speed_prop = %d \n", speed_ext); //prints
    	speed_ext = speed_prop + ROTATION_COEFF * speed_correct;
    	speed_int = speed_prop - ROTATION_COEFF * speed_correct;


    	if (speed_ext > SPEED_MAX){
    	    	 		speed_ext = SPEED_MAX;

    	    	}
    	//chprintf((BaseSequentialStream *)&SD3, "speed_int = %f \n", speed_int); //prints

    	set_motors_speed(speed_ext, speed_int, mode_deplacement);

    	/*if(is_there_obstacle()){
    		avoid_obstacle();
    		right_motor_set_speed(new_speed[0]);
    		left_motor_set_speed(new_speed[1]);
    	}else{
    		right_motor_set_speed(400);
    		left_motor_set_speed(400);
    	}*/

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

static float coeff_int(float pid)  // enlever les valeurs numériques
{

	float coeff_int;

	if (pid > ACC_MAX){
		pid = ACC_MAX;
	}
	coeff_int =  pid/ACC_MAX;

	return coeff_int;
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


static void set_motors_speed(int16_t speed_ext, float speed_int, int8_t mode_deplacement) // modifié
{

	switch (mode_deplacement){
			 case MODE_FRONT_LEFT:
				 right_motor_set_speed(speed_int);
				 left_motor_set_speed(speed_ext);
				 break;
			 case MODE_BACK_LEFT:
				 right_motor_set_speed(-speed_int);
				 left_motor_set_speed(-speed_ext);
				 break;
			 case MODE_BACK_RIGHT:
				 right_motor_set_speed(-speed_ext);
				 left_motor_set_speed(-speed_int);
				 break;
			 case MODE_FRONT_RIGHT:
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
	static float Ki = 0.05;
	static uint8_t Kd = 20;

	static int16_t old_error = NULL;
	int16_t orientation_pid =NULL;

	static int16_t derivee = NULL;//debug

	integrale += Ki *error*dt;
	derivee = error-old_error;//debug

	if(integrale > ARW_MAX){
			integrale = ARW_MAX;
	}else if(integrale < ARW_MIN ){
			integrale = ARW_MIN;
	}

	orientation_pid = Kp*error + integrale + (Kd*(error-old_error))/dt;
	old_error = error;
	//chprintf((BaseSequentialStream *)&SD3, "integral = %d  derivee = %d error=%d ", integrale, derivee, error); //debug
	return orientation_pid;
}

static void avoid_obstacle(void){
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
			error_distance_to_wall = get_error_distance_to_wall_right();
			error_orientation=get_error_orientation_right();

			new_speed[0]=400+regulator_orientation(error_orientation)-error_distance_to_wall;
			new_speed[1]=400-regulator_orientation(error_orientation)+error_distance_to_wall;
			break;
		case FRONT_LEFT :
		case BACK_LEFT :
		case LEFT :
			if((previous_state == FRONT_RIGHT) || (previous_state == BACK_RIGHT) || (previous_state == RIGHT)){
				integrale = NULL; //s'il y a un changement d'état, on remet l'integrale de l'erreur à zero
			}
			error_distance_to_wall = get_error_distance_to_wall_left();
			error_orientation=get_error_orientation_left();

			new_speed[0]=400-regulator_orientation(error_orientation)+error_distance_to_wall;
			new_speed[1]=400+regulator_orientation(error_orientation)-error_distance_to_wall;
			break;
		default :
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
/**************************END PUBLIC FUNCTIONS***********************************/
