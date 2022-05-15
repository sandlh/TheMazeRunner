#include "motors_control.h"
#include "orientation.h"
#include "distance.h"

#include <msgbus/messagebus.h>
#include <motors.h>
#include <stdlib.h>

#include <math.h>

//define for the regulator_speed
#define KD_SPEED 0.15
#define KI_SPEED 3
#define KP_SPEED 1.5
#define MOTOR_THREAD_TIME 2 //[ms]
#define AWM_MAX  100
#define AWM_MIN  -AWM_MAX

#define AWU_MAX  100   //anti wind up for the regulator speed
#define AWU_MIN  -AWU_MAX

#define NORME_MAX 5500
#define NORME_MIN 750

#define ACC_MAX 10000

#define SPEED_COEFF 5
#define SPEED_MAX 1000
#define SPEED_MIN 0

#define ARW_MAX 50 //anti rewind max and min values for wall following PID
#define ARW_MIN -50

#define SPEED_BIAS 400

#define KP_ORIENTATION 4
#define KI_ORIENTATION 1
#define KD_ORIENTATION 20



static void set_motors_speed(int16_t speed_ext, float speed_int);
static float regulator_speed(void);

static int16_t speed_proportionelle(int16_t norme);
static void calculate_speeds(int16_t speed_prop);

static int16_t speed_ext = 0;
static int16_t speed_int = 0;
static int8_t integrale_pid_wall_following = 0;
static int16_t regulator_wall_following(int16_t error);
static void follow_wall(int16_t speed_prop);

/***************************INTERNAL FUNCTIONS************************************/
static THD_WORKING_AREA(motor_control_thd_wa, 512);
static THD_FUNCTION(motor_control_thd, arg)
{
	(void) arg;
	     chRegSetThreadName(__FUNCTION__);

	     int16_t speed_prop = ZERO;

	     systime_t time;

	     while(true)
	     {
	    	time = chVTGetSystemTime();

	    	speed_prop = speed_proportionelle(get_norme());

	    	if(is_there_obstacle()){
		    	follow_wall(speed_prop);
	    	}else{
	    	integrale_pid_wall_following = ZERO;
			calculate_speeds(speed_prop);
	    	set_motors_speed(speed_ext, speed_int);
	    	}

			chThdSleepUntilWindowed(time, time + MS2ST(MOTOR_THREAD_TIME));

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

static void calculate_speeds(int16_t speed_prop)

{

	float pid = regulator_speed();

	if (pid > ACC_MAX){
		pid = ACC_MAX;
	}
	float speed_correct =  pid/ACC_MAX;

	speed_ext = speed_prop * (1+speed_correct);
	speed_int = speed_prop * (1-speed_correct);

}

static float regulator_speed(void)
{
	//PID
		static int16_t ancienne_erreur = 0;
		float speed_pid =0;

		float new_integrale = 0;
		static float integrale = 0;

		int16_t error = abs(get_error());
		new_integrale = KI_SPEED * error*ORIENTATION_THREAD_PERIOD;
		integrale += new_integrale;

		if(integrale > AWU_MAX)
				integrale = AWU_MAX;
		else if (integrale < AWU_MIN )
				integrale = AWU_MIN;

		speed_pid = KP_SPEED*error + integrale + KD_SPEED*(error-ancienne_erreur)/ORIENTATION_THREAD_PERIOD;
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
/*************************FUNCTIONS FOR WALL FOLLOWING**********************************/
static int16_t regulator_wall_following(int16_t error)
{
	//PID
	static int16_t old_error = ZERO;
	int16_t wall_following_pid =ZERO;

	integrale_pid_wall_following += KI_ORIENTATION *error*DISTANCE_THREAD_TIME;

	if(integrale_pid_wall_following > ARW_MAX){
			integrale_pid_wall_following = ARW_MAX;
	}else if(integrale_pid_wall_following < ARW_MIN ){
			integrale_pid_wall_following = ARW_MIN;
	}

	wall_following_pid = KP_ORIENTATION*error + integrale_pid_wall_following + (KD_ORIENTATION*(error-old_error))/DISTANCE_THREAD_TIME;
	old_error = error;
	return wall_following_pid;
}

static void follow_wall(int16_t speed_prop){
	uint8_t index_sensor_max = index_highest_sensor_value();
	static int16_t error_orientation_with_wall = ZERO;	//error to keep the robot parallele to wall
	static int16_t error_distance_to_wall = ZERO;		//error to keep the robot at a certain distance from the wall

	int8_t mode_deplacement = get_mode_deplacement(); //gets where the gravity is pointing to

	switch(index_sensor_max){
		case FRONT_RIGHT :					//if the max sensor is on the right -> the wall is on the right
		case BACK_RIGHT :					//so the errors for PID are the errors of the sensors on the right
		case RIGHT :
			error_distance_to_wall = get_error_distance_to_wall_right();
			error_orientation_with_wall=get_error_orientation_right();
			break;
		case FRONT_LEFT :					//if the max sensor is on the left -> the wall is on the left
		case BACK_LEFT :					//so the errors PID are the errors of the sensors on the left
		case LEFT :
			error_distance_to_wall = get_error_distance_to_wall_left();
			error_orientation_with_wall=get_error_orientation_left();
			break;
		default :
			break;
	}

	int16_t pid_wall_following=regulator_wall_following(error_orientation_with_wall);

	if((mode_deplacement==FRONT_RIGHT) || (mode_deplacement==FRONT_LEFT)){ 	//calculates speeds if the gravity is in front
		speed_ext=SPEED_BIAS+pid_wall_following-error_distance_to_wall;		//so robot follows wall forward
		speed_int=SPEED_BIAS-pid_wall_following+error_distance_to_wall;
	}else{
		speed_ext=-SPEED_BIAS+pid_wall_following+error_distance_to_wall; //calculates speeds if gravity is behind
		speed_int=-SPEED_BIAS-pid_wall_following-error_distance_to_wall; //so robot follows wall backward
	}
	if(speed_prop==SPEED_MIN){ //if there is no slope and the robot is following a wall, the robot stops
		speed_ext=SPEED_MIN;
		speed_int=SPEED_MIN;
	}

	if((index_sensor_max==FRONT_RIGHT)||(index_sensor_max==RIGHT)||(index_sensor_max==BACK_RIGHT)){	//if wall on the right -> right wheel=exterior wheel
		 right_motor_set_speed(speed_ext);															//						 left wheel=interior wheel
		 left_motor_set_speed(speed_int);
	}else{									//otherwise, wall on the left -> right wheel=interior wheel
		 right_motor_set_speed(speed_int);	//								left wheel = exterior wheel
		 left_motor_set_speed(speed_ext);
	}
}
/***********************END FUNCITONS FOR WALL FOLLOWING********************************/

/*************************END INTERNAL FUNCTIONS**********************************/


/****************************PUBLIC FUNCTIONS*************************************/
void motors_control_start(void)
{
	motors_init();
	orientation_start();
	//starts motors control thread
	chThdCreateStatic(motor_control_thd_wa, sizeof(motor_control_thd_wa), NORMALPRIO+1, motor_control_thd, NULL);
}
/****************************END PUBLIC FUNCTIONS*************************************/


