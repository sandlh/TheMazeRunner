#include "motors_control.h"
#include "orientation.h"
#include "distance.h"

#include <motors.h>
#include <math.h>
#include <stdbool.h>

/*#include "ch.h" for chprintf
#include "hal.h"
#include <chprintf.h>
#include <usbcfg.h>
#include <msgbus/messagebus.h>*/

#define Kibis 0.1
#define Kd 0.011  // 0.0015
#define Ki 0.0025   //0.003
#define Kp 0.05    //0.01
#define THREAD_TIME 4 //[ms]
#define FRONT 0
#define BACK 1
#define LEFT 2
#define RIGHT 3
static uint8_t mode_deplacement=16;
#define AWM_MAX  100
#define AWM_MIN  -AWM_MAX

/*test avoid obstacle */
#define SAFE_DISTANCE 900
static int speed_translation=0;
/* end test avoid obstacle */

/***************************INTERNAL FUNCTIONS************************************/

static void set_motors_speed(float speed);
static void set_mode_deplacement(float error, bool meme_signe, bool sens);
static void set_motors_speed_PID(float speed);
static int regulator_speed(float error);
static void set_motors_speed_pente(float pente);
void motors_control_start(void);
/*test avoid obstacle */
static void avoid_obstacle(void);
static void go_back_forth(void);
/* end test avoid obstacle */

static THD_WORKING_AREA(motor_control_thd_wa, 512);
static THD_FUNCTION(motor_control_thd, arg)
{
     (void) arg;
     chRegSetThreadName(__FUNCTION__);

     float error = 0;
     float pente =0;
     systime_t time;
     while(true)
     {
    	time = chVTGetSystemTime();

    	pente = get_pente();
		//error = get_error();

		//set_mode_deplacement(error, meme_signe, sens);
		//set_motors_speed(100);// provisoire
		//if (error == 0){
			//set_motors_speed_pente(pente);
	//	}else{
		/*if (error == 0){
			right_motor_set_speed(0);
			left_motor_set_speed(0);
		}else {
		//set_motors_speed_PID(regulator_speed(error));
		}*/
		//go_back_forth();
		//avoid_obstacle();
		//right_motor_set_speed(speed_translation);
		//left_motor_set_speed(speed_translation);
		chThdSleepUntilWindowed(time, time + MS2ST(THREAD_TIME));
     }
}
//PID pour faire tourner les moteurs par la suite afin d'avoir un meilleur ajustement et aussi pouvoir faire évoluer la trajectoire

static void set_motors_speed_pente(float pente){// ne marche pas

	uint8_t dt = 4;
	static float integ = 0;
	static float speed = 0;
	integ = Kibis *pente*dt;
	speed += integ;
	if ( pente )
	right_motor_set_speed(speed);
	left_motor_set_speed(speed);

}

static int regulator_speed(float error)
{
	//PID
	uint8_t dt = 4;
	static float ancienne_erreur = 0;
	float speed_pid =0;

	float new_integrale = 0;
	static float integrale = 0;

	new_integrale = Ki * error*dt;
	integrale += new_integrale;

	if(integrale > AWM_MAX)
			integrale = AWM_MAX;
	else if (integrale < AWM_MIN )
			integrale = AWM_MIN;

	speed_pid = Kp*error + integrale + Kd*(error-ancienne_erreur)/dt;
	ancienne_erreur = error;
	return speed_pid;
}

static void set_motors_speed_PID(float speed){

		right_motor_set_speed(-speed);
		left_motor_set_speed(speed);

}
/*
 static void set_motors_speed(float speed){ // avant PID pour les tests

	 switch (mode_deplacement){
		 case FRONT:
			 right_motor_set_speed(speed);
			 left_motor_set_speed(speed);
			 break;
		 case BACK:
			 right_motor_set_speed(-speed);
			 left_motor_set_speed(-speed);
			 break;
		 case LEFT:
			 right_motor_set_speed(speed);
			 left_motor_set_speed(-speed);
			 break;
		 case RIGHT:
			 right_motor_set_speed(-speed);
			 left_motor_set_speed(speed);
			 break;
	 }
}

static void set_mode_deplacement(float error, bool meme_signe, bool sens){   // cette fonction était là pour les tests avant le PID
	if ((error == 0) && (sens==1)){ // && (sens==1))
		mode_deplacement = FRONT;
	}else if ((error == 0)&&(sens ==0)) {
		mode_deplacement = BACK;
	}else if (meme_signe == 1){
		mode_deplacement = RIGHT;
	} else {
		mode_deplacement = LEFT;
	}
}
*/

/***********************Test avoid obstacle***********************************/
static void avoid_obstacle(void){
	uint16_t distance_front_right = get_distance_front_right();
	uint16_t distance_front_left = get_distance_front_left();
	uint16_t distance_front = distance_front_right + distance_front_left;

	uint16_t distance_back_right = get_distance_back_right();
	uint16_t distance_back_left = get_distance_back_left();
	uint16_t distance_back = distance_back_right + distance_back_left;


	if((distance_front > SAFE_DISTANCE) & (speed_translation>0)){
		speed_translation = 0;
	}
	if((distance_back > SAFE_DISTANCE) & (speed_translation<0)){
		speed_translation=0;
	}
}

static void go_back_forth(void){
	static uint16_t i=0;
	i++;
	if(i<1000){
		speed_translation=400;
	}else if(i>=1000){
		speed_translation=-400;
	}if(i>=2000){
		i=0;
	}
}

/***********************End test avoid obstacle*******************************/


/*************************END INTERNAL FUNCTIONS**********************************/


/****************************PUBLIC FUNCTIONS*************************************/

void motors_control_start(void)
{
	//initialisation
	motors_init();
	orientation_start();

	chThdCreateStatic(motor_control_thd_wa, sizeof(motor_control_thd_wa), NORMALPRIO+1, motor_control_thd, NULL);
}
/**************************END PUBLIC FUNCTIONS***********************************/
