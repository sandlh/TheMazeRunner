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


#define Kd 0  // 0.0015
#define Ki 0   //0.003
#define Kp 0.001   //0.01
#define THREAD_TIME 4 //[ms]
#define AWM_MAX  100
#define AWM_MIN  -AWM_MAX

#define NORME_MAX 5000
#define NORME_MIN 1500

#define ACC_MAX 1000

#define FRONT_LEFT 0
#define FRONT_RIGHT 1
#define BACK_RIGHT 2
#define BACK_LEFT 3

#define SPEED_COEFF 5
#define SPEED_MAX 1000
#define SPEED_MIN 0

#define MAX_COEFF 1
#define ROTATION_COEFF 10

static void set_motors_speed(int16_t speed_ext, float speed_int, int8_t mode_deplacement);
static float regulator_speed(int16_t error);

static int16_t speed_proportionelle(int16_t norme);
static float coeff_int(float pid);
void motors_control_start(void);


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

    	int16_t speed_right = 0;
    	int16_t speed_left = 0;
    	int16_t speed_prop = 0;
    	float pid = 0;
    	pid = regulator_speed(error);
    	float speed_correct;

    	speed_prop = speed_proportionelle(norme);

    	//chprintf((BaseSequentialStream *)&SD3, "speed_prop = %d \n", speed_ext); //prints
    	//speed_ext = speed_prop; //- ROTATION_COEFF * speed_correct;
    	//speed_int = speed_prop; //+ ROTATION_COEFF * speed_correct;

		/*
    	float speed_int = 0;
    	float pid = 0;
    	pid = regulator_speed(error);
    	 chprintf((BaseSequentialStream *)&SD3, "pid = %f \n", pid);
    	 if (speed_ext > SPEED_MAX){
    	 		speed_ext = SPEED_MAX;
    	}
    	speed_int = speed_ext*coeff_int( pid);
	*/
    	if (speed_prop > SPEED_MAX){
    	    	 		speed_prop = SPEED_MAX;

    	    	}
    	//chprintf((BaseSequentialStream *)&SD3, "speed_int = %f \n", speed_int); //prints

    	//set_motors_speed(speed_ext, speed_int, mode_deplacement);

    	if(is_there_obstacle()){
    		avoid_obstacle(speed_prop);
    		speed_right=get_speed_right();
    		speed_left=get_speed_left();
    				//chprintf((BaseSequentialStream *)&SD3, "speed_r = %d   speed_l = %d", speed[RIGHT],speed[LEFT]);
    	}else{
    		speed_right=speed_prop;
    		speed_left=speed_prop;
    	}
    	right_motor_set_speed(speed_right);
    	left_motor_set_speed(speed_left);

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
	//chprintf((BaseSequentialStream *)&SD3, "error = %d \n", error); //prints
	//chprintf((BaseSequentialStream *)&SD3, "error = %d \n", error); //prints


	//chprintf((BaseSequentialStream *)&SD3, "coeff= %f \n", pid); //prints

	if (pid > ACC_MAX){
		pid = ACC_MAX;
	}
	coeff_int =  pid/ACC_MAX;
	//chprintf((BaseSequentialStream *)&SD3, "coeff_int = %f \n", coeff_int); //prints

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

		new_integrale = Ki * error*dt;
		integrale += new_integrale;

		if(integrale > AWM_MAX)
				integrale = AWM_MAX;
		else if (integrale < AWM_MIN )
				integrale = AWM_MIN;

		//chprintf((BaseSequentialStream *)&SD3, "error = %d \n", error); //prints

		speed_pid = Kp*error + integrale + Kd*(error-ancienne_erreur)/dt;
		ancienne_erreur = error;
		//chprintf((BaseSequentialStream *)&SD3, "error2 = %d \n", ancienne_erreur); //prints

    	//chprintf((BaseSequentialStream *)&SD3, "speed_PID = %f \n", speed_pid); //prints
		return speed_pid;
	}


static void set_motors_speed(int16_t speed_ext, float speed_int, int8_t mode_deplacement)
{


	switch (mode_deplacement){
			 case FRONT_LEFT:
				 right_motor_set_speed(-speed_int);
				 left_motor_set_speed(-speed_ext);
				 break;
			 case BACK_LEFT:
				 right_motor_set_speed(speed_int);
				 left_motor_set_speed(speed_ext);
				 break;
			 case BACK_RIGHT:
				 right_motor_set_speed(speed_ext);
				 left_motor_set_speed(speed_int);
				 break;
			 case FRONT_RIGHT:
				 right_motor_set_speed(-speed_ext);
				 left_motor_set_speed(-speed_int);
				 break;
		 }
}

void motors_control_start(void)
{
	motors_init();
	orientation_start();

	chThdCreateStatic(motor_control_thd_wa, sizeof(motor_control_thd_wa), NORMALPRIO+1, motor_control_thd, NULL);
}
