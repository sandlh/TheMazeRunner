#include <ch.h>
#include <hal.h>
#include <memory_protection.h>
#include <main.h>
#include <motors.h>


#include "motors_control.h"

int main(void)
{
	halInit();
	chSysInit();
	mpu_init();

	motors_control_start();


	systime_t time;
	while (true) {
	    	time = chVTGetSystemTime();

	    	//waits 0.05 second
	    	chThdSleepUntilWindowed(time, time + MS2ST(50));
	    }
	}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
