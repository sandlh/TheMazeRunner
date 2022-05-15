/*
 * main.c
 *
 *  Created on: Apr 25, 2022
 *      Author: Sandra L'Herminé et Barbara de Groot
 */
#include <ch.h>
#include <hal.h>
#include <memory_protection.h>
#include <main.h>
#include <motors.h>
#include <usbcfg.h>
#include <chprintf.h>


#include "motors_control.h"
#include "distance.h"

static void serial_start(void)
{
	static SerialConfig ser_cfg = {
	    115200,
	    0,
	    0,
	    0,
	};

	sdStart(&SD3, &ser_cfg); // UART3.
}

int main(void)
{
	halInit();
	chSysInit();
	mpu_init();

	//starts the serial communication
    serial_start();
    //starts the USB communication
    usb_start();

	motors_control_start();
	distance_start();

	systime_t time;
	while (true) {
	    	time = chVTGetSystemTime();
	    	//waits 0.05 second
	    	chThdSleepUntilWindowed(time, time + MS2ST(MAIN_THREAD_TIME));
	    }
	}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
