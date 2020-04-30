#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "ch.h"
#include "hal.h"
#include "memory_protection.h"

#include <usbcfg.h>
#include <chprintf.h>
#include <motors.h>
#include <sensors/proximity.h>
#include <selector.h>

#include <main.h>
#include <move.h>

messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);

static void serial_start(void){

	static SerialConfig ser_cfg = {
	    115200,
	    0,
	    0,
	    0,
	};

	sdStart(&SD3, &ser_cfg); //UART3.
}

//-----------------------------------------MAIN------------------------------------------

int main(void){

	//static uint8_t mode;

	halInit();
    chSysInit();
    mpu_init();

    messagebus_init(&bus, &bus_lock, &bus_condvar);

    //starts the serial communication
    serial_start();

    //start the USB communication
    usb_start();

    //starts the proximity sensors
    proximity_start();
    calibrate_ir();

    //initialize the motors
	motors_init();

	//LAUNCH MODE : SELECTOR IN 0 -> AUTOMATIC MODE
	//				SELECTOR IN 1 -> SEMI-AUTOMATIC MODE
	//AFTER LAUNCHING -> SELECTOR IS USED TO PAUSE OR UNPAUSE (to be done)

	//starts the threads that controls the movement of the robot
	move_start(); //doit recuperer le mode

    /* Infinite loop. */
    while (1){

       	uint8_t test;
        test = get_selector();
        chprintf((BaseSequentialStream *) &SD3, "mode = %d\n", test);

        chThdSleepMilliseconds(5000); //waits 1 second
    }
}

//-------------------------------------------------------------------------------------

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void){

    chSysHalt("Stack smashing detected");
}
