/*
 * main.c
 *
 *  Created on: 15 Apr 2020
 *      Author: Julien MOREL et Isis BOU JAOUDE
 */

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
	//static uint8_t pause;

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

    //start the microphone

    //initialize the motors
	motors_init();

	//starts the threads that controls the movement of the robot
	move_start();

	//LAUNCH MODE : SOUTH -> AUTOMATIC MODE (4)
	//				NORTH -> SEMI-AUTOMATIC MODE (12)
	//			    WEST  -> PAUSE MODE (SLEEP) (8)
	// 				EAST  -> STOP MODE (REINITIALIZE) (0)

	/* starts the threads that controls the movement of the robot
	move_start(); doit recuperer le mode */
	//chprintf((BaseSequentialStream *) &SD3, "mode = %d\r\n", mode);

	/*mode = get_selector();
	/if(mode == 4){
		chprintf((BaseSequentialStream *) &SD3, "AUTOMATIC\r\n");
	}
	else if (mode == 12){
		chprintf((BaseSequentialStream *) &SD3, "SEMI-AUTO\r\n");;
	}
	else
		return 0;*/

    /* Infinite loop. */
    while (1){

    	/*pause = get_selector();
    	if(pause == 8){
    		chprintf((BaseSequentialStream *) &SD3, "PAUSE\r\n");
    		//pause_function, arret des moteurs, allumage de LEDS
    	}
    	else {
    		chprintf((BaseSequentialStream *) &SD3, "WORKING\r\n");
    	}*/

    	chThdSleepMilliseconds(1000); //waits 1 second
    }
}

//-------------------------------------------------------------------------------------

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void){

    chSysHalt("Stack smashing detected");
}
