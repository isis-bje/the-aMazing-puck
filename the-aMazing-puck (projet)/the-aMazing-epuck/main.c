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
#include <selector.h>
#include <motors.h>
#include <sensors/proximity.h>
#include <audio/microphone.h>

#include <main.h>
#include <move.h>
#include <sound.h>

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

    //start the microphone
    mic_start(&processSound); //processSound is a callback function used when samples are ready

	//LAUNCH MODE : OTHER -> The epuck is still until a mode is selected
    //				SOUTH -> AUTOMATIC MODE (4)
	//				NORTH -> SEMI-AUTOMATIC MODE (12)

    static uint8_t mode = 0;

    while (mode != AUTO && mode != SEMIAUTO){

    	mode = get_selector();
    	chThdSleepMilliseconds(1000);

    }

    if(mode == AUTO){
    	chprintf((BaseSequentialStream *) &SD3, "AUTOMATIC\r\n");
    }
    else if (mode == SEMIAUTO){
		chprintf((BaseSequentialStream *) &SD3, "SEMI-AUTO\r\n");
    }

    //starts the threads that controls the movement of the robot
    move_start(mode);

    /* Infinite loop. */
    while (1){

    	//waits 1 second
    	chThdSleepMilliseconds(1000);

    	chprintf((BaseSequentialStream *) &SD3, "Sleep\r\n"); //to be removed

    }
}

//-------------------------------------------------------------------------------------

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void){

    chSysHalt("Stack smashing detected");
}
