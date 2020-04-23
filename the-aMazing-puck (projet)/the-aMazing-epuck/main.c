// From the ePuck library

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

// Our files

#include <main.h>
#include <move.h>

//declaration necessaire pour faire fonctionner le capteur de distance ?
messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);

//NE PAS RETIRER
static void serial_start(void)
{
	static SerialConfig ser_cfg = {
	    115200,
	    0,
	    0,
	    0,
	};

	sdStart(&SD3, &ser_cfg); //UART3.
}

int main(void)
{
    halInit();
    chSysInit();
    mpu_init();

    //NE PAS RETIRER
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
	//starts the threads that controls the movement of the robot
	move_start();

    /* Infinite loop. */
    while (1){

		//waits 1 second
        chThdSleepMilliseconds(1000);
    }
}

// NE PAS RETIRER 
#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
