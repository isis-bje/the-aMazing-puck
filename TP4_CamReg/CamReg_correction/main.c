#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "ch.h"
#include "hal.h"
#include "memory_protection.h"
#include <usbcfg.h>
#include <main.h>
#include <motors.h>
#include <camera/po8030.h>
#include <chprintf.h>

#include <pi_regulator.h>
#include <process_image.h>

#include <sensors/proximity.h>
#include <capteur_distance_test.h>

/* void SendUint8ToComputer(uint8_t* data, uint16_t size)
{
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)"START", 5);
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)&size, sizeof(uint16_t));
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)data, size);
}*/


//declaration necessaire pour faire fonctionner le capteur de distance
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

	sdStart(&SD3, &ser_cfg); // UART3.
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


    //starts the camera
    //dcmi_start();
	//po8030_start();

    //inits the motors
	motors_init();

	//stars the threads for the pi regulator and the processing of the image
	//pi_regulator_start();
	//process_image_start();

	int32_t mur_proche[NB_CAPTEURS] = {0, 0, 0, 0, 0, 0, 0, 0};

    /* Infinite loop. */
    while (1){

    	 //détection des murs à proximité, remplissage tableau (pas de boucle "for" car les capteurs ne sont pas tous utilisés -> 0,1,2,5,6,7
    	measureDists(mur_proche);
    	//chprintf((BaseSequentialStream *) &SD3, "Capteur front left: %d \n", mur_proche[FRONT_LEFT]);
    	//chprintf((BaseSequentialStream *) &SD3, "Capteur front right: %d \n", mur_proche[FRONT_RIGHT]);
		//chprintf((BaseSequentialStream *) &SD3, "Capteur front side left: %d \n", mur_proche[FRONT_SIDE_LEFT]);
		chprintf((BaseSequentialStream *) &SD3, "Capteur front side right: %d \n", mur_proche[FRONT_SIDE_RIGHT]);
		//chprintf((BaseSequentialStream *) &SD3, "Capteur side left: %d \n", mur_proche[SIDE_LEFT]);
		//chprintf((BaseSequentialStream *) &SD3, "Capteur side right: %d \n", mur_proche[SIDE_RIGHT]);
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
