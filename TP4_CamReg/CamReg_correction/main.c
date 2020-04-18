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

	int32_t wall_close[NB_CAPTEURS] = {0, 0, 0, 0, 0, 0, 0, 0};
	int32_t path[NB_CAPTEURS] = {0, 0, 0, 0, 0, 0, 0, 0};

    /* Infinite loop. */
    while (1){

    	 //détection des murs à proximité, remplissage tableau (pas de boucle "for" car les capteurs ne sont pas tous utilisés -> 0,1,2,5,6,7
    	measureDists(wall_close);
    	//chprintf((BaseSequentialStream *) &SD3, "Capteur front left calibrated: %d \n", wall_close[FRONT_LEFT]);
    	//chprintf((BaseSequentialStream *) &SD3, "Capteur front right calibrated: %d \n", wall_close[FRONT_RIGHT]);
		//chprintf((BaseSequentialStream *) &SD3, "Capteur front side left calibrated: %d \n", wall_close[FRONT_SIDE_LEFT]);
		chprintf((BaseSequentialStream *) &SD3, "Capteur front side right calibrated: %d \n", wall_close[FRONT_SIDE_RIGHT]);
		//chprintf((BaseSequentialStream *) &SD3, "Capteur side left calibrated: %d \n", wall_close[SIDE_LEFT]);
		//chprintf((BaseSequentialStream *) &SD3, "Capteur side right calibrated: %d \n", wall_close[SIDE_RIGHT]);

		//détection des ouvertures à proximité, remplissage tableau, algorithme choix du chemin
		findPath(path);
	 	//chprintf((BaseSequentialStream *) &SD3, "Capteur front left: %d \n", path[FRONT_LEFT]);
	    //chprintf((BaseSequentialStream *) &SD3, "Capteur front right: %d \n", path[FRONT_RIGHT]);
		//chprintf((BaseSequentialStream *) &SD3, "Capteur front side left: %d \n", path[FRONT_SIDE_LEFT]);
		chprintf((BaseSequentialStream *) &SD3, "Capteur front side right: %d \n", path[FRONT_SIDE_RIGHT]);
		//chprintf((BaseSequentialStream *) &SD3, "Capteur side left: %d \n", path[SIDE_LEFT]);
		//chprintf((BaseSequentialStream *) &SD3, "Capteur side right: %d \n", path[SIDE_RIGHT]);


		//algorithme de résolution du labyrinthe,
		//détection bancale des passage (si mur s'éloigne et assez éloigné, détection du passage) -> probablement à améliorer
		//à vérifier: tourner au bon moment, ne pas activer le demi-tour par erreur, détection correcte des passages, etc...
		//si passage à droite, y aller
		if(path[SIDE_RIGHT] > get_prox(SIDE_RIGHT) && get_prox(SIDE_RIGHT) < DISTANCE_PASSAGE)
		{
			chprintf((BaseSequentialStream *) &SD3, "passage à droite détecté");
			//fonctions pour faire tourner le robot à droite
		}
		//si pas de passage à droite, aller tout droit
		else if(path[FRONT_LEFT] > get_prox(FRONT_LEFT) && path[FRONT_RIGHT] > get_prox(FRONT_RIGHT))
		{
			chprintf((BaseSequentialStream *) &SD3, "passage devant détecté");
			//fonctions pour faire avancer le robot tout droit
		}
		//si passage ni à droite, ni tout droit, mais à gauche, aller à gauche
		else if(path[SIDE_LEFT] > get_prox(SIDE_LEFT) && get_prox(SIDE_LEFT) < DISTANCE_PASSAGE)
		{
			chprintf((BaseSequentialStream *) &SD3, "passage à gauche détecté");
			//fonctions pour faire tourner le robot à gauche
		}
		//si aucun passage n'existe, faire demi-tour
		else
		{
			chprintf((BaseSequentialStream *) &SD3, "aucun passage détecté, demi-tour");
			//fonctions pour faire un demi-tour
		}


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
