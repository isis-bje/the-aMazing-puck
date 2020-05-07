#ifndef MOVE_H_
#define MOVE_H_

#include "ch.h"
#include "hal.h"

enum nodes{NODE_ERROR, CROSSROAD, T_JUNCTION_LEFT, T_JUNCTION_RIGHT, T_JUNCTION,
		   STRAIGHT_PATH, CORNER_LEFT, CORNER_RIGHT, CUL_DE_SAC};

enum capteurs{FRONT_RIGHT, FRONT_SIDE_RIGHT,
	          SIDE_RIGHT, BACK_RIGHT,
			  BACK_LEFT, SIDE_LEFT,
			  FRONT_SIDE_LEFT, FRONT_LEFT,
			  NB_CAPTEURS};

enum flip{OFF, ON};

#define AUTO                        4
#define SEMIAUTO                    12

#define THRESHOLD_WALL		      	450 // valeur de seuil pour détecter une ouverture, ajustée expérimentalement
#define THRESHOLD_FRONT             500 // valeur de seuil pour détecter un mur frontal, ajustée expérimentalement
#define THRESHOLD_BACK              400 // valeur de seuil pour détecter un mur arrière, ajustée expérimentalement

#define WHEEL_PERIMETER				13
#define NBSTEPS_ONE_TURN			1000
#define QUARTER_TURN_ABS       		327 //423 selon le calcul théorique, ajusté expérimentalement pour 1/4 tour
#define HALF_TURN_ABS				654 //846 selon le calcul théorique, ajusté expérimentalement pour 1/2 tour
#define KP							0   // to be adjusted

#define SLEEP_TIME				 	50 //[ms]


//-----------external functions--------------

void move_start(uint8_t mode);

#endif /* MOVE_H_ */
