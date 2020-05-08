#ifndef MOVE_H_
#define MOVE_H_

#include "ch.h"
#include "hal.h"

enum capteurs{FRONT_RIGHT, FRONT_SIDE_RIGHT,
	          SIDE_RIGHT, BACK_RIGHT,
			  BACK_LEFT, SIDE_LEFT,
			  FRONT_SIDE_LEFT, FRONT_LEFT,
			  NB_CAPTEURS};

enum flip{OFF, ON};

#define AUTO                        4
#define SEMIAUTO                    12

#define THRESHOLD_WALL		      	400 // threshold used to detect a side opening
#define THRESHOLD_FRONT             550 // threshold used to detect a front wall
#define THRESHOLD_BACK              400 // threshold used to detect a back wall

#define WHEEL_PERIMETER				13
#define NBSTEPS_ONE_TURN			1000
#define MIDDLE						50 //adjusted number of steps to get to the middle of a junction
#define QUARTER_TURN_ABS       		327 //423 is the theoretical value, adjusted experimentally
#define HALF_TURN_ABS				654 //846 is the theoretical value, adjusted experimentally
#define KP		                    0.01

#define SLEEP_TIME				 	50 //[ms]


//-----------external functions--------------

void move_start(uint8_t mode);

#endif /* MOVE_H_ */
