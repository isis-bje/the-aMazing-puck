#include "move.h"

#include "ch.h"
#include "hal.h"
#include <chprintf.h>
#include <usbcfg.h>
#include <sensors/proximity.h>
#include <motors.h>

uint8_t junction_detection(int32_t find_path[NB_CAPTEURS]);
void measure_dist_cal(int32_t dist_cal[NB_CAPTEURS]);
void measure_dist(int32_t dist[NB_CAPTEURS]);

void print_calibrated_measures(int32_t path_cal[NB_CAPTEURS]);
void print_measures(int32_t path[NB_CAPTEURS]);

int8_t steps_to_cm(int16_t nb_steps);
int16_t cm_to_steps(int8_t dist_cm);
void move_command(uint8_t node_type/*, bool state*/);


//Thread that controls the movement of the robot

static THD_WORKING_AREA(waThdMove, 512);
static THD_FUNCTION(ThdMove, arg)
{
    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    uint8_t node_type = 0;
	int32_t path[NB_CAPTEURS] = {0, 0, 0, 0, 0, 0, 0, 0};
	/*int32_t path_cal[NB_CAPTEURS] = {0, 0, 0, 0, 0, 0, 0, 0};*/

	while(1){

		node_type = junction_detection(path);
		move_command(node_type);

		chThdSleepMilliseconds(SLEEP_TIME);
	}

}

//-------------------------------------------------------------------------------------

// EXTERNAL FUNCTIONS

void move_start(void){

	right_motor_set_speed(0);
	left_motor_set_speed(0);

	chThdCreateStatic(waThdMove, sizeof(waThdMove), NORMALPRIO, ThdMove, NULL);
}

//-------------------------------------------------------------------------------------

// INTERNAL FUNCTIONS

//algorithme de d�tection du type de jonction
uint8_t junction_detection(int32_t find_path[]){
	uint8_t node_type = 0;
	measure_dist(find_path);

	if(find_path[FRONT_LEFT] < THRESHOLD_WALL && find_path[FRONT_RIGHT] < THRESHOLD_WALL) //si passage devant ouvert
	{
		if(find_path[SIDE_LEFT] < THRESHOLD_WALL && find_path[SIDE_RIGHT] < THRESHOLD_WALL) //si passage � gauche et � droite
		{
			node_type = CROSSROAD;
		}
		else if(find_path[SIDE_LEFT] < THRESHOLD_WALL) //si passage � gauche
		{
			node_type = T_JUNCTION_LEFT;
		}
		else if(find_path[SIDE_RIGHT] < THRESHOLD_WALL) //si passage � droite
		{
			node_type = T_JUNCTION_RIGHT;
		}
		else //passage uniquement tout droit
		{
			node_type = STRAIGHT_PATH;
		}
	}
	else //si pas de passage devant
	{
		if(find_path[SIDE_LEFT] < THRESHOLD_WALL && find_path[SIDE_RIGHT] < THRESHOLD_WALL) //si passage � gauche et � droite
		{
			node_type = T_JUNCTION;
		}
		else if(find_path[SIDE_LEFT] < THRESHOLD_WALL) //si passage � gauche
		{
			node_type = CORNER_LEFT;
		}
		else if(find_path[SIDE_RIGHT] < THRESHOLD_WALL) //si passage � droite
		{
			node_type = CORNER_RIGHT;
		}
		else //aucun passage
		{
			node_type = CUL_DE_SAC;
		}
	}
	return node_type;
}

void move_command(uint8_t node_type/*, bool state*/){

	/*if(state) //automatic mode
	{*/
		switch(node_type)
		{
			case CROSSROAD :
				turn_right();
				break;

			case T_JUNCTION_LEFT :
				go_forward();
				break;

			case T_JUNCTION_RIGHT :
				turn_right();
				break;

			case T_JUNCTION :
				turn_right();
				break;

			default :
				break;
		}
	/*}
	else //semi-automatic mode
	{
		switch(node_type)
		{
			case CROSSROAD :
				get_sound_order();
				break;

			case T_JUNCTION_LEFT :
				get_sound_order();
				break;

			case T_JUNCTION_RIGHT :
				get_sound_order();
				break;

			case T_JUNCTION :
				get_sound_order();
				break;

			default :
				break;
		}
	}*/

	switch(node_type)
	{
		case NODE_ERROR :
			stop();
			chprintf((BaseSequentialStream *) &SD3, "error");
			break;

		case STRAIGHT_PATH :
			go_forward();
			break;

		case CORNER_LEFT :
			turn_left();
			break;

		case CORNER_RIGHT :
			turn_right();
			break;

		case CUL_DE_SAC :
			half_turn();
			break;

		default :
			break;
	}
}


void print_calibrated_measures(int32_t path_cal[]){

	measure_dist_cal(path_cal);

	chprintf((BaseSequentialStream *) &SD3, "Capteur front left calibrated: %d \n", path_cal[FRONT_LEFT]);
	chprintf((BaseSequentialStream *) &SD3, "Capteur front right calibrated: %d \n", path_cal[FRONT_RIGHT]);
	chprintf((BaseSequentialStream *) &SD3, "Capteur front side left calibrated: %d \n", path_cal[FRONT_SIDE_LEFT]);
	chprintf((BaseSequentialStream *) &SD3, "Capteur front side right calibrated: %d \n", path_cal[FRONT_SIDE_RIGHT]);
	chprintf((BaseSequentialStream *) &SD3, "Capteur side left calibrated: %d \n", path_cal[SIDE_LEFT]);
	chprintf((BaseSequentialStream *) &SD3, "Capteur side right calibrated: %d \n", path_cal[SIDE_RIGHT]);

}

void print_measures(int32_t path[]){

	chprintf((BaseSequentialStream *) &SD3, "Capteur front left: %d \n", path[FRONT_LEFT]);
	chprintf((BaseSequentialStream *) &SD3, "Capteur front right: %d \n", path[FRONT_RIGHT]);
	chprintf((BaseSequentialStream *) &SD3, "Capteur front side left: %d \n", path[FRONT_SIDE_LEFT]);
	chprintf((BaseSequentialStream *) &SD3, "Capteur front side right: %d \n", path[FRONT_SIDE_RIGHT]);
	chprintf((BaseSequentialStream *) &SD3, "Capteur side left: %d \n", path[SIDE_LEFT]);
	chprintf((BaseSequentialStream *) &SD3, "Capteur side right: %d \n", path[SIDE_RIGHT]);
}

void measure_dist_cal(int32_t dist_cal[NB_CAPTEURS]){

	dist_cal[FRONT_RIGHT] = get_calibrated_prox(FRONT_RIGHT);
	dist_cal[FRONT_LEFT] = get_calibrated_prox(FRONT_LEFT);
	dist_cal[FRONT_SIDE_RIGHT] = get_calibrated_prox(FRONT_SIDE_RIGHT);
	dist_cal[FRONT_SIDE_LEFT] = get_calibrated_prox(FRONT_SIDE_LEFT);
	dist_cal[SIDE_RIGHT] = get_calibrated_prox(SIDE_RIGHT);
	dist_cal[SIDE_LEFT] = get_calibrated_prox(SIDE_LEFT);
}

void measure_dist(int32_t dist[NB_CAPTEURS]){

	dist[FRONT_RIGHT] = get_prox(FRONT_RIGHT);
	dist[FRONT_LEFT] = get_prox(FRONT_LEFT);
	dist[FRONT_SIDE_RIGHT] = get_prox(FRONT_SIDE_RIGHT);
	dist[FRONT_SIDE_LEFT] = get_prox(FRONT_SIDE_LEFT);
	dist[SIDE_RIGHT] = get_prox(SIDE_RIGHT);
	dist[SIDE_LEFT] = get_prox(SIDE_LEFT);
}

int8_t steps_to_cm(int16_t nb_steps){ // from -100 - 100 cm to -32000 - 32000 steps

	return nb_steps*WHEEL_PERIMETER/NBSTEPS_ONE_TURN;
}

int16_t cm_to_steps(int8_t dist_cm){ // from -100 - 100 cm to -32000 - 32000 steps

	return dist_cm*NBSTEPS_ONE_TURN/WHEEL_PERIMETER;
}

void stop(){
	left_motor_set_speed(0);
	right_motor_set_speed(0);
}

void turn_right(){
	left_motor_set_speed(500);
	right_motor_set_speed(-500);
}

void turn_left(){
	left_motor_set_speed(-500);
	right_motor_set_speed(500);
}

void go_forward(){
	left_motor_set_speed(500);
	right_motor_set_speed(500);
}

void half_turn(){
	left_motor_set_speed(-500);
	right_motor_set_speed(500);
}
