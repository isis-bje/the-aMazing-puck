
#include "ch.h"
#include "hal.h"
#include <chprintf.h>
#include <usbcfg.h>

#include <sensors/proximity.h>
#include <motors.h>
#include <leds.h>

#include <sound.h>
#include <move.h>

//---------------------------------CONSTANTS---------------------------------

enum capteurs{FRONT_RIGHT,
			  FRONT_SIDE_RIGHT,
	          SIDE_RIGHT,
			  BACK_RIGHT,
			  BACK_LEFT,
			  SIDE_LEFT,
			  FRONT_SIDE_LEFT,
			  FRONT_LEFT,
			  NB_CAPTEURS};

enum flip{OFF, ON};

#define THRESHOLD_SIDE_WALL		    400  // threshold used to detect a side opening
#define THRESHOLD_FRONT             550  // threshold used to detect a front wall
#define THRESHOLD_BACK              400  // threshold used to detect a back wall

#define WHEEL_PERIMETER				13
#define NBSTEPS_ONE_TURN			1000 // theoretical value
#define MIDDLE_JUNCTION				260  // adjusted number of steps to get to the middle of a junction
#define QUARTER_TURN_ABS       		327  // adjusted experimentally, 423 is the theoretical value
#define HALF_TURN_ABS				654  // adjusted experimentally, 846 is the theoretical value
#define KP		                    0.01 // adjusted experimentally

//-----------------------------Static Variables-----------------------------

static uint8_t program;

//---------------------------Internal declarations---------------------------

void move_command(uint8_t node_type);
uint8_t junction_detection(int32_t find_path[NB_CAPTEURS]);

void automatic_command(uint8_t node_type);
void semiautomatic_command(uint8_t node_type);
void general_command(uint8_t node_type);
void execute_sound_command(uint8_t command);

void stop(void);
void turn_right_90(void);
void turn_left_90(void);
void go_forward_regulator(void);
void go_forward(void);
void half_turn(void);

void measure_dist(int32_t dist[NB_CAPTEURS]);
void print_measures(int32_t path[NB_CAPTEURS]);

//------------------------------Movement Thread------------------------------

static THD_WORKING_AREA(waThdMove, 512);
static THD_FUNCTION(ThdMove, arg){

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    uint8_t node_type = 0;
	int32_t path[NB_CAPTEURS] = {0, 0, 0, 0, 0, 0, 0, 0};

	while(1){

		node_type = junction_detection(path);
		//print_measures(path);

		move_command(node_type);

		chThdSleepMilliseconds(SLEEP_TIME);
	}
}

//-----------------------------External functions-----------------------------

/* Purpose : Launches the movement thread,
 * 		     Retrieves the mode used passing it to the "program" variable.
 *
 * Parameters :
 *
 * (in)  uint8_t mode	Indicates the mode the e-puck is in.
 *
 */
void move_start(uint8_t mode){

	right_motor_set_speed(0);
	left_motor_set_speed(0);

	program = mode;

	chThdCreateStatic(waThdMove, sizeof(waThdMove), NORMALPRIO, ThdMove, NULL);
}

//-----------------------------Internal functions-----------------------------

/* Purpose : Launches the correct execution function according to the mode the e-puck is in.
 *
 * Parameters :
 *
 * (in)  uint8_t node_type	 Indicates the type of junction the e-puck is in.
 *
 */

void move_command(uint8_t node_type){

	if(program == AUTO){ //automatic mode

		automatic_command(node_type);
	}
	else if(program == SEMIAUTO){ //semi-automatic mode

		semiautomatic_command(node_type);
	}
}

/* Purpose : Compare the values returned by the proximity sensors with thresholds
 * 			 in order to determine the junction the e-puck is in
 *
 * Parameters :
 *
 * (in)  in32_t find_path[]
 *
 */
uint8_t junction_detection(int32_t find_path[]){

	uint8_t node_type = 0;

	measure_dist(find_path);   																				// fill an array with proximity sensors values

	if(find_path[SIDE_LEFT] > THRESHOLD_SIDE_WALL && find_path[SIDE_RIGHT] > THRESHOLD_SIDE_WALL)						// if no opening neither left nor right
	{
		if(find_path[FRONT_LEFT] < THRESHOLD_FRONT && find_path[FRONT_RIGHT] < THRESHOLD_FRONT) 			// if opening forward
		{
			node_type = STRAIGHT_PATH;
		}
		else																								// if no opening forward
		{
			node_type = CUL_DE_SAC;
		}
	}
	else if(find_path[SIDE_LEFT] < THRESHOLD_SIDE_WALL || find_path[SIDE_RIGHT] < THRESHOLD_SIDE_WALL)  				// if opening left or right
	{
		left_motor_set_pos(0);
		right_motor_set_pos(0);
		int left_pos = 0;
		int right_pos = 0;
		chprintf((BaseSequentialStream *) &SD3, "JUNCTION DETECTED, GOING INSIDE \r\n");
		do{
			go_forward();
			measure_dist(find_path);
			left_pos = left_motor_get_pos();
			right_pos = right_motor_get_pos();
			chprintf((BaseSequentialStream *) &SD3, "steps left: %d \r\n", left_pos);
			chprintf((BaseSequentialStream *) &SD3, "steps right: %d \r\n", right_pos);
		}while((left_pos < MIDDLE_JUNCTION && right_pos  < MIDDLE_JUNCTION) &&                         // go to the middle of the junction
			   (find_path[FRONT_LEFT] < THRESHOLD_FRONT && find_path[FRONT_RIGHT] < THRESHOLD_FRONT));      // if too close to a wall
		stop();

		if(find_path[SIDE_LEFT] < THRESHOLD_SIDE_WALL && find_path[SIDE_RIGHT] < THRESHOLD_SIDE_WALL) 				// if opening left and right
		{
			if(find_path[FRONT_LEFT] < THRESHOLD_FRONT && find_path[FRONT_RIGHT] < THRESHOLD_FRONT) 		// if opening forward
			{
				node_type = CROSSROAD;
			}
			else																							// if no opening forward
			{
				node_type = T_JUNCTION;
			}
		}
		else if(find_path[SIDE_LEFT] < THRESHOLD_SIDE_WALL && find_path[SIDE_RIGHT] > THRESHOLD_SIDE_WALL) 			// if opening only left
		{
			if(find_path[FRONT_LEFT] < THRESHOLD_FRONT && find_path[FRONT_RIGHT] < THRESHOLD_FRONT) 		// if opening forward
			{
				node_type = T_JUNCTION_LEFT;
			}
			else																							// if no opening forward
			{
				node_type = CORNER_LEFT;
			}
		}
		else if(find_path[SIDE_LEFT] > THRESHOLD_SIDE_WALL && find_path[SIDE_RIGHT] < THRESHOLD_SIDE_WALL) 			// if opening only right
		{
			if(find_path[FRONT_LEFT] < THRESHOLD_FRONT && find_path[FRONT_RIGHT] < THRESHOLD_FRONT) 		// if opening forward
			{
				node_type = T_JUNCTION_RIGHT;
			}
			else																							// if no opening forawrd
			{
				node_type = CORNER_RIGHT;
			}
		}
	}
	chprintf((BaseSequentialStream *) &SD3, "node_type: %d\r\n", node_type);

	return node_type;
}

void automatic_command(uint8_t node_type){

	switch(node_type){

		case CROSSROAD :
			chprintf((BaseSequentialStream *) &SD3, "CROSSROAD, TURNING RIGHT \r\n");
			turn_right_90();
			while(get_prox(SIDE_LEFT) < THRESHOLD_SIDE_WALL && get_prox(SIDE_RIGHT) < THRESHOLD_SIDE_WALL){  //tant que le croisement n'a pas été passé
				go_forward();
			}
			break;

		case T_JUNCTION_LEFT :
			chprintf((BaseSequentialStream *) &SD3, "T LEFT, GOING FORWARD \r\n");
			go_forward();
			chprintf((BaseSequentialStream *) &SD3, "T LEFT, I GOT OUT \r\n");
			break;

		case T_JUNCTION_RIGHT :
			chprintf((BaseSequentialStream *) &SD3, "T RIGHT, TURNING RIGHT \r\n");
			turn_right_90();
			while(get_prox(SIDE_LEFT) < THRESHOLD_SIDE_WALL && get_prox(SIDE_RIGHT) < THRESHOLD_SIDE_WALL){  //tant que le croisement n'a pas été passé
				go_forward();
			}
			break;

		case T_JUNCTION :
			chprintf((BaseSequentialStream *) &SD3, "T JUNCTION, TURNING RIGHT \r\n");
			turn_right_90();
			while(get_prox(SIDE_RIGHT) < THRESHOLD_SIDE_WALL){  //tant que le croisement n'a pas été passé
				go_forward();
			}
			break;

		default :
			break;
	}

	general_command(node_type);
}

void semiautomatic_command(uint8_t node_type){

	uint8_t command = 0;

	switch(node_type){

		case CROSSROAD :
			set_led(LED1, ON);
			set_led(LED3, ON);
			set_led(LED5, ON);
			set_led(LED7, ON);
			command = wait_receive_order(node_type);
			set_led(LED1, OFF);
			set_led(LED3, OFF);
			set_led(LED5, OFF);
			set_led(LED7, OFF);
			break;

		case T_JUNCTION_LEFT :
			set_led(LED1, ON);
			set_led(LED3, ON);
			set_led(LED5, ON);
			set_led(LED7, ON);
			command = wait_receive_order(node_type);
			set_led(LED1, OFF);
			set_led(LED3, OFF);
			set_led(LED5, OFF);
			set_led(LED7, OFF);
			break;

		case T_JUNCTION_RIGHT :
			set_led(LED1, ON);
			set_led(LED3, ON);
			set_led(LED5, ON);
			set_led(LED7, ON);
			command = wait_receive_order(node_type);
			set_led(LED1, OFF);
			set_led(LED3, OFF);
			set_led(LED5, OFF);
			set_led(LED7, OFF);
			break;

		case T_JUNCTION :
			set_led(LED1, ON);
			set_led(LED3, ON);
			set_led(LED5, ON);
			set_led(LED7, ON);
			command = wait_receive_order(node_type);
			set_led(LED1, OFF);
			set_led(LED3, OFF);
			set_led(LED5, OFF);
			set_led(LED7, OFF);
			break;

		default :
			break;
	}

	execute_sound_command(command);
	general_command(node_type);

}

void general_command(uint8_t node_type){

	switch(node_type){

		case NODE_ERROR :
			stop();
			chprintf((BaseSequentialStream *) &SD3, "NODE TYPE ERROR \r\n");
			break;

		case STRAIGHT_PATH :
			chprintf((BaseSequentialStream *) &SD3, "CORRIDOR, GOING FORWARD \r\n");
			go_forward_regulator();
			break;

		case CORNER_LEFT :
			chprintf((BaseSequentialStream *) &SD3, "CORNER LEFT, TURNING LEFT \r\n");
			turn_left_90();
			while(get_prox(SIDE_LEFT) < THRESHOLD_SIDE_WALL){  //tant que le croisement n'a pas été passé
				go_forward();
			}
			break;

		case CORNER_RIGHT :
			chprintf((BaseSequentialStream *) &SD3, "CORNER RIGHT, TURNING RIGHT \r\n");
			turn_right_90();
			while(get_prox(SIDE_RIGHT) < THRESHOLD_SIDE_WALL){  //tant que le croisement n'a pas été passé
				go_forward();
			}
			break;

		case CUL_DE_SAC :
			chprintf((BaseSequentialStream *) &SD3, "CUL DE SAC, GOING BACK \r\n");
			half_turn();
			break;

		default :
			break;
	}
}

void execute_sound_command(uint8_t command){

	switch(command){

		case STOP :
			chprintf((BaseSequentialStream *) &SD3, "ORDER: STOP \r\n");
			stop();
			break;

		case GO_FORWARD :
			chprintf((BaseSequentialStream *) &SD3, "ORDER: FORWARD \r\n");
			do{  //avance jusqu'à sortir de la jonction
				go_forward();
			}while(get_prox(SIDE_LEFT) < THRESHOLD_SIDE_WALL || get_prox(SIDE_RIGHT) < THRESHOLD_SIDE_WALL);
			stop();
			break;

		case TURN_LEFT :
			chprintf((BaseSequentialStream *) &SD3, "ORDER: LEFT \r\n");
			turn_left_90();
			do{  //avance jusqu'à sortir de la jonction
				go_forward();
			}while(get_prox(SIDE_LEFT) < THRESHOLD_SIDE_WALL);
			stop();
			break;

		case TURN_RIGHT :
			chprintf((BaseSequentialStream *) &SD3, "ORDER: RIGHT \r\n");
			turn_right_90();
			do{  //avance jusqu'à sortir de la jonction
				go_forward();
			}while(get_prox(SIDE_RIGHT) < THRESHOLD_SIDE_WALL);
			stop();
			break;

		case HALF_TURN :
			chprintf((BaseSequentialStream *) &SD3, "ORDER: HALF TURN \r\n");
			half_turn();
			do{  //avance jusqu'à sortir de la jonction
				go_forward();
			}while(get_prox(SIDE_LEFT) < THRESHOLD_SIDE_WALL || get_prox(SIDE_RIGHT) < THRESHOLD_SIDE_WALL);
			stop();
			break;

		default :
			break;
	}
}

void stop(void){

	set_led(LED1, ON);
	set_led(LED3, ON);
	set_led(LED5, ON);
	set_led(LED7, ON);

	left_motor_set_speed(0);
	right_motor_set_speed(0);

	chThdSleepMilliseconds(SLEEP_TIME);

	set_led(LED1, OFF);
	set_led(LED3, OFF);
	set_led(LED5, OFF);
	set_led(LED7, OFF);

}

void turn_right_90(void){

	set_led(LED3, ON);

	int left_motor_pos = 0;

	left_motor_set_pos(0);
	left_motor_set_speed(500);
	right_motor_set_speed(-500);

	while(left_motor_pos < QUARTER_TURN_ABS){   //boucle  tant qu'un quart de tour n'a pas été fait

		left_motor_pos = left_motor_get_pos();
	}
	set_led(LED3, OFF);
	stop();
}

void turn_left_90(void){

	set_led(LED7, ON);

	int right_motor_pos = 0;

	right_motor_set_pos(0);
	left_motor_set_speed(-500);
	right_motor_set_speed(500);

	while(right_motor_pos < QUARTER_TURN_ABS){

		right_motor_pos = right_motor_get_pos();

	}
	set_led(LED7, OFF);
	stop();
}

void go_forward_regulator(void){ //fonction de déplacement en ligne droite avec régulateur proportionnel

	int32_t error_kp = get_prox(SIDE_RIGHT) + get_prox(FRONT_SIDE_RIGHT) - get_prox(SIDE_LEFT) - get_prox(FRONT_SIDE_LEFT);
	set_led(LED1, ON);

		left_motor_set_speed(500 - error_kp*KP);
		right_motor_set_speed(500 + error_kp*KP);

	set_led(LED1, OFF);
}

void go_forward(void){ //déplacement en ligne droite sans régulateur afin de sortir des jonctions
	set_led(LED1, ON);
	left_motor_set_speed(500);
	right_motor_set_speed(500);
	set_led(LED1, OFF);
}

void half_turn(void){

	set_led(LED5, ON);

	int right_motor_pos = 0;

	right_motor_set_pos(0);
	left_motor_set_speed(-500);
	right_motor_set_speed(500);

	while(right_motor_pos < HALF_TURN_ABS){     //doublé pour faire un demi-tour (HALF_TURN était déjà pris)

		right_motor_pos = right_motor_get_pos();

	}
	set_led(LED5, OFF);
	stop();
}

void measure_dist(int32_t dist[NB_CAPTEURS]){

	dist[FRONT_RIGHT] = get_prox(FRONT_RIGHT);
	dist[FRONT_LEFT] = get_prox(FRONT_LEFT);
	dist[FRONT_SIDE_RIGHT] = get_prox(FRONT_SIDE_RIGHT);
	dist[FRONT_SIDE_LEFT] = get_prox(FRONT_SIDE_LEFT);
	dist[SIDE_RIGHT] = get_prox(SIDE_RIGHT);
	dist[SIDE_LEFT] = get_prox(SIDE_LEFT);
	dist[BACK_RIGHT] = get_prox(BACK_RIGHT);
	dist[BACK_LEFT] = get_prox(BACK_LEFT);
}

void print_measures(int32_t path[]){

	chprintf((BaseSequentialStream *) &SD3, "Capteur front left: %d \r\n", path[FRONT_LEFT]);
	chprintf((BaseSequentialStream *) &SD3, "Capteur front right: %d \r\n", path[FRONT_RIGHT]);
	chprintf((BaseSequentialStream *) &SD3, "Capteur front side left: %d \r\n", path[FRONT_SIDE_LEFT]);
	chprintf((BaseSequentialStream *) &SD3, "Capteur front side right: %d \r\n", path[FRONT_SIDE_RIGHT]);
	chprintf((BaseSequentialStream *) &SD3, "Capteur side left: %d \r\n", path[SIDE_LEFT]);
	chprintf((BaseSequentialStream *) &SD3, "Capteur side right: %d \r\n", path[SIDE_RIGHT]);
	chprintf((BaseSequentialStream *) &SD3, "Capteur back left: %d \r\n", path[BACK_LEFT]);
	chprintf((BaseSequentialStream *) &SD3, "Capteur back right: %d \r\n", path[BACK_RIGHT]);
}

