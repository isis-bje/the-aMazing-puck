
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

#define SPEED                       500
#define WHEEL_PERIMETER				13
#define NBSTEPS_ONE_TURN			1000  // theoretical value
#define MIDDLE_JUNCTION				320   // adjusted number of steps to get to the middle of a junction
#define QUARTER_TURN_ABS       		327   // adjusted number of steps to do a quarter turn, 423 is the theoretical value
#define HALF_TURN_ABS				654   // adjusted number of steps to do a half turn, 846 is the theoretical value
#define KP		                    0.012 // adjusted value of the proportional regulator

//-----------------------------Static Variables-----------------------------

static uint8_t program;

//---------------------------Internal Declarations---------------------------

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
//void print_measures(int32_t path[NB_CAPTEURS]);

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

//-----------------------------External Functions-----------------------------

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

//-----------------------------Internal Functions-----------------------------

/* Purpose : Launches the correct execution function according to the mode the
 * 			 e-puck is in.
 *
 * Parameters :
 *
 * (in)  uint8_t node_type	 Indicates the type of junction the e-puck is in
 *
 */

void move_command(uint8_t node_type){

	if(program == AUTO){ 					//automatic mode

		automatic_command(node_type);
	}
	else if(program == SEMIAUTO){ 			//semi-automatic mode

		semiautomatic_command(node_type);
	}
}

/* Purpose : Compares the values returned by the proximity sensors with
 * 			 thresholds in order to determine the junction the e-puck is in.
 *
 * Parameters :
 *
 * (in)  in32_t find_path[]    Array with the values measured by the proximity
 * 							   sensors.
 *
 * (out) uint8_t node_type	   Indicates the type of junction the e-puck is in
 *
 */
uint8_t junction_detection(int32_t find_path[]){

	uint8_t node_type = 0;

	measure_dist(find_path);

	// there is no opening neither left nor right
	if(find_path[SIDE_LEFT] > THRESHOLD_SIDE_WALL &&
	   find_path[SIDE_RIGHT] > THRESHOLD_SIDE_WALL){

		// an opening ahead is found
		if(find_path[FRONT_LEFT] < THRESHOLD_FRONT &&
		   find_path[FRONT_RIGHT] < THRESHOLD_FRONT){

			node_type = STRAIGHT_PATH;
		}
		// no opening ahead
		else{
			node_type = CUL_DE_SAC;
		}
	}
	// an opening left or right is found
	else if(find_path[SIDE_LEFT] < THRESHOLD_SIDE_WALL ||
			find_path[SIDE_RIGHT] < THRESHOLD_SIDE_WALL){

		left_motor_set_pos(0);
		right_motor_set_pos(0);
		// set position counters to 0
		int left_pos = 0;
		int right_pos = 0;

		// loop used to place the e-puck in the middle of the junction
		do{

			go_forward();
			measure_dist(find_path);
			left_pos = left_motor_get_pos();
			right_pos = right_motor_get_pos();

		}while((left_pos < MIDDLE_JUNCTION && right_pos  < MIDDLE_JUNCTION) &&
			   (find_path[FRONT_LEFT] < THRESHOLD_FRONT && find_path[FRONT_RIGHT] < THRESHOLD_FRONT));
		// either the e-puck has moved enough steps or a wall is too close
		stop();

		// an opening left and right are found
		if(find_path[SIDE_LEFT] < THRESHOLD_SIDE_WALL &&
		   find_path[SIDE_RIGHT] < THRESHOLD_SIDE_WALL){

			// an opening ahead is found
			if(find_path[FRONT_LEFT] < THRESHOLD_FRONT &&
			   find_path[FRONT_RIGHT] < THRESHOLD_FRONT){
				node_type = CROSSROAD;
			}
			//there is no opening ahead
			else{
				node_type = T_JUNCTION;
			}
		}

		// there is an opening only on the left side
		else if(find_path[SIDE_LEFT] < THRESHOLD_SIDE_WALL &&
				find_path[SIDE_RIGHT] > THRESHOLD_SIDE_WALL){
			//there is an opening ahead
			if(find_path[FRONT_LEFT] < THRESHOLD_FRONT &&
			   find_path[FRONT_RIGHT] < THRESHOLD_FRONT){
				node_type = T_JUNCTION_LEFT;
			}
			//no opening ahead
			else{
				node_type = CORNER_LEFT;
			}
		}
		// there is an opening only on the right side
		else if(find_path[SIDE_LEFT] > THRESHOLD_SIDE_WALL &&
				find_path[SIDE_RIGHT] < THRESHOLD_SIDE_WALL){
			//there is an opening ahead
			if(find_path[FRONT_LEFT] < THRESHOLD_FRONT &&
			   find_path[FRONT_RIGHT] < THRESHOLD_FRONT){
				node_type = T_JUNCTION_RIGHT;
			}
			//no opening ahead
			else{
				node_type = CORNER_RIGHT;
			}
		}
	}

	return node_type;
}

/* Purpose : Decides what movement needs to be done in automatic mode
 * 			 according to the junction the e-puck is in.
 *
 * Parameters :
 *
 * (in)  uint8_t node_type    Indicates the type of junction the e-puck is in
 *
 */
void automatic_command(uint8_t node_type){

	switch(node_type){

		case CROSSROAD :
			turn_right_90();
			//as long as the e-puck didn't move past the junction
			while(get_prox(SIDE_LEFT) < THRESHOLD_SIDE_WALL &&
				  get_prox(SIDE_RIGHT) < THRESHOLD_SIDE_WALL){
				go_forward();
			}
			break;

		case T_JUNCTION_LEFT :
			go_forward();
			break;

		case T_JUNCTION_RIGHT :
			turn_right_90();
			//as long as the e-puck didn't move past the junction
			while(get_prox(SIDE_LEFT) < THRESHOLD_SIDE_WALL &&
				  get_prox(SIDE_RIGHT) < THRESHOLD_SIDE_WALL){
				go_forward();
			}
			break;

		case T_JUNCTION :
			turn_right_90();
			//as long as the e-puck didn't move past the junction
			while(get_prox(SIDE_RIGHT) < THRESHOLD_SIDE_WALL){
				go_forward();
			}
			break;

		default :
			break;
	}

	general_command(node_type);
}

/* Purpose : Retrieves the command given by sound when the e-puck is in a junction
 * 			 with multiple paths and calls the execution function.
 *
 * Parameters :
 *
 * (in)  uint8_t node_type   Indicates the type of junction the e-puck is in
 *
 */
void semiautomatic_command(uint8_t node_type){

	uint8_t command = 0;

	switch(node_type){

		case CROSSROAD :
			command = wait_receive_order(node_type);
			break;

		case T_JUNCTION_LEFT :
			command = wait_receive_order(node_type);
			break;

		case T_JUNCTION_RIGHT :
			command = wait_receive_order(node_type);
			break;

		case T_JUNCTION :
			command = wait_receive_order(node_type);
			break;

		default :
			break;
	}

	execute_sound_command(command);

	general_command(node_type);

}

/* Purpose : Decides what movement needs to be done if the e-puck is in a junction
 * 			 with only one possible path.
 *
 * Parameters :
 *
 * (in)  uint8_t node_type   Indicates the type of junction the e-puck is in
 *
 */
void general_command(uint8_t node_type){

	switch(node_type){

		case NODE_ERROR :
			stop();
			chprintf((BaseSequentialStream *) &SD3, "NODE TYPE ERROR \r\n");
			break;

		case STRAIGHT_PATH :
			go_forward_regulator();
			break;

		case CORNER_LEFT :
			turn_left_90();
			//as long as the e-puck didn't move past the junction
			while(get_prox(SIDE_LEFT) < THRESHOLD_SIDE_WALL){
				go_forward();
			}
			break;

		case CORNER_RIGHT :
			turn_right_90();
			//as long as the e-puck didn't move past the junction
			while(get_prox(SIDE_RIGHT) < THRESHOLD_SIDE_WALL){
				go_forward();
			}
			break;

		case CUL_DE_SAC :
			half_turn();
			break;

		default :
			break;
	}
}

/* Purpose : Calls the movement functions according
 * 			 to the sound order received
 *
 * Parameters :
 *
 * (in)  uint8_t command   The sound order received
 *
 */
void execute_sound_command(uint8_t command){

	switch(command){

		case STOP :
			stop();
			break;

		case GO_FORWARD :
			//as long as the e-puck didn't move past the junction
			do{
				go_forward();
			}while(get_prox(SIDE_LEFT) < THRESHOLD_SIDE_WALL ||
				   get_prox(SIDE_RIGHT) < THRESHOLD_SIDE_WALL);
			stop();
			break;

		case TURN_LEFT :
			turn_left_90();
			//as long as the e-puck didn't move past the junction
			do{
				go_forward();
			}while(get_prox(SIDE_LEFT) < THRESHOLD_SIDE_WALL);
			stop();
			break;

		case TURN_RIGHT :
			turn_right_90();
			//as long as the e-puck didn't move past the junction
			do{
				go_forward();
			}while(get_prox(SIDE_RIGHT) < THRESHOLD_SIDE_WALL);
			stop();
			break;

		case HALF_TURN :
			half_turn();
			//as long as the e-puck didn't move past the junction
			do{
				go_forward();
			}while(get_prox(SIDE_LEFT) < THRESHOLD_SIDE_WALL ||
				   get_prox(SIDE_RIGHT) < THRESHOLD_SIDE_WALL);
			stop();
			break;

		default :
			break;
	}
}

/* Purpose : Movement function that stops the e-puck
 */
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

/* Purpose : Movement function that rotates the e-puck 90 degrees
 * 			 on its right.
 */
void turn_right_90(void){

	int left_motor_pos = 0;

	set_led(LED3, ON);

	left_motor_set_pos(0);
	left_motor_set_speed(SPEED);
	right_motor_set_speed(-SPEED);

	//as long as the e-puck didn't rotate a quarter turn
	while(left_motor_pos < QUARTER_TURN_ABS){
		left_motor_pos = left_motor_get_pos();
	}

	set_led(LED3, OFF);
	stop();
}

/* Purpose : Movement function that rotates the e-puck 90 degrees
 *           on its left.
 */
void turn_left_90(void){

	int right_motor_pos = 0;

	set_led(LED7, ON);

	right_motor_set_pos(0);
	left_motor_set_speed(-SPEED);
	right_motor_set_speed(SPEED);

	//as long as we didn't rotate a quarter turn
	while(right_motor_pos < QUARTER_TURN_ABS){
		right_motor_pos = right_motor_get_pos();
	}

	set_led(LED7, OFF);
	stop();
}

/* Purpose : Movement function that moves the e-puck forwards
 * 			 with a proportional regulator (in a corridor)
 *
 */
void go_forward_regulator(void){

	int32_t error_kp = get_prox(SIDE_RIGHT) + get_prox(FRONT_SIDE_RIGHT)
			         - get_prox(SIDE_LEFT) - get_prox(FRONT_SIDE_LEFT);

	set_led(LED1, ON);

	left_motor_set_speed(SPEED - error_kp*KP);
	right_motor_set_speed(SPEED + error_kp*KP);

	set_led(LED1, OFF);
}

/* Purpose : Movement function that moves the e-puck forwards
 * 			 without a proportional regulator (to go to the
 * 			 middle or out of a junction)
 *
 */
void go_forward(void){

	set_led(LED1, ON);

	left_motor_set_speed(SPEED);
	right_motor_set_speed(SPEED);

	set_led(LED1, OFF);
}

/* Purpose : Movement function that rotates the e-puck 180 degrees
 * 			 on its left
 *
 */
void half_turn(void){

	int right_motor_pos = 0;

	set_led(LED5, ON);

	right_motor_set_pos(0);
	left_motor_set_speed(-SPEED);
	right_motor_set_speed(SPEED);

	//until a half turn is done
	while(right_motor_pos < HALF_TURN_ABS){

		right_motor_pos = right_motor_get_pos();
	}

	set_led(LED5, OFF);

	stop();
}

/* Purpose : Fills an array with the measures of the proximity sensors
 *
 * Parameters :
 *
 * (in) int32_t dist[]   The array to fill with the measured values
 *
 */
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

/* Purpose : Prints the measures of the proximity sensors
 * 			 Used to monitor the measures in order to determine the thresholds needed.
 *
 * Parameters :
 *
 * (in) int32_t path[]   The array containing the values to print
 *
 */
/*void print_measures(int32_t path[]){

	chprintf((BaseSequentialStream *) &SD3, "Capteur front left: %d \r\n", path[FRONT_LEFT]);
	chprintf((BaseSequentialStream *) &SD3, "Capteur front right: %d \r\n", path[FRONT_RIGHT]);
	chprintf((BaseSequentialStream *) &SD3, "Capteur front side left: %d \r\n", path[FRONT_SIDE_LEFT]);
	chprintf((BaseSequentialStream *) &SD3, "Capteur front side right: %d \r\n", path[FRONT_SIDE_RIGHT]);
	chprintf((BaseSequentialStream *) &SD3, "Capteur side left: %d \r\n", path[SIDE_LEFT]);
	chprintf((BaseSequentialStream *) &SD3, "Capteur side right: %d \r\n", path[SIDE_RIGHT]);
	chprintf((BaseSequentialStream *) &SD3, "Capteur back left: %d \r\n", path[BACK_LEFT]);
	chprintf((BaseSequentialStream *) &SD3, "Capteur back right: %d \r\n", path[BACK_RIGHT]);
}
*/

