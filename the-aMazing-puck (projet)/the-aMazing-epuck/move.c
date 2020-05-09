#include "move.h"

#include "ch.h"
#include "hal.h"
#include <chprintf.h>
#include <usbcfg.h>

#include <sensors/proximity.h>
#include <motors.h>
#include <leds.h>
#include <sound.h>

static uint8_t program;

uint8_t junction_detection(int32_t find_path[NB_CAPTEURS]);
void move_command(uint8_t node_type);

void execute_sound_command(uint8_t command);
void automatic_command(uint8_t node_type);
void semiautomatic_command(uint8_t node_type);
void general_command(uint8_t node_type);

void stop(void);
void turn_right_90(void);
void turn_left_90(void);
void go_forward_regulator(void);
void go_forward(void);
void half_turn(void);

void measure_dist_cal(int32_t dist_cal[NB_CAPTEURS]);
void measure_dist(int32_t dist[NB_CAPTEURS]);
void print_calibrated_measures(int32_t path_cal[NB_CAPTEURS]);
void print_measures(int32_t path[NB_CAPTEURS]);
int8_t steps_to_cm(int16_t nb_steps);
int16_t cm_to_steps(int8_t dist_cm);

//Thread that controls the movement of the robot

static THD_WORKING_AREA(waThdMove, 512);
static THD_FUNCTION(ThdMove, arg)
{
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

//-------------------------EXTERNAL FUNCTIONS-------------------------------------------

void move_start(uint8_t mode){

	right_motor_set_speed(0);
	left_motor_set_speed(0);

	program = mode;

	chThdCreateStatic(waThdMove, sizeof(waThdMove), NORMALPRIO, ThdMove, NULL); //NORMALPRIO + 2 ?
}

//-------------------------INTERNAL FUNCTIONS--------------------------------------------

//Detection of a junction
uint8_t junction_detection(int32_t find_path[]){

	uint8_t node_type = 0;

	measure_dist(find_path);   																				// fill an array with proximity sensors values

	if(find_path[SIDE_LEFT] > THRESHOLD_WALL && find_path[SIDE_RIGHT] > THRESHOLD_WALL)						// if no opening neither left nor right
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
	else if(find_path[SIDE_LEFT] < THRESHOLD_WALL || find_path[SIDE_RIGHT] < THRESHOLD_WALL)  				// if opening left or right
	{
		left_motor_set_pos(0);
		right_motor_set_pos(0);

		do{
			go_forward();
			measure_dist(find_path);
		}while((left_motor_get_pos() < MIDDLE && right_motor_get_pos() < MIDDLE) || 						// go to the middle of the junction
			   (find_path[FRONT_LEFT] < THRESHOLD_FRONT && find_path[FRONT_RIGHT] < THRESHOLD_FRONT));  	// if too close to a wall
		stop();

		if(find_path[SIDE_LEFT] < THRESHOLD_WALL && find_path[SIDE_RIGHT] < THRESHOLD_WALL) 				// if opening left and right
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
		else if(find_path[SIDE_LEFT] < THRESHOLD_WALL && find_path[SIDE_RIGHT] > THRESHOLD_WALL) 			// if opening only left
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
		else if(find_path[SIDE_LEFT] > THRESHOLD_WALL && find_path[SIDE_RIGHT] < THRESHOLD_WALL) 			// if opening only right
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

void move_command(uint8_t node_type){

	if(program == AUTO){ //automatic mode

		automatic_command(node_type);
	}
	else if(program == SEMIAUTO){//semi-automatic mode

		semiautomatic_command(node_type);
	}
}

void print_calibrated_measures(int32_t path_cal[]){

	measure_dist_cal(path_cal);

	chprintf((BaseSequentialStream *) &SD3, "Capteur front left calibrated: %d \r\n", path_cal[FRONT_LEFT]);
	chprintf((BaseSequentialStream *) &SD3, "Capteur front right calibrated: %d \r\n", path_cal[FRONT_RIGHT]);
	chprintf((BaseSequentialStream *) &SD3, "Capteur front side left calibrated: %d \r\n", path_cal[FRONT_SIDE_LEFT]);
	chprintf((BaseSequentialStream *) &SD3, "Capteur front side right calibrated: %d \r\n", path_cal[FRONT_SIDE_RIGHT]);
	chprintf((BaseSequentialStream *) &SD3, "Capteur side left calibrated: %d \r\n", path_cal[SIDE_LEFT]);
	chprintf((BaseSequentialStream *) &SD3, "Capteur side right calibrated: %d \r\n", path_cal[SIDE_RIGHT]);
	chprintf((BaseSequentialStream *) &SD3, "Capteur back left calibrated: %d \r\n", path_cal[BACK_LEFT]);
	chprintf((BaseSequentialStream *) &SD3, "Capteur back right calibrated: %d \r\n", path_cal[BACK_RIGHT]);

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

void measure_dist_cal(int32_t dist_cal[NB_CAPTEURS]){

	dist_cal[FRONT_RIGHT] = get_calibrated_prox(FRONT_RIGHT);
	dist_cal[FRONT_LEFT] = get_calibrated_prox(FRONT_LEFT);
	dist_cal[FRONT_SIDE_RIGHT] = get_calibrated_prox(FRONT_SIDE_RIGHT);
	dist_cal[FRONT_SIDE_LEFT] = get_calibrated_prox(FRONT_SIDE_LEFT);
	dist_cal[SIDE_RIGHT] = get_calibrated_prox(SIDE_RIGHT);
	dist_cal[SIDE_LEFT] = get_calibrated_prox(SIDE_LEFT);
	dist_cal[BACK_RIGHT] = get_calibrated_prox(BACK_RIGHT);
	dist_cal[BACK_LEFT] = get_calibrated_prox(BACK_LEFT);
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

int8_t steps_to_cm(int16_t nb_steps){ // from -100 - 100 cm to -32000 - 32000 steps

	return nb_steps*WHEEL_PERIMETER/NBSTEPS_ONE_TURN;
}

int16_t cm_to_steps(int8_t dist_cm){ // from -100 - 100 cm to -32000 - 32000 steps

	return dist_cm*NBSTEPS_ONE_TURN/WHEEL_PERIMETER;
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

void execute_sound_command(uint8_t command){

	switch(command){

		case STOP :
			stop();
			break;

		case GO_FORWARD :
			do{  //avance jusqu'à sortir de la jonction
				go_forward();
			}while(get_prox(SIDE_LEFT) < THRESHOLD_WALL || get_prox(SIDE_RIGHT) < THRESHOLD_WALL);
			stop();
			break;

		case TURN_LEFT :
			turn_left_90();
			do{  //avance jusqu'à sortir de la jonction
				go_forward();
			}while(get_prox(SIDE_LEFT) < THRESHOLD_WALL);
			stop();
			break;

		case TURN_RIGHT :
			turn_right_90();
			do{  //avance jusqu'à sortir de la jonction
				go_forward();
			}while(get_prox(SIDE_RIGHT) < THRESHOLD_WALL);
			stop();
			break;

		case HALF_TURN :
			half_turn();
			do{  //avance jusqu'à sortir de la jonction
				go_forward();
			}while(get_prox(SIDE_LEFT) < THRESHOLD_WALL || get_prox(SIDE_RIGHT) < THRESHOLD_WALL);
			stop();
			break;

		default :
			break;
	}
}

void automatic_command(uint8_t node_type){

	switch(node_type){

		case CROSSROAD :
			turn_right_90();
			while(get_prox(SIDE_LEFT) < THRESHOLD_WALL && get_prox(SIDE_RIGHT) < THRESHOLD_WALL){  //tant que le croisement n'a pas été passé
				go_forward();
			}
			break;

		case T_JUNCTION_LEFT :
			go_forward();
			break;

		case T_JUNCTION_RIGHT :
			turn_right_90();
			while(get_prox(SIDE_LEFT) < THRESHOLD_WALL && get_prox(SIDE_RIGHT) < THRESHOLD_WALL){  //tant que le croisement n'a pas été passé
				go_forward();
			}
			break;

		case T_JUNCTION :
			turn_right_90();
			while(get_prox(SIDE_RIGHT) < THRESHOLD_WALL){  //tant que le croisement n'a pas été passé
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
			chprintf((BaseSequentialStream *) &SD3, "error \r\n");
			break;

		case STRAIGHT_PATH :
			go_forward_regulator();
			break;

		case CORNER_LEFT :
			turn_left_90();
			while(get_prox(SIDE_LEFT) < THRESHOLD_WALL){  //tant que le croisement n'a pas été passé
				go_forward();
			}
			break;

		case CORNER_RIGHT :
			turn_right_90();
			while(get_prox(SIDE_RIGHT) < THRESHOLD_WALL){  //tant que le croisement n'a pas été passé
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




