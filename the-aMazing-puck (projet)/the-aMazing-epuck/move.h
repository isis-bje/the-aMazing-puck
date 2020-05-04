
#ifndef MOVE_H_
#define MOVE_H_

enum nodes{NODE_ERROR, CROSSROAD, T_JUNCTION_LEFT, T_JUNCTION_RIGHT, T_JUNCTION,
		   STRAIGHT_PATH, CORNER_LEFT, CORNER_RIGHT, CUL_DE_SAC};

enum commands{STOP, FORWARD, TURN_LEFT, TURN_RIGHT, HALF_TURN};

#define NB_CAPTEURS					8
#define FRONT_RIGHT					0 //IR0
#define FRONT_LEFT					7 //IR7
#define FRONT_SIDE_RIGHT			1 //IR1
#define FRONT_SIDE_LEFT				6 //IR6
#define SIDE_RIGHT					2 //IR2
#define SIDE_LEFT					5 //IR5

#define THRESHOLD_WALL		      	200 //? valeur de seuil pour détecter une ouverture, à modifier expérimentalement
#define SLEEP_TIME				 	1000

#define WHEEL_PERIMETER				13
#define NBSTEPS_ONE_TURN			1000

void move_start(void);
void turn_right(void);
void turn_left(void);
void go_forward(void);
void half_turn(void);

#endif /* MOVE_H_ */
