
#ifndef MOVE_H_
#define MOVE_H_

enum nodes{NODE_ERROR, CROSSROAD, T_JUNCTION_LEFT, T_JUNCTION_RIGHT, T_JUNCTION,
		   STRAIGHT_PATH, CORNER_LEFT, CORNER_RIGHT, CUL_DE_SAC};

enum commands{STOP, FORWARD, TURN_LEFT, TURN_RIGHT, HALF_TURN};

enum flip{OFF, ON};

#define NB_CAPTEURS					8
#define FRONT_RIGHT					0 //IR0
#define FRONT_LEFT					7 //IR7
#define FRONT_SIDE_RIGHT			1 //IR1
#define FRONT_SIDE_LEFT				6 //IR6
#define SIDE_RIGHT					2 //IR2
#define SIDE_LEFT					5 //IR5

#define THRESHOLD_WALL		      	250 // valeur de seuil pour détecter une ouverture, ajustée expérimentalement
#define THRESHOLD_FRONT             300 // valeur de seuil pour détecter un mur frontal, ajustée expérimentalement

#define WHEEL_PERIMETER				13
#define NBSTEPS_ONE_TURN			1000
#define QUARTER_TURN_ABS       		327 //423 selon le calcul théorique, ajusté expérimentalement pour 1/4 tour
#define HALF_TURN_ABS				654 //846 selon le calcul théorique, ajusté expérimentalement pour 1/2 tour
#define KP							0 //? valeur choisie au hasard (en l'attente de tests)

#define SLEEP_TIME				 	50

void move_start(void);
void stop(void);
void turn_right_90(void);
void turn_left_90(void);
void go_forward_regulator(void);
void go_forward(void);
void half_turn(void);

#endif /* MOVE_H_ */
