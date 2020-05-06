
#ifndef MOVE_H_
#define MOVE_H_

enum nodes{NODE_ERROR, CROSSROAD, T_JUNCTION_LEFT, T_JUNCTION_RIGHT, T_JUNCTION,
		   STRAIGHT_PATH, CORNER_LEFT, CORNER_RIGHT, CUL_DE_SAC};

//enum commands{STOP, FORWARD, TURN_LEFT, TURN_RIGHT, HALF_TURN};  //déplacé dans sound.h

enum flip{OFF, ON};

enum capteurs{FRONT_RIGHT, FRONT_SIDE_RIGHT, SIDE_RIGHT, BACK_RIGHT, BACK_LEFT, SIDE_LEFT, FRONT_SIDE_LEFT, FRONT_LEFT, NB_CAPTEURS};

#define THRESHOLD_WALL		      	450 // valeur de seuil pour détecter une ouverture, ajustée expérimentalement
#define THRESHOLD_FRONT             500 // valeur de seuil pour détecter un mur frontal, ajustée expérimentalement

#define THRESHOLD_BACK				400 //  /!\  valeur à ajuster
#define KP							0.1 //  /!\  valeur à ajuster

#define WHEEL_PERIMETER				13
#define NBSTEPS_ONE_TURN			1000
#define QUARTER_TURN_ABS       		327 //423 selon le calcul théorique, ajusté expérimentalement pour 1/4 tour
#define HALF_TURN_ABS				654 //846 selon le calcul théorique, ajusté expérimentalement pour 1/2 tour

#define SLEEP_TIME				 	50

void move_start(void);
void stop(void);
void turn_right_90(void);
void turn_left_90(void);
void go_forward_regulator(void);
void go_forward(void);
void half_turn(void);

#endif /* MOVE_H_ */
