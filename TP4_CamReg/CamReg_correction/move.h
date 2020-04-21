/*
 * move.h
 *
 *  Created on: 15 Apr 2020
 *      Author: Julien MOREL et Isis BOU JAOUDE
 */

#ifndef MOVE_H_
#define MOVE_H_

enum nodes{NODE_ERROR, CROSSROAD, T_JUNCTION_LEFT, T_JUNCTION_RIGHT, T_JUNCTION,
		   STRAIGHT_PATH, CORNER_LEFT, CORNER_RIGHT, CUL_DE_SAC};

#define NB_CAPTEURS					8
#define FRONT_RIGHT					0 //IR0
#define FRONT_LEFT					7 //IR7
#define FRONT_SIDE_RIGHT			1 //IR1
#define FRONT_SIDE_LEFT				6 //IR6
#define SIDE_RIGHT					2 //IR2
#define SIDE_LEFT					5 //IR5

#define THRESHOLD_WALL		      200 //? valeur de seuil pour détecter une ouverture, à modifier expérimentalement
#define SLEEP_TIME

void move_start(void);

#endif /* MOVE_H_ */
