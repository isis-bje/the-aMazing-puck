#include "move.h"

#include "ch.h"
#include "hal.h"
#include <chprintf.h>
#include <usbcfg.h>
#include <sensors/proximity.h>

int junction_detection(int32_t find_path);
void measure_dist_cal(int32_t dist_cal[NB_CAPTEURS]);
void measure_dist(int32_t dist[NB_CAPTEURS]);

static THD_WORKING_AREA(waThdMove, 128);
static THD_FUNCTION(ThdMove, arg)
{
    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    uint8_t node_type = 0;
	int32_t path[NB_CAPTEURS] = {0, 0, 0, 0, 0, 0, 0, 0};

	while(1){
		junction_detection(path);
	}

	chThdSleepMilliseconds(SLEEP_TIME);

}


//détection des murs à proximité, remplissage tableau (pas de boucle "for" car les capteurs ne sont pas tous utilisés -> 0,1,2,5,6,7
//measureDists(wall_close);
//chprintf((BaseSequentialStream *) &SD3, "Capteur front left calibrated: %d \n", wall_close[FRONT_LEFT]);
//chprintf((BaseSequentialStream *) &SD3, "Capteur front right calibrated: %d \n", wall_close[FRONT_RIGHT]);
//chprintf((BaseSequentialStream *) &SD3, "Capteur front side left calibrated: %d \n", wall_close[FRONT_SIDE_LEFT]);
//chprintf((BaseSequentialStream *) &SD3, "Capteur front side right calibrated: %d \n", wall_close[FRONT_SIDE_RIGHT]);
//chprintf((BaseSequentialStream *) &SD3, "Capteur side left calibrated: %d \n", wall_close[SIDE_LEFT]);
//chprintf((BaseSequentialStream *) &SD3, "Capteur side right calibrated: %d \n", wall_close[SIDE_RIGHT]);

//détection des ouvertures à proximité, remplissage tableau, algorithme choix du chemin
//findPath(path);
//chprintf((BaseSequentialStream *) &SD3, "Capteur front left: %d \n", path[FRONT_LEFT]);
//chprintf((BaseSequentialStream *) &SD3, "Capteur front right: %d \n", path[FRONT_RIGHT]);
//chprintf((BaseSequentialStream *) &SD3, "Capteur front side left: %d \n", path[FRONT_SIDE_LEFT]);
//chprintf((BaseSequentialStream *) &SD3, "Capteur front side right: %d \n", path[FRONT_SIDE_RIGHT]);
//chprintf((BaseSequentialStream *) &SD3, "Capteur side left: %d \n", path[SIDE_LEFT]);
//chprintf((BaseSequentialStream *) &SD3, "Capteur side right: %d \n", path[SIDE_RIGHT]);

void move_start(void){

	chThdCreateStatic(waThdMove, sizeof(waThdMove), NORMALPRIO, ThdMove, NULL);
}


//algorithme de détection du type de jonction
int junction_detection(int32_t find_path){

	measure_dist(find_path);

	if(find_path[FRONT_LEFT] < THRESHOLD_WALL && find_path[FRONT_RIGHT] < THRESHOLD_WALL) //si passage devant ouvert
	{
		if(find_path[SIDE_LEFT] < THRESHOLD_WALL && find_path[SIDE_RIGHT] < THRESHOLD_WALL) //si passage à gauche et à droite
		{
			node_type = CROSSROAD;
		}
		else if(find_path[SIDE_LEFT] < THRESHOLD_WALL) //si passage à gauche
		{
			node_type = T_JUNCTION_LEFT;
		}
		else if(find_path[SIDE_RIGHT] < THRESHOLD_WALL) //si passage à droite
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
		if(find_path[SIDE_LEFT] < THRESHOLD_WALL && find_path[SIDE_RIGHT] < THRESHOLD_WALL) //si passage à gauche et à droite
		{
			node_type = T_JUNCTION;
		}
		else if(find_path[SIDE_LEFT] < THRESHOLD_WALL) //si passage à gauche
		{
			node_type = CORNER_LEFT;
		}
		else if(find_path[SIDE_RIGHT] < THRESHOLD_WALL) //si passage à droite
		{
			node_type = CORNER_RIGHT;
		}
		else //aucun passage
		{
			node_type = CUL_DE_SAC;
		}
	}
	chprintf((BaseSequentialStream *) &SD3, "%d", node_type);

	return node_type;
}

void measure_dist_cal(int32_t dist_cal[NB_CAPTEURS])
{
	dist_cal[FRONT_RIGHT] = get_calibrated_prox(FRONT_RIGHT);
	dist_cal[FRONT_LEFT] = get_calibrated_prox(FRONT_LEFT);
	dist_cal[FRONT_SIDE_RIGHT] = get_calibrated_prox(FRONT_SIDE_RIGHT);
	dist_cal[FRONT_SIDE_LEFT] = get_calibrated_prox(FRONT_SIDE_LEFT);
	dist_cal[SIDE_RIGHT] = get_calibrated_prox(SIDE_RIGHT);
	dist_cal[SIDE_LEFT] = get_calibrated_prox(SIDE_LEFT);
}

void measure_dist(int32_t dist[NB_CAPTEURS])
{
	dist[FRONT_RIGHT] = get_prox(FRONT_RIGHT);
	dist[FRONT_LEFT] = get_prox(FRONT_LEFT);
	dist[FRONT_SIDE_RIGHT] = get_prox(FRONT_SIDE_RIGHT);
	dist[FRONT_SIDE_LEFT] = get_prox(FRONT_SIDE_LEFT);
	dist[SIDE_RIGHT] = get_prox(SIDE_RIGHT);
	dist[SIDE_LEFT] = get_prox(SIDE_LEFT);
}
