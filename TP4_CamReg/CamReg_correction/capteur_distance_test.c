#include "ch.h"
#include "hal.h"
#include <chprintf.h>
#include <usbcfg.h>
#include <capteur_distance_test.h>

#include <sensors/proximity.h>

// - IR0 (front-right) + IR4 (back-left)
// - IR1 (front-right-45deg) + IR5 (left)
// - IR2 (right) + IR6 (front-left-45deg)
// - IR3 (back-right) + IR7 (front-left)


void measureDists(int32_t dists[NB_CAPTEURS])
{
	dists[FRONT_RIGHT] = get_calibrated_prox(FRONT_RIGHT);
	dists[FRONT_LEFT] = get_calibrated_prox(FRONT_LEFT);
	dists[FRONT_SIDE_RIGHT] = get_calibrated_prox(FRONT_SIDE_RIGHT);
	dists[FRONT_SIDE_LEFT] = get_calibrated_prox(FRONT_SIDE_LEFT);
	dists[SIDE_RIGHT] = get_calibrated_prox(SIDE_RIGHT);
	dists[SIDE_LEFT] = get_calibrated_prox(SIDE_LEFT);
}
