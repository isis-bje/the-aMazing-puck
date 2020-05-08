#ifndef SOUND_H_
#define SOUND_H_

#include "ch.h"
#include "hal.h"

#define FFT_SIZE 	        1024

enum nodes{NODE_ERROR, CROSSROAD, T_JUNCTION_LEFT, T_JUNCTION_RIGHT, T_JUNCTION,
		   STRAIGHT_PATH, CORNER_LEFT, CORNER_RIGHT, CUL_DE_SAC};

enum order{STOP = 1, GO_FORWARD, TURN_LEFT, TURN_RIGHT, HALF_TURN, RETRY};

//put the invoking thread into sleep until it can process the audio datas
uint8_t wait_receive_order(uint8_t node_type);

//callback function
void processSound(int16_t *data, uint16_t num_samples);

#endif
