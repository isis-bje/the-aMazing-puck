#ifndef SOUND_H_
#define SOUND_H_

#define FFT_SIZE 	        1024

enum order{STOP, GO_FORWARD, TURN_LEFT, TURN_RIGHT, HALF_TURN};

//put the invoking thread into sleep until it can process the audio datas
uint8_t wait_receive_order(void);

void processSound(int16_t *data, uint16_t num_samples);

#endif
