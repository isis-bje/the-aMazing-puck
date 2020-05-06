#ifndef SOUND_H_
#define SOUND_H_

#define FFT_SIZE 	1024

enum commands{STOP, FORWARD, TURN_LEFT, TURN_RIGHT, HALF_TURN};  //  /!\double de l'énum en move.h, pour les ordres selon la fréquence

typedef enum {
	//2 times FFT_SIZE because these arrays contain complex numbers (real + imaginary)
	LEFT_CMPLX_INPUT = 0,
	RIGHT_CMPLX_INPUT,
	FRONT_CMPLX_INPUT,
	BACK_CMPLX_INPUT,
	//Arrays containing the computed magnitude of the complex numbers
	LEFT_OUTPUT,
	RIGHT_OUTPUT,
	FRONT_OUTPUT,
	BACK_OUTPUT
} BUFFER_NAME_t;


void processSound(int16_t *data, uint16_t num_samples);

/*
*	put the invoking thread into sleep until it can process the audio datas
*/
void wait_send_to_computer(void);

/*
*	Returns the pointer to the BUFFER_NAME_t buffer asked
*/
float* get_audio_buffer_ptr(BUFFER_NAME_t name);

void sound_remote(float* data);

void doFFT_optimized(uint16_t size, float* complex_buffer);

uint8_t get_sound_order(void);


#endif
