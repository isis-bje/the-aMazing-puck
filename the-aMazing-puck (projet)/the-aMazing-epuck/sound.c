
#include "ch.h"
#include "hal.h"
#include <main.h>

#include <usbcfg.h>
#include <chprintf.h>

#include <audio/microphone.h>
#include <sound.h>

#include <arm_math.h>
#include <arm_const_structs.h>

//---------------------------------CONSTANTS---------------------------------

#define FFT_SIZE 	         1024
#define MIN_VALUE_THRESHOLD	 10000

#define MIN_FREQ		     10	//156Hz no analysis before this index to limit the use of resources
#define FREQ_FORWARD	     16	//250Hz
#define FREQ_LEFT		     19	//296Hz
#define FREQ_RIGHT		     23	//359Hz
#define FREQ_HTURN   	     26	//406Hz
#define FREQ_STOP            30 //469Hz
#define MAX_FREQ		     35	//547Hz no analysis after this index to limit the use of resources

#define FREQ_FORWARD_L		(FREQ_FORWARD-1)
#define FREQ_FORWARD_H		(FREQ_FORWARD+1)
#define FREQ_LEFT_L			(FREQ_LEFT-1)
#define FREQ_LEFT_H			(FREQ_LEFT+1)
#define FREQ_RIGHT_L		(FREQ_RIGHT-1)
#define FREQ_RIGHT_H		(FREQ_RIGHT+1)
#define FREQ_HTURN_L		(FREQ_HTURN-1)
#define FREQ_HTURN_H		(FREQ_HTURN+1)
#define FREQ_STOP_L			(FREQ_STOP-1)
#define FREQ_STOP_H			(FREQ_STOP+1)

//------------------------Static Variables and Arrays------------------------

//semaphore
static BSEMAPHORE_DECL(get_order_sem, TRUE);

//2 times FFT_SIZE because these arrays contain complex numbers (real + imaginary)
static float micLeft_cmplx_input[2 * FFT_SIZE];
static float micRight_cmplx_input[2 * FFT_SIZE];
static float micFront_cmplx_input[2 * FFT_SIZE];
static float micBack_cmplx_input[2 * FFT_SIZE];

//Arrays containing the computed magnitude of the complex numbers
static float micLeft_output[FFT_SIZE];
static float micRight_output[FFT_SIZE];
static float micFront_output[FFT_SIZE];
static float micBack_output[FFT_SIZE];

static uint8_t command;

//---------------------------Internal Declarations---------------------------

void doFFT_optimized(uint16_t size, float* complex_buffer);
void sound_remote(float* data);

//-----------------------------External Functions-----------------------------

/* Purpose : Asks to re-issue a sound order until it is possible to execute it.
 *
 * Parameters :
 *
 * (in)  uint8_t node_type	  Indicates the type of junction the e-puck is in.
 *
 * (out) uint8_t command      Gives the sound command received.
 */
uint8_t wait_receive_order(uint8_t node_type){

	do{
		chBSemWait(&get_order_sem);
	}
	while((node_type == T_JUNCTION_LEFT && command == TURN_RIGHT) ||
		  (node_type == T_JUNCTION_RIGHT && command == TURN_LEFT) ||
		  (node_type == T_JUNCTION && command == GO_FORWARD) ||
		  (command == RETRY));

	return command;
}

/* Purpose : Callback used when the demodulation of the four microphones
 * 			 is done (the samples are ready).
 *
 * Parameters :
 *
 * (in)  int16_t *data		   Points to the buffer containing 4 times 160
 * 							   samples taken every 10 ms (16kHz).
 *	 	 	 	 	 	 	   The samples are sorted by microphones (4).
 *
 * (in)  uint16_t num_samples  Number of samples taken.
 *
 */
void processSound(int16_t *data, uint16_t num_samples){

	//Part 1 : The sample buffers are filled.

	static uint16_t nb_samples = 0;

	//loop used to fill the buffers
	for(uint16_t i = 0 ; i < num_samples ; i+=4){

		micRight_cmplx_input[nb_samples] = (float)data[i + MIC_RIGHT];
		micLeft_cmplx_input[nb_samples] = (float)data[i + MIC_LEFT];
		micBack_cmplx_input[nb_samples] = (float)data[i + MIC_BACK];
		micFront_cmplx_input[nb_samples] = (float)data[i + MIC_FRONT];

		nb_samples++;

		micRight_cmplx_input[nb_samples] = 0;
		micLeft_cmplx_input[nb_samples] = 0;
		micBack_cmplx_input[nb_samples] = 0;
		micFront_cmplx_input[nb_samples] = 0;

		nb_samples++;

		//stop condition : the buffer is full
		if(nb_samples >= (2 * FFT_SIZE)){
			break;
		}
	}

	//Part 2 : The FFT and the magnitude of the complex numbers are computed.

	if(nb_samples >= (2 * FFT_SIZE)){

		//	FFT computation
		doFFT_optimized(FFT_SIZE, micRight_cmplx_input);
		doFFT_optimized(FFT_SIZE, micLeft_cmplx_input);
		doFFT_optimized(FFT_SIZE, micFront_cmplx_input);
		doFFT_optimized(FFT_SIZE, micBack_cmplx_input);

		//	Magnitude computation
		arm_cmplx_mag_f32(micRight_cmplx_input, micRight_output, FFT_SIZE);
		arm_cmplx_mag_f32(micLeft_cmplx_input, micLeft_output, FFT_SIZE);
		arm_cmplx_mag_f32(micFront_cmplx_input, micFront_output, FFT_SIZE);
		arm_cmplx_mag_f32(micBack_cmplx_input, micBack_output, FFT_SIZE);

		nb_samples = 0;

		sound_remote(micLeft_output);
	}
}

//-----------------------------Internal Functions-----------------------------

/* Purpose : Detect the highest value in a buffer and attribute a movement
 * 			 command depending on it
 *
 * Parameters :
 *
 * (in)  float* data   Points to the buffer containing 4 times 160 samples
 * 					   taken every 10 ms (16kHz).
 * 				       The samples are sorted by microphones (4).
 *
 */
void sound_remote(float* data){

	float max_norm = MIN_VALUE_THRESHOLD;
	int16_t max_norm_index = -1;

	//Search for the highest peak
	for(uint16_t i = MIN_FREQ ; i <= MAX_FREQ ; i++){

		if(data[i] > max_norm){

			max_norm = data[i];
			max_norm_index = i;
		}
	}

	//Go forward
	if(max_norm_index >= FREQ_FORWARD_L && max_norm_index <= FREQ_FORWARD_H){
		command = GO_FORWARD;
	}
	//Turn left
	else if(max_norm_index >= FREQ_LEFT_L && max_norm_index <= FREQ_LEFT_H){
		command = TURN_LEFT;
	}
	//Turn right
	else if(max_norm_index >= FREQ_RIGHT_L && max_norm_index <= FREQ_RIGHT_H){
		command = TURN_RIGHT;
	}
	//Turn around
	else if(max_norm_index >= FREQ_HTURN_L && max_norm_index <= FREQ_HTURN_H){
		command = HALF_TURN;
	}
	//Stop
	else if(max_norm_index >= FREQ_STOP_L && max_norm_index <= FREQ_STOP_H){
		command = STOP;
	}
	//No command received
	else{
		command = RETRY;
	}

	//once a command is found a signal is issued
	chBSemSignal(&get_order_sem);
}

/* Purpose : To compute the FFT with an optimized function from a library.
 * 			 This FFT function stores the results in the input buffer given.
 *
 * Parameters :
 *
 * (in)  float* complex_buffer Points to the complex data buffer.
 *
 * (in)  uint16_t size    	   Size of the complex data buffer.
 *
 */
void doFFT_optimized(uint16_t size, float* complex_buffer){

	if(size == 1024)
		arm_cfft_f32(&arm_cfft_sR_f32_len1024, complex_buffer, 0, 1);
}

