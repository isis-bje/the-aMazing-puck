#ifndef MOVE_H_
#define MOVE_H_

#include "ch.h"
#include "hal.h"

#define AUTO                        4
#define SEMIAUTO                    12
#define SLEEP_TIME				 	50 //[ms]

//---------------------------External declarations---------------------------

void move_start(uint8_t mode);

//---------------------------------------------------------------------------

#endif /* MOVE_H_ */
