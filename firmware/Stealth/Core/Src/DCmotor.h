/*
 * DCmotor.h
 *
 *  Created on: Nov 22, 2023
 *      Author: heymosbrother
 */

#ifndef SRC_DCMOTOR_H_
#define SRC_DCMOTOR_H_

// Library including
#include "stm32f4xx_hal.h" // for PWM, GPIO and EXTI

class DCmotor {
public:
	DCmotor(); 			// contructor
	virtual ~DCmotor(); // destructor

	void tickCount(int direction);	// add or deduct tick by 1

private:
	long long int ticks;
};

#endif /* SRC_DCMOTOR_H_ */
