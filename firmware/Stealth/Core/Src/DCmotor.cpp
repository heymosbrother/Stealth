/*
 * DCmotor.cpp
 *
 *  Created on: Nov 22, 2023
 *      Author: heymosbrother
 */

#include "DCmotor.h"

DCmotor::DCmotor() {
	// TODO Auto-generated constructor stub

}

DCmotor::~DCmotor() {
	// TODO Auto-generated destructor stub
}

DCmotor::tickCount(int direciton) {
	if(direction == 1) {
		ticks++;
	} else if(direction == -1) {
		ticks--;
	}
}
