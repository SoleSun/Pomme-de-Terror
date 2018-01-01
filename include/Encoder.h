#pragma once
#include "Configuration.h"
#include <stdint.h>

extern volatile uint32_t counts[4];
extern volatile unsigned long prevTime[4];
    
class Encoder {
	private: 
		void enableExternalInterrupt (unsigned int INTX, unsigned int mode);
		void disableExternalInterrupt (unsigned int INTX);
		void start (unsigned int INTX);
		int stop (unsigned int INTX);
		
	public:
		Encoder ();
		~Encoder();
		int getDistanceRightWheel();
		int getDistanceLeftWheel();
		int getTicks (unsigned int INTX);

    int convertDistToTicks (unsigned int distance);

};
