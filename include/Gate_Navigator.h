#pragma once 
#include "include/RetrivalFSM.h"
#include <phys253.h>
#include <LiquidCrystal.h>
#include "include/Configuration.h"
#include "include/Encoder.h"

class Gate_Navigator {
    private: 
		  int thresholdVal, proportionalVal, derivativeVal, speedVal, distToGateVal, threshGateVal, distanceAfterGateVal;
		
    public:
		Gate_Navigator (int thresholdValue, int proportionalGain, int derivativeGain, int motorSpeed, int distanceToGate, int threshGate, int distanceAfterGate);
		
		//Primary method for line following 
		bool Drive (bool drivingOnLeftSurface); 
		
		//Getter methods
		int getErrorValue(int prevError); 
		
		void driveSlow ();
};

enum GateSequenceStates { s_detectingGate, s_detectedGate, s_passedThroughGate, s_detectedPrisonEntrance } 

/* First Stage: detectingGate */
void infraredSearch (); 
