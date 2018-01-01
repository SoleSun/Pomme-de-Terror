#pragma once

#include "Encoder.h"
#include "Configuration.h"

class Zipline_Navigator{
	private: 
		int thresholdVal; 
		int proportionalVal;
		int derivativeVal;
		int speedVal;
		int distToZipline;
		int degreeToTurn;
		int noOfCrossesEncountered;
		int rightConst;
		int leftConst;

		void tapeFollow (bool drivingOnLeftSurface);
		void driveToZipline(); 
		void latch (bool drivingOnLeftSurface);
		void lift ();
		void lower ();
		
	public:
		Zipline_Navigator (int thresh, int p, int d, int speed, int dist, int degree, int rightConstant, int leftConstant);
		~Zipline_Navigator();
		void Drive(bool drivingOnLeftSurface);
};
