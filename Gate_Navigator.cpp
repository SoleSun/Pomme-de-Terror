#include "include/Gate_Navigator.h"

int thresholdVal, proportionalVal, derivativeVal, speedVal, distToGateVal, threshGateVal, distanceAfterGateVal;

Gate_Navigator::Gate_Navigator (int thresholdValue, int proportionalGain, int derivativeGain, int motorSpeed, int distanceToGate, int threshGate, int distanceAfterGate) {
	thresholdVal         = thresholdValue;
	proportionalVal      = proportionalGain;
	derivativeVal        = derivativeGain;
	speedVal             = motorSpeed;
    distToGateVal        = distanceToGate;
    threshGateVal        = threshGate;
    distanceAfterGateVal = distanceAfterGate;
};

/**
 * Change the hard-coded PID parameters
 * for a slower speed
 */
void Gate_Navigator::driveSlow(){
  thresholdVal   = 150;
  speedVal       = 90;
  proportionalVal = 25;
  derivativeVal  = 12;
}

/**
 * @return 10 if all four sensors have been activated; else follow truth table
 */
int Gate_Navigator::getErrorValue(int prevError) {

  /* 
   *  L    CL    CR    R    Err    Index
   *  0    0     0     0     4      0
   *  0    0     0     1     3      1  
   *  0    0     1     0     1      2
   *  0    0     1     1     2      3
   *  0    1     0     0    -1      4
   *  0    1     0     1     6      5
   *  0    1     1     0     0      6
   *  0    1     1     1     6      7
   *  1    0     0     0    -3      8
   *  1    0     0     1     5      9
   *  1    0     1     0     5      10
   *  1    0     1     1     5      11
   *  1    1     0     0    -2      12
   *  1    1     0     1     5      13
   *  1    1     1     0     6      14
   *  1    1     1     1     7      15
   */
   
  //const int errorValues [16] = {4,3,1,2,-1,6,0,6,-3,5,5,5,-2,5,6,7};
  
  bool 
    L = analogRead(leftQRDSensor) > thresholdVal,
    CL = analogRead(centreLeftQRDSensor) > thresholdVal,
    CR = analogRead(centreRightQRDSensor) > thresholdVal,
    R = analogRead(rightQRDSensor) > thresholdVal;

  if ((CL && CR) && (L||R)) return 10;
  else if ( CL && CR )     return 0;
  else if ( CL && !CR )    return -1;
  else if ( !CL && CR )    return 1;
  else                     return (prevError > 0) ? 2 : -2;
}

/*
 * @param true if the robot is running on the surface with the
 * claw facing outwards
 * @return true if the first cross has been detected 
 */
bool Gate_Navigator::Drive(bool drivingOnLeftSurface) {
  /* Parameters for calculating PID tape navigation */
  int lastError = 0, recentError = 0;
  int q = 0, m = 0, con = 0;

  int leftSurfacePostGateOffsetDist = 60; 
  
  /* Access the encoder functionality be instatiating it */
  Encoder distCalculator = Encoder();
  
  /* Parameters for determining if robot has stopped at least once before the gate */
  bool stoppedOnce = false, passedRamp = false;
  
  GateSequenceStates currentState = detectingGate;
  
  /* Give some manifest signal that the routine is starting */
  LCD.clear(); LCD.home();
  LCD.print("Sequence 1");
  delay(1000); 
      
  while (true){
  
      switch (currentState){ 
    
    /* The distance that was travelled so far */
  	int averageDist = (distCalculator.getDistanceRightWheel() + distCalculator.getDistanceLeftWheel()) / 2;
    
    if (drivingOnLeftSurface) {
      if (averageDist > (distanceAfterGateVal - leftSurfacePostGateOffsetDist)){
        driveSlow();
        
        if ( analogRead(leftQRDSensor) > thresholdVal && analogRead(centreLeftQRDSensor) > thresholdVal && analogRead(centreRightQRDSensor) > thresholdVal && analogRead(rightQRDSensor) > thresholdVal){
          motor.speed(leftMotor, -speedVal);    motor.speed(rightMotor, -speedVal); 
  
          return true;
        }
      }
    }
    else {
      if (averageDist > distanceAfterGateVal){
        motor.speed(leftMotor, 0);    motor.speed(rightMotor, 0);
        delay(200);
        
        LCD.print("Entrance at:"); LCD.setCursor(0,1); LCD.print(averageDist);
        
        forward(10);
  
        while(analogRead(centreRightQRDSensor) < thresholdVal){
          motor.speed(leftMotor,-70);    motor.speed(rightMotor,-70);
        }
  
        motor.speed(leftMotor, 0);    motor.speed(rightMotor, 0);
                
        return true;
      }
    }
    
      
    /* If the beacon is not flashing 1 KHz and the robot has already travelled its allotted safe distance */
  	if (!stoppedOnce && averageDist > distToGateVal) {

      /* Stop the robot */
      motor.speed(leftMotor, 0);
      motor.speed(rightMotor, 0);
      stoppedOnce = true;

//      if (drivingOnLeftSurface){
        /* Wait for the gate alarm to cycle at least once */
        /* If the analog value goes below the threshold, then the gate is off */
        while (analogRead(OneKHzSensorPin) > threshGateVal){ 
          /* If the gate is initially on, then it will catch on this loop. */
          LCD.clear(); LCD.home();
          LCD.print("Stop Dist: "); LCD.print(averageDist); 
          LCD.setCursor(0,1); LCD.print("QSD: "); LCD.print(analogRead (OneKHzSensorPin));
          delay(15);
        }
        while (analogRead(OneKHzSensorPin) < threshGateVal){ 
          /* If the gate is initially off, then it will catch on this one*/
          LCD.clear(); LCD.home();
          LCD.print("Stop Dist: "); LCD.print(averageDist); 
          LCD.setCursor(0,1); LCD.print("QSD: "); LCD.print(analogRead (OneKHzSensorPin));
          delay(15);
        }

//        recentError = lastError = 0;
//      } else {
//        /* Wait for the gate alarm to cycle at least once */
//        /* If the analog value goes below the threshold, then the gate is on */
//        while (analogRead(OneKHzSensorPin) < threshGateVal){ 
//          /* If the gate is initially on, then it will catch on this loop. */
//          LCD.clear(); LCD.home();
//          LCD.print("Stop Dist: "); LCD.print(averageDist); 
//          LCD.setCursor(0,1); LCD.print("QSD: "); LCD.print(analogRead (OneKHzSensorPin));
//          delay(15);
//        }
//        while (analogRead(OneKHzSensorPin) > threshGateVal){ 
//          /* If the gate is initially off, then it will catch on this one*/
//          LCD.clear(); LCD.home();
//          LCD.print("Stop Dist: "); LCD.print(averageDist); 
//          LCD.setCursor(0,1); LCD.print("QSD: "); LCD.print(analogRead (OneKHzSensorPin));
//          delay(15);
//        }
//      }
  	}
    else {
      /* Check the QRD sensors */
      int error = getErrorValue (lastError); 
      
  		if(error != lastError){
  		  recentError = lastError;
  		  q=m;
  		  m=1;
  		}
  	  
  		int proportional = proportionalVal * error,
  			derivative   = (int) (derivativeVal * (float)(error - recentError) / (q + m) );
  		con = proportional + derivative;
  	  
  		m++;
  		motor.speed(leftMotor, -speedVal + con);    motor.speed(rightMotor, speedVal + con);
  	  
  		lastError = error;
  	}

    if (stopbutton()) {
      delay(100);
      if (stopbutton())
      { 
        LCD.clear(); LCD.home();
        LCD.print("Exiting");
        LCD.setCursor(0,1); LCD.print("Drive");
        delay(500);
        return false;
      }
    }

  }
};
