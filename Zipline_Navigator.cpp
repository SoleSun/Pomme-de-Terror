#include <phys253.h>
#include <LiquidCrystal.h>
#include "include/Zipline_Navigator.h"
#include "include/RetrivalFSM.h"

/**
 * Many of the parameters for navigating need to be determined via trial and error
 * so manually setting the parameters is most effective strategy
 */
Zipline_Navigator::Zipline_Navigator (int thresh, int p, int d, int speed, int dist, int degree, int rightConstant, int leftConstant){
	thresholdVal = thresh;
	proportionalVal = p;
	derivativeVal = d;
	speedVal = speed;
	distToZipline = dist;
	degreeToTurn = degree;
	noOfCrossesEncountered = 0;
    rightConst = rightConstant;
    leftConst = leftConstant;
}

/**
 * Call this to navigate the robot to the cross bar that whose
 * tangent line leads straight to the zipline
 */
void Zipline_Navigator::tapeFollow(bool drivingOnLeftSurface) {
	int lastError = 0, recentError = 0;
  int q = 0, m = 0, con = 0;
  
	/* 
	Assuming the zipline navigator takes control after six crosses, 
	the minimum number of crosses to traverse is three
	*/
  int maximumCrosses;
  if (drivingOnLeftSurface) maximumCrosses = 2;
  else                      maximumCrosses = 1;
  
	while (true){

    LCD.clear(); LCD.home();

    bool CR, CL;
    
		if ( (analogRead(centreRightQRDSensor) > thresholdVal || analogRead(centreLeftQRDSensor) > thresholdVal) && (analogRead(rightQRDSensor) > thresholdVal || analogRead(leftQRDSensor) > thresholdVal)){
      noOfCrossesEncountered++;
      
      motor.speed(rightMotor, 0);    motor.speed(leftMotor, 0);    delay(100);    

       LCD.print("Cross Detected");
      LCD.setCursor(0,1);
      LCD.print(noOfCrossesEncountered);
      
      forward(15);
      // now pivoting the wheel to turn right
      
      while(analogRead(centreLeftQRDSensor) < thresholdVal){
           motor.speed(rightMotor, 65);    motor.speed(leftMotor, 65);
      }
      motor.speed(rightMotor, 0);    motor.speed(leftMotor, 0);   delay(100);
			
			if (noOfCrossesEncountered > maximumCrosses) {
			  return;
		  }
     
		}
   
    CL = analogRead(centreLeftQRDSensor) > thresholdVal;
    CR = analogRead(centreRightQRDSensor) > thresholdVal;
      
		int error;
		if ( CL && CR )          error = 0;
		else if ( CL && !CR )    error = -1;
		else if ( !CL && CR )    error = 1;
		else               		   error = (lastError > 0) ? 2 : -2;
		
		if(!(error == lastError)){
		  recentError = lastError;
		  q=m;
		  m=1;
		}
	  
		int proportional = proportionalVal * error,
			derivative   = (int) (derivativeVal * (float)(error - recentError) / (q + m) );
		con = proportional + derivative;
	  
		m++;
		motor.speed(leftMotor, -speedVal + con);
		motor.speed(rightMotor, speedVal + con);
	  
		lastError = error;
	}
}

/**
 * Drives the robot straight to the zipline until it reaches
 * a predetermined distance 
 */
void Zipline_Navigator::driveToZipline() {
  maneuver(distToZipline, distToZipline, leftConst, rightConst, speedVal, false);
  Claw newClaw(CLAWOPENPOSITION, CLAWCLOSEPOSITION, ARMUPPOSITION,CLAWDELAY, ARMDELAY, CLAWPIN, ARMPIN); 
  newClaw.stageThreeConfiguration();
}

/**
 * Assume robot has successfully reached the zipline,
 * driving in a straight line. Now maneuver to have the 
 * box face the zipline
 */
void Zipline_Navigator::latch(bool drivingOnLeftSurface) {
	if (drivingOnLeftSurface){
	  motor.speed(leftMotor, 0);    motor.speed(rightMotor, 0);

    lift();
  
    /* Now slowly foward move */
    maneuver (34,34,leftConst, rightConst, speedVal, false);
  
    lower(); 
	} 
	else {
    /* Turn Left */
    motor.speed(leftMotor, -90);    motor.speed(rightMotor, -90); 
    delay(degreeToTurn);
    /* Stop after turning */
    motor.speed(leftMotor, 0);    motor.speed(rightMotor, 0);
    /* Claw is close to zipline. Move backwards and give the lift some space */
    maneuver (30, 30, leftConst, rightConst, speedVal, true);    

    /* Lift the box with the animals */
    lift();
  
    /* Now slowly foward move */
    maneuver (35,35,leftConst, rightConst, speedVal, false);
  
    lower(); 
	}
  
  return;
}

void Zipline_Navigator::lift(){
  RCServo0.attach(RCServo0Output);    RCServo1.attach(RCServo1Output);

  /* Original angles were 15 and 63*/
  //10 and 46
  //2 and 26
  //2 and 30
  //2 and 33
  const int initalizedangle = 177, servo0 = 2, servo2 = 33;
  int incrementstandard = initalizedangle - servo2;

  // 102 + 8 = 110 must be << 114(max increment for servo2)
  for (int i = 0; i <= 102; i++){
    if ( i > 0 && i < 102){
      RCServo0.write(initalizedangle - 40 - i);    RCServo1.write(initalizedangle - 8 - i);
    }
    else if (i == 0){
      RCServo0.write(initalizedangle - 37);        RCServo1.write(initalizedangle - 8);
    }
    else{
      RCServo0.write(servo0);      RCServo1.write(servo2);
    }
    delay(30);
  }

  return;
}

void Zipline_Navigator::lower(){
  const int initalizedangle = 177, servo0 = 2, servo2 = 33;
  int incrementstandard = initalizedangle - servo2;

  // 102 + 8 = 110 must be << 114(max increment for servo2)
  for (int i = 102; i >= 0; i--){
    if ( i > 0 && i < 102){
      RCServo0.write(initalizedangle - 40 - i);    RCServo1.write(initalizedangle - 8 - i);
    }
    else if (i == 0){
      RCServo0.write(initalizedangle - 37);        RCServo1.write(initalizedangle - 8);
    }
    else{
      RCServo0.write(servo0);      RCServo1.write(servo2);
    }
    delay(30);
  }

  RCServo0.detach();    RCServo1.detach();

  return;
}

void Zipline_Navigator::Drive(bool drivingOnLeftSurface){
  tapeFollow(drivingOnLeftSurface);
  motor.speed(leftMotor,0); motor.speed(rightMotor, 0);
  driveToZipline();
  latch(drivingOnLeftSurface);
}

