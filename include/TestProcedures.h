#pragma once
//#include <Arduino.h>

class TestProcedures {
	
	public:
		TestProcedures ();
		~TestProcedures();
		void testGateSensors();
		void testPID (int thresholdVal, int proportionalVal, int derivativeVal, int speedVal);
    void testMotors ();
    void testLift ();
    void testEncoders();
    void testMinMotor();
    void testManeuver (int leftTargetDistance, int rightTargetDistance, int maneuverLeftConstant, int maneuverRightConstant, int minMotorSpeed, bool reverse);
    void testAccelerate(int thresholdVal, int proportionalVal, int derivativeVal, int speedVal);
    void clawTesting();
    void testQRDs();
};
