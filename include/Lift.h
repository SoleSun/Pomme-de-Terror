/*
 * Lift.h - Header file for the lifting mechnism 
 */
#pragma once
 #ifndef Lift_h
 #define Lift_h

 #include <phys253.h>

class Lift{
    
  public:
  Lift(int LeftServoPin, int RightServoPin);
    void LiftUp();
    void LiftDown();

  private:
  
    int _LeftServoPin;
    int _RightServoPin;
    
    ServoTINAH *leftServo;
    ServoTINAH *rightServo;

 
};

#endif
