#include <phys253.h>
#include "include/Lift.h"

Lift::Lift(int LeftServoPin, int RightServoPin){
  
  portMode(0, OUTPUT) ;      //   ***** from 253 template file
  portMode(1, OUTPUT) ;      //   ***** from 253 template file

  portMode(LeftServoPin, OUTPUT);
  portMode(RightServoPin, OUTPUT);


  _LeftServoPin = LeftServoPin;
  _RightServoPin = RightServoPin;
    
   leftServo = new ServoTINAH;
   rightServo = new ServoTINAH;
  
  leftServo->attach(_LeftServoPin);
  rightServo->attach(_RightServoPin);

} // Constructor completed

void Lift::LiftUp(){
    leftServo->write(90);
    rightServo->write(0);
}

void Lift::LiftDown(){
    leftServo->write(0);
    rightServo->write(90);
    delay(15);

}

