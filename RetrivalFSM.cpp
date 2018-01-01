# include "include/RetrivalFSM.h"

/////////// TEMP
#define CROSS_DETECT_NEW 

// alignment Parameters
#define START_SPEED 90
#define LEFT_FRONT_CONST 1
#define RIGHT_FRONT_CONST 1
#define LEFT_BACK_CONST 1
#define RIGHT_BACK_CONST 1
#define FORWARD_LEFT_LEFT_WHEEL_SPEED 0
#define FORWARD_LEFT_RIGHT_WHEEL_SPEED 70


//PID parameters:
int proportional; 
int derivative; 
int threshold; 
int motorSpeed; 
int lastError, recentError;
int q = 0, m = 0, con = 0; 

// Competition Surface Left
int armDownPositionsLeft[6] = {5,5,5,5,5,5};
int alignmentDistanceForwardLeft[7] = {10,10,10,10,10,10,10};
int alignmentDistanceBackwardLeft[7] = {3, 10, 10, 10, 10,10,5};
int forwardDistanceLeft[6]= {15, 15, 15, 15,20,15};

// StateMachine State:
States g_CurrentState = S_TapeFollow; 

// Testing pursposes
#define LCDDELAY 500
//#define DEBUG 7
long cumulativeTime = 0; 
int startTime = 0; 


void executeRetrivalFSM(int p, int d, int QRDthreshold, int MotorSpeed){

    // initial set up. 
    bool fsmDone = false;
    Claw newClaw(CLAWOPENPOSITION, CLAWCLOSEPOSITION, ARMUPPOSITION,CLAWDELAY, ARMDELAY, CLAWPIN, ARMPIN); 
    int counter = 0;

    // tape following requirements. 
    lastError = 0;
    recentError = 0; 
    
    // PID: 
    proportional = p;
    derivative = d;
    threshold = QRDthreshold;
    motorSpeed = MotorSpeed; 


    startTime = millis();
    
    while(!fsmDone){
        switch(g_CurrentState){ 
            LCD.clear();
                    LCD.home();
                     LCD.clear();
                    LCD.print("Cross Detected");
                   
            case S_TapeFollow:  
             tapeFollow(); 
            // this state just is suppose to follow tape and then stop. 
                if(detectCross()) {
                    //printing for clarification. 
                    LCD.clear();
                    LCD.home();
                    LCD.print("Cross Detected");


                    // new state machine: /////////////////////////////////////////////////
                    g_CurrentState = S_Alignment; 
                }
                break;
                

            case S_Alignment:
                // alignment of the chassis for grabbing the agent.
                // momve forward, left turn to find the tape, then go backwards. Then go back to tape following. 
                forward(alignmentDistanceForwardLeft[counter]);

                LCD.clear();
                    LCD.home();
                    LCD.clear();
                    LCD.print("Alignment Maneuver");

                    // left surface
                if(counter !=0){
                    while(!backOnTape()){
                    forwardAndLeft();
                    }  
                }
                
                
                reverse(alignmentDistanceBackwardLeft[counter]);
                g_CurrentState = S_Retrieve; 
                motor.speed(leftMotor, 0);
                motor.speed(rightMotor, 0);
                break; 



             case S_TapeAlignment:
                    // go forward:
                    forward(forwardDistanceLeft[counter]);
                    // now pivoting the wheel to turn right
//                    while(!analogRead(centreLeftQRDSensor) > threshold){
//                         motor.speed(rightMotor, 0);
//                         motor.speed(leftMotor, -50);
//                    }

                    g_CurrentState = S_TapeFollow;
                     counter++; 
                    break;  
            
//            case S_Forward:
//                // go slightly forward to align the claw with the animal 
//                 LCD.clear();
//                 LCD.home();
//                 LCD.print("Forward");
//                 delay(LCDDELAY );
//                 LCD.clear();
//
//                 // calling go forward function.
//                forward(forwardDistanceLeft[counter]);
//                 // stopping the motor:
//                motor.speed(leftMotor, 0);
//                motor.speed(rightMotor, 0);

                // setting the speeds for the motors as well as switching the states. 
                //g_CurrentState = S_Retrieve;

                ////////////////////////////////////// new state machine

               g_CurrentState = S_TapeFollow; 
                break;
                    
            
            case S_Retrieve:
                LCD.clear();
                LCD.home();
                LCD.print("Retrieve");    
//                delay(LCDDELAY);

                 // if not grabbed 6 agents yets, retrieve more. 
                if (counter >= 0 && counter < 6){
                    newClaw.retrieve(armDownPositionsLeft[counter]);
                   // newt state machine:
                   g_CurrentState = S_TapeAlignment;
                } 
                else if(counter >=6){
                    // if 6 agents have been grabbed, switch states correspodningly.
                    g_CurrentState = S_Exit;
                }

                // resetting errors:
                lastError = 0; 
                break; 
            
            
//            case S_ForwardLeft:
////                LCD.clear();
////                LCD.home();
////                LCD.print("forwardLeft");
////                delay(LCDDELAY);
//
//                 // make the cart go forward and left until tape is detected.
//                while(!backOnTape()){
//                    forwardAndLeft();
//                }
//                 // back to tape following. 
//                g_CurrentState = S_TapeFollow;
//                break;


            case S_Exit: 
                LCD.clear();
                LCD.home();
                LCD.print("done");
                delay(LCDDELAY);
                fsmDone = true; 
                counter = 0; 
                g_CurrentState = S_TapeFollow;
                motor.speed(leftMotor, 0);    motor.speed(rightMotor,0);
                break; 
            
            default:
                break; 
        }
    
    }
    
 }
 

// check cross 

const bool detectCross(){
    //reading in inputs from QRDs
    bool 
        L = analogRead(leftQRDSensor) > threshold,
        CL = analogRead(centreLeftQRDSensor) > threshold,
        CR = analogRead(centreRightQRDSensor) > threshold,
        R = analogRead(rightQRDSensor) > threshold; 


  #ifdef DEBUG
        cumulativeTime += millis()- startTime;
        if(cumulativeTime > 1000){
         LCD.clear();
         LCD.home();
         LCD.print(" CR  "); LCD.print(analogRead(centreRightQRDSensor)); 
         LCD.print(" CL "); LCD.print(analogRead(centreLeftQRDSensor)); 
         LCD.setCursor(0,1);
         LCD.print("L "); LCD.print(analogRead(leftQRDSensor)); 
         LCD.print(" R "); LCD.print(analogRead(rightQRDSensor)); 
        }
         
   #endif

    if((L||R)&&(CL&&CR)){      
        return true;
      }else{
        return false; 
      }
       
     //checking if a cross is detected, ie: when center 2 sees tape as well as one of the outer ones.

     
//     if (CL && CR) {
//        // on track
//        if (L && R) {
//            return true;
//        }
//        else if (L) {
//            motor.speed(rightMotor,0);
//            motor.speed(leftMotor,ALIGNSPEED);
//        }
//        else if (R) {
//            // turn right motor off (or slow)  
//             motor.speed(leftMotor,0); 
//             motor.speed(rightMotor,ALIGNSPEED); 
//        }
//    }
//    return false; 
}

// back to tape
const bool backOnTape(){
    //require testing**********************************************
    bool 
        CL = analogRead(centreLeftQRDSensor) > threshold,
        CR = analogRead(centreRightQRDSensor) > threshold;
        

    if( CR)
        return true;
    else 
        return false; 
    
}


//tape following
void  tapeFollow(){
    // boolean values for PID tuning
    bool 
        CL = analogRead(centreLeftQRDSensor) > threshold,
        CR = analogRead(centreRightQRDSensor) > threshold;
    // Copied from Joel
    int error;
          if ( CL && CR )       error = 0;
        else if ( CL && !CR )    error = -1;
        else if ( !CL && CR )    error = 1;
        else{
           if( lastError < 0 )    error = -2;
           else                 error = 2;
        }
        if(!(error == lastError)){
          recentError = lastError;
          q=m;
          m=1;
        }
      
        int proportionalVal = proportional * error;
        int derivativeVal   = (int) (derivative * (float)(error - recentError) / (q + m) );
        con = proportionalVal + derivativeVal;
      
        m++;
        motor.speed(leftMotor, -motorSpeed + con);
        motor.speed(rightMotor, motorSpeed + con);
      
        lastError = error;


        // testing code:
//        LCD.home();
//        LCD.clear();
//        LCD.print(" CR  "); LCD.print(analogRead(centreRightQRDSensor)); 
//         LCD.print(" CL "); LCD.print(analogRead(centreRightQRDSensor)); 
//         LCD.setCursor(0,1);
//         LCD.print("L "); LCD.print(analogRead(leftQRDSensor)); 
//         LCD.print(" R "); LCD.print(analogRead(rightQRDSensor)); 
//       delay(100);

}
 
 
 
 
 // move Forward
 
 void forward(int distanceToTravel){
 
    maneuver(distanceToTravel, distanceToTravel,RIGHT_FRONT_CONST,LEFT_FRONT_CONST, START_SPEED, false);
 }


 void forwardAndLeft(){
        
       //make the motors travel forward but veer off to the left. The numbers needs to be tuned. 
       motor.speed(leftMotor, FORWARD_LEFT_LEFT_WHEEL_SPEED);
       motor.speed(rightMotor, FORWARD_LEFT_RIGHT_WHEEL_SPEED);
    
 }

 
 void reverse(int distanceToBack){
    maneuver(distanceToBack, distanceToBack,RIGHT_BACK_CONST,LEFT_BACK_CONST, START_SPEED, true);
 }
 
 
 // maneuvering the robot. 
void maneuver(int leftTargetDistance, int rightTargetDistance,int leftConstant, int rightConstant, int startMotorSpeed, bool reverse){
    // setting up 
    Serial.begin(9600);
    Encoder encoders = Encoder();
    long startTime = millis();
    int leftD;
    int rightD;
    int leftSpeed = startMotorSpeed; 
    int rightSpeed = startMotorSpeed; 
    int error; 

    // going through the loop and while the distance travelled by either of the wheels have reached the target distance. 
    // tune the speed according to eachother. 
    // i am gonna make the left wheel the reference, ie: tune the right wheel, because the left wheel seems to travel a bit slower.
     
    while(leftTargetDistance > encoders.getDistanceLeftWheel() && rightTargetDistance > encoders.getDistanceRightWheel()){
        
        //reading distance travelled by wheels
        leftD = encoders.getDistanceLeftWheel();
        rightD = encoders.getDistanceRightWheel();
         
        startTime = millis();

        // if left travels further than right, then slow the leftWheel
        if(leftD > rightD){
            error = 1;
        } else if (leftD < rightD){
            error = -1;
        } else {
            error = 0;
        }

        //writing the speeds. 
        if(!reverse){
            leftSpeed = -startMotorSpeed + leftConstant * error; 
            rightSpeed = startMotorSpeed + rightConstant * error;
            motor.speed(leftMotor, leftSpeed);
            motor.speed(rightMotor,rightSpeed);
        }else{
           leftSpeed = (startMotorSpeed+20) - leftConstant * error; 
           rightSpeed = -(startMotorSpeed + 20) - rightConstant * error;
           motor.speed(leftMotor, leftSpeed);
           motor.speed(rightMotor, rightSpeed); 
        }

 //debugging code;
        #ifdef DEBUG
          cumulativeTime += millis()- startTime;
          if(cumulativeTime >= 50){
            LCD.clear(); LCD.home();
            LCD.print("L: "), LCD.print(leftD),LCD.print(" "), LCD.print(leftSpeed);
            LCD.setCursor(0,1);
            LCD.print("R:"), LCD.print(" "), LCD.print(rightD), LCD.print(" "), LCD.print(rightSpeed);
            cumulativeTime = 0; 
          }
              
       #endif
   }

    // stopping all the motors
     motor.speed(leftMotor,0);
     motor.speed(rightMotor,0); 
  
    
//    #ifdef DEBUG
//         while(true){
//            if (stopbutton()){
//                delay(100);
//                if(stopbutton()){
//                    return;
//                }
//            }
//         }
//    #endif    
}

//void maneuver(int leftTargetDistance, int rightTargetDistance,int leftConstant, int rightConstant, int minimumMotorSpeed, bool reverse){
//    Serial.begin(9600);
//    int leftDifference = leftTargetDistance;
//    int rightDifference = rightTargetDistance; 
//    Encoder encoders = Encoder();
//    long startTime = millis();
//    
//    while(leftDifference > 0 && rightDifference >0){
//        leftDifference = leftTargetDistance- encoders.getDistanceLeftWheel();
//        rightDifference = rightTargetDistance - encoders.getDistanceRightWheel();
//
//    if(stopbutton()){
//        return; 
//    }
//        // calculating the calibrated speeds. 
//        int leftSpeed; 
//        int rightSpeed; 
//        if( reverse){
//            leftSpeed = (minimumMotorSpeed + leftDifference * leftConstant);
//            rightSpeed = - (minimumMotorSpeed + rightDifference * rightConstant); 
//            
//            if (backOnTape()){
//                return; 
//            }
//        }else{
//         leftSpeed = -(minimumMotorSpeed + leftDifference * leftConstant);
//         rightSpeed = minimumMotorSpeed + rightDifference * rightConstant;
//        }
//
//        // if the difference is zero, set the speed to zero. 
//        if(leftDifference <= 0){
//            leftSpeed = 0;
//        }
//        if(rightDifference <= 0){
//            rightSpeed = 0; 
//        }
//
//    cumulativeTime += millis()- startTime;
//        #ifdef DEBUG
//
//          if(cumulativeTime >= 1000){
//            LCD.clear();
//            LCD.print("L: "), LCD.print(leftDifference),LCD.print(" "), LCD.print(encoders.getDistanceLeftWheel()),LCD.print(" "), LCD.print(rightSpeed);
//            LCD.setCursor(0,1);
//            LCD.print("R:"), LCD.print(" "), LCD.print(rightDifference), LCD.print(" "), LCD.print(encoders.getDistanceRightWheel()), LCD.print(" "), LCD.print(leftSpeed);
//            cumulativeTime = 0; 
//          }
//              
//       #endif
//
//    //changing the speeds of the motors. 
//    
//        motor.speed(leftMotor, leftSpeed);
//        motor.speed(rightMotor, rightSpeed);
//    }
//
//    motor.speed(leftMotor, 0);
//    motor.speed(rightMotor, 0);
//
//    while(true){
//        if (stopbutton()){
//            delay(100);
//            if(stopbutton()){
//                return;
//            }
//        }
//    }
//}
 
 

 
 
