/* 
 *  Main file that switches between states
 *  and handles the UI 
 */
 
#include <phys253.h>
#include <LiquidCrystal.h>
#include <avr/eeprom.h>
#include <avr/interrupt.h>
#include "include/Configuration.h"
#include "include/MenuItem.h"
#include "include/Encoder.h"
#include "include/Gate_Navigator.h"
#include "include/TestProcedures.h"
#include "include/RetrivalFSM.h"
#include "include/Zipline_Navigator.h"
/*
 * A cross is black tape that crosses the path, indicating either the entrance
 * to the agent bowl or the location of the agents. Once the first cross is 
 * encountered, the code assumes that following crosses are agent locations
 */
int noOfCrossesEncountered = 0;

uint16_t MenuItem::MenuItemCount = 0;
/* 
 *  Add the menu items here 
 *  WARNING!! When new MenuItems are added,
 *  make sure to chang the numberOfPIDMenuOptions
 *  in the Configuration.h file!!
 *  Bad code I know...
 */
MenuItem * Speed;
MenuItem * ProportionalGain;
MenuItem * DerivativeGain;
MenuItem * Threshold; /* Thershold for detecting black line */
MenuItem * DistanceToGate; /* Distance to the gate */
MenuItem * DistanceAfterGate; /* Distance after gate telling the robot to slow down */
MenuItem * ThresholdGate; /* IR analog read value for detecting an ON and OFF */
MenuItem * LeftTargetDistanceValue; 
MenuItem * RightTargetDistanceValue;
MenuItem * ManeuverLeftConstant;
MenuItem * ManeuverRightConstant;
MenuItem * DistanceToZipline;
MenuItem * StartMotorSpeed;   
MenuItem * HowLongToTurn;        
MenuItem * menuItems[numberOfPIDMenuOptions];
Gate_Navigator * gateSequence;
Zipline_Navigator * ziplineSequence;
// **(array + 1)

/* 
 *  Mediator responsible for delegating and 
 *  switching between the different states
 *  
 *  @param true if the robot is running with the claw facing us;
 *         else, false
 */
void Pilot(bool left) {
  gateSequence = 
  new Gate_Navigator (Threshold->Value, ProportionalGain->Value, DerivativeGain->Value, Speed->Value, DistanceToGate->Value, ThresholdGate->Value, DistanceAfterGate->Value);
  gateSequence->Drive(left);

  executeRetrivalFSM(25, 12, 150, 90);

  ziplineSequence = 
  new Zipline_Navigator (150, 25, 12, 90, DistanceToZipline->Value, HowLongToTurn->Value, ManeuverRightConstant->Value, ManeuverLeftConstant->Value);
  ziplineSequence->Drive(left);
}

void testMenu() {
  LCD.clear(); LCD.home();
  LCD.print("Entering");
  LCD.setCursor(0,1); LCD.print("Test Menu");
  delay(500);

  const int noOfTestOptions = 11; 
  
  TestProcedures t = TestProcedures (); 
  
  while (true) {
    int menuIndex = map(knob(6), 0, 1023, 0, noOfTestOptions);

    LCD.clear(); LCD.home();
    switch (menuIndex){
      case 0:
        LCD.print(">Gate PID");
        LCD.setCursor(0,1); LCD.print("Motor Lift");
        break;
      case 1:
        LCD.print("Gate >PID");
        LCD.setCursor(0,1); LCD.print("Motor Lift");
        break;
      case 2:
        LCD.print("Gate PID");
        LCD.setCursor(0,1); LCD.print(">Motor Lift");
        break;
      case 3:
        LCD.print("Gate PID");
        LCD.setCursor(0,1); LCD.print("Motor >Lift");
        break;
      case 4:
        LCD.print(">Encoder Acc");
        LCD.setCursor(0,1); LCD.print("MinMotor"); 
        break;
      case 5:
        LCD.print("Encoder >Acc");
        LCD.setCursor(0,1); LCD.print("MinMotor"); 
        break;
      case 6:
        LCD.print("Encoder Acc");
        LCD.setCursor(0,1); LCD.print(">MinMotor");
        break;
      case 7: 
        LCD.print("Encoder Acc");
        LCD.setCursor(0,1); LCD.print("MinMotor >Forward");
        break;
      case 8:
        LCD.print(">Reverse Claw");
        LCD.setCursor(0,1); LCD.print("QRD");
        break;
      case 9:
        LCD.print("Reverse >Claw");
        LCD.setCursor(0,1); LCD.print("QRD");
        break; 
      case 10:
        LCD.print("Reverse Claw");
        LCD.setCursor(0,1); LCD.print(">QRD");
        break; 
     }
    delay(100);

    if (startbutton()) {
      delay(100);
      if (startbutton()) { 
        switch (menuIndex){
          case 0:
            t.testGateSensors();
            break;
          case 1:
            t.testPID(Threshold->Value, ProportionalGain->Value, DerivativeGain->Value, Speed->Value);
            break;
          case 2:
            t.testMotors();
            break;
          case 3:
            t.testLift();
            break;
          case 4:
            t.testEncoders();
            break;
          case 5:
            t.testAccelerate(Threshold->Value, ProportionalGain->Value, DerivativeGain->Value, Speed->Value);
            break;
          case 6: 
           t.testMinMotor();
           break;
          case 7:
            t.testManeuver(LeftTargetDistanceValue->Value,RightTargetDistanceValue->Value,ManeuverLeftConstant->Value,ManeuverRightConstant->Value,StartMotorSpeed->Value, false);
            break;
          case 8:
            t.testManeuver(LeftTargetDistanceValue->Value,RightTargetDistanceValue->Value,ManeuverLeftConstant->Value,ManeuverRightConstant->Value,StartMotorSpeed->Value, true);
            break;
          case 9:
            t.clawTesting();
            break;
          case 10:
            t.testQRDs();
            break;
        }
      } // if - cross check start button
    }

    if (stopbutton())
    {
      delay(100);
      if (stopbutton())
      { 
        LCD.clear(); LCD.home();
        LCD.print("Exiting");
        LCD.setCursor(0,1); LCD.print("Test Menu");
        delay(500);
        return;
      }
    }
  }
}

void PIDMenu() {
  LCD.clear(); LCD.home();
  LCD.print("Entering");
  LCD.setCursor(0,1); LCD.print("PID Menu");
  delay(500);
 
  while (true) {
    /* Show MenuItem value and knob value */
    int menuIndex = map(knob(6), 0, 1023, 0, numberOfPIDMenuOptions); /* Menu items plus the Drive option */
    
    LCD.clear(); LCD.home();
    LCD.print(menuItems[menuIndex]->Name); LCD.print(" "); LCD.print(menuItems[menuIndex]->Value);
    LCD.setCursor(0, 1);
    LCD.print("Set to "); LCD.print(knob(7)); LCD.print("?");
    delay(100);
    
    /* Press start button to save the new value */
    if (startbutton())
    {
      delay(100);
      if (startbutton())
      { 
        menuItems[menuIndex]->Value = knob(7);
        menuItems[menuIndex]->Save();
        delay(250);
      } // if - cross check start button
    } //if - first check start button

    if (stopbutton())
    {
      delay(100);
      if (stopbutton())
      { 
        LCD.clear(); LCD.home();
        LCD.print("Exiting");
        LCD.setCursor(0,1); LCD.print("PID Menu");
        delay(500);
        return;
      }
    }
  }
}

void mainMenu()
{
  LCD.clear(); LCD.home();
  LCD.print("Entering menu");
  delay(500);

  RCServo0.write(177);    RCServo1.write(177);
  RCServo0.detach();      RCServo1.detach();
 
  while (true){
    int menuIndex = map(knob(6), 0, 1024, 0, numberOfMainMenuOptions); /* Menu items plus the Drive option */
    
    LCD.clear(); LCD.home();
    switch (menuIndex){
      case 0:
        LCD.print("->Left Right");
        LCD.setCursor(0,1); LCD.print("Values Test");
        break;
      case 1:
        LCD.print("Left ->Right");
        LCD.setCursor(0,1); LCD.print("Values Test");
        break;
      case 2:
        LCD.print("Left Right");
        LCD.setCursor(0,1); LCD.print("->Values Test");
        break;
      case 3:
        LCD.print("Left Right");
        LCD.setCursor(0,1); LCD.print("Values ->Test");
        break;
    }
    delay(100);

    if (startbutton()) {
      delay(100);
      if (startbutton()) { 
        switch (menuIndex){
          case 0:
            Pilot(true);
            break;
          case 1:
            Pilot(false);
            break;
          case 2:
            PIDMenu();
            break;
          case 3:
            testMenu();
            break;
        }
      } // if - cross check start button
    } //if - first check start button
  }
}

void setup()
{
  LCD.clear();
  LCD.home();
  #include <phys253setup.txt>
  LCD.print("Welcome!");
  delay(1000);
  Serial.begin(9600);

  Speed                    = new MenuItem("Speed");
  ProportionalGain         = new MenuItem("P-gain");
  DerivativeGain           = new MenuItem("D-gain");
  Threshold                = new MenuItem("Threshold");
  DistanceToGate           = new MenuItem("GateDist");
  DistanceAfterGate        = new MenuItem("PostGateDist");
  ThresholdGate            = new MenuItem("ThreshGate");
  DistanceToZipline        = new MenuItem("ZipDist");
  LeftTargetDistanceValue  = new MenuItem("LTD");
  RightTargetDistanceValue = new MenuItem("RTD");
  ManeuverLeftConstant     = new MenuItem("LMP");
  ManeuverRightConstant    = new MenuItem("RMP");
  StartMotorSpeed          = new MenuItem("SMSpeed");
  HowLongToTurn            = new MenuItem("Turn");
  
  menuItems[0]      = Speed; 
  menuItems[1]      = ProportionalGain; 
  menuItems[2]      = DerivativeGain; 
  menuItems[3]      = Threshold;
  menuItems[4]      = DistanceToGate;
  menuItems[5]      = ThresholdGate;
  menuItems[6]      = DistanceAfterGate;
  menuItems[7]      = LeftTargetDistanceValue;
  menuItems[8]      = RightTargetDistanceValue;
  menuItems[9]      = ManeuverLeftConstant;
  menuItems[10]     = ManeuverRightConstant;
  menuItems[11]     = StartMotorSpeed;
  menuItems[12]     = DistanceToZipline;
  menuItems[13]     = HowLongToTurn;

}
 
void loop()
{
  mainMenu();
}
