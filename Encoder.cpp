#include "include/Encoder.h"
#include <phys253.h>
#include <avr/interrupt.h>

volatile uint32_t counts[4];
volatile unsigned long prevTime[4];
    
Encoder::Encoder() {
	start(leftEncoder);
	start(rightEncoder);
}

Encoder::~Encoder() {
  counts[0] = counts [1] = counts[2] = counts[3] = 0;
  prevTime[0] = prevTime [1] = prevTime[2] = prevTime[3] = 0;
}

void Encoder::enableExternalInterrupt (unsigned int INTX, unsigned int mode) {
  if (INTX > 3 || mode > 3 || mode == 1) return;
  cli();
  /* Allow pin to trigger interrupts        */
  EIMSK |= (1 << INTX);
  /* Clear the interrupt configuration bits */
  EICRA &= ~(1 << (INTX*2+0));
  EICRA &= ~(1 << (INTX*2+1));
  /* Set new interrupt configuration bits   */
  EICRA |= mode << (INTX*2);
  sei();
}

void Encoder::disableExternalInterrupt (unsigned int INTX) {
  if (INTX > 3) return;
  EIMSK &= ~(1 << INTX);
}

void Encoder::start (unsigned int INTX) {
  counts[INTX] = 0;
  enableExternalInterrupt(INTX, RISING);
}

int Encoder::getTicks (unsigned int INTX){
  return counts[INTX];
}

/*
 *  @return value the right encoder has travelled in centimeters
 */
int Encoder::getDistanceRightWheel(){
	return (int) ((counts[rightEncoder] * wheelDiameter * pi) / (gearRatio * 48));
}

/* 
 *  @return value the left encoder has travelled in centimeters
 */
int Encoder::getDistanceLeftWheel(){
	return (int) ((counts[leftEncoder] * wheelDiameter * pi) / (gearRatio * 48));
}

int Encoder::convertDistToTicks (unsigned int distance) {
  return (int) ((distance * gearRatio * 48) / (wheelDiameter * pi));
}

ISR(INT0_vect){
  if (millis() > prevTime[0] + waitTime)
  {
    counts[0]++;
    prevTime[0] = millis();
  }
}

ISR(INT1_vect){
  if (millis() > prevTime[1] + waitTime)
  {
    counts[1]++;
    prevTime[1] = millis();
  }
}

ISR(INT2_vect){
  if (millis() > prevTime[2] + waitTime)
  {
    counts[2]++;
    prevTime[2] = millis();
  }
}

ISR(INT3_vect){
  if (millis() > prevTime[3] + waitTime)
  {
    counts[3]++;
    prevTime[3] = millis();
  }
}
