// MultiStepper.pde
// -*- mode: C++ -*-
//
// Shows how to multiple simultaneous steppers
// Runs one stepper forwards and backwards, accelerating and decelerating
// at the limits. Runs other steppers at the same time
//
// Copyright (C) 2009 Mike McCauley
// $Id: MultiStepper.pde,v 1.1 2011/01/05 01:51:01 mikem Exp mikem $

#include <AccelStepper.h>
#define X_STEP_PIN         54
#define X_DIR_PIN          55
#define X_ENABLE_PIN       38

#define MOVEMENT 10000

long origin = 0;
bool clockwise = true;

// Define some steppers and the pins the will use
AccelStepper stepper1; // Defaults to AccelStepper::FULL4WIRE (4 pins) on 2, 3, 4, 5
AccelStepper stepper2(AccelStepper::FULL3WIRE, X_STEP_PIN, X_DIR_PIN, X_ENABLE_PIN);
AccelStepper stepper3(AccelStepper::FULL2WIRE, 10, 11);


void setup()
{  
    /*stepper1.setMaxSpeed(200.0);
    stepper1.setAcceleration(100.0);
    stepper1.moveTo(24);*/
    
    stepper2.setMaxSpeed(800.0);
    stepper2.setAcceleration(100.0);
    move(&stepper2,10000);
    
    /*stepper3.setMaxSpeed(300.0);
    stepper3.setAcceleration(100.0);
    stepper3.moveTo(1000000); */
}

void move(AccelStepper* stepper, long relative) {
  if (relative < 0) {
    origin = origin - (stepper->currentPosition() + relative);
    stepper->setCurrentPosition(-relative);
  }
  stepper->move(relative);
}

void loop()
{
    // Change direction at the limits
    if (stepper2.distanceToGo() == 0) {
      if (clockwise) {
        move(&stepper2,-2*MOVEMENT);
      } else {
        move(&stepper2,2*MOVEMENT);
      }
      clockwise = !clockwise;
    }

    stepper2.run();
   // stepper3.run();
}
