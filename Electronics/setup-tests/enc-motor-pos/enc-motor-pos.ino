/*
 * MIT License
 * 
 * Copyright (c) 2020-2022 OxRAM Society
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

/* Driving multiple stepper motors with AccelStepper library
 *
 * This example shows:
 * - how to setup multiple stepper motors with AccelStepper
 * - how to use the encoder to update the target position of the motors
 */

/* Stepper motors */
#include <MultiStepper.h>
#include <AccelStepper.h>
/* Smart Controller LCD */
#include <SPI.h>
#include <U8g2lib.h>
/* Rotary encoder */
#include <Encoder.h>

#include "pins.h"
#include "logo.h"

#define MAX_SPEED 1500

/* Rotary encoder (RAMPS Smart Controller) */
Encoder enc(ENC1,ENC2);
int enc_curr_state;
int enc_prev_state;

/* Stepper motors */
MultiStepper motors;
AccelStepper motorX(AccelStepper::DRIVER, X_STEP_PIN, X_DIR_PIN);
AccelStepper motorY(AccelStepper::DRIVER, Y_STEP_PIN, Y_DIR_PIN);
//AccelStepper motorZ(AccelStepper::DRIVER, Z_STEP_PIN, Z_DIR_PIN);
//AccelStepper motorE(AccelStepper::DRIVER, E_STEP_PIN, E_DIR_PIN);
//AccelStepper motorQ(AccelStepper::DRIVER, Q_STEP_PIN, Q_DIR_PIN);
//AccelStepper motorA(AccelStepper::DRIVER, A_STEP_PIN, A_DIR_PIN);

/* LCD graphics controller */
U8G2_ST7920_128X64_1_SW_SPI u8g2(U8G2_R0, LCD_SCK , LCD_MOSI, LCD_CS);

/* Motors target position */
long positions[2];
long pos = 0;

/* Setup a single stepper motor */
void setup_motor(AccelStepper* motor, uint8_t pin) {
  motor->setEnablePin(pin);
  motor->setPinsInverted(false,false,true);
  motor->setMaxSpeed(MAX_SPEED);
  motor->enableOutputs(); // Don't forget this!
}

void setup() {
  //Serial.begin(9600);

  /* Setup LED */
  //pinMode(LED_PIN, OUTPUT);

  /* Rotary encoder */
  enc_prev_state = enc.read();

  /* Stepper motors */
  setup_motor(&motorX, X_ENABLE_PIN);
  motors.addStepper(&motorX);
  setup_motor(&motorY, Y_ENABLE_PIN);
  motors.addStepper(&motorY);

  /* Set motors target */
  positions[0] = pos;
  positions[1] = pos;
  motors.moveTo(positions);

  /* Splash screen */
  u8g2.begin();
  do {
    u8g2.setFont(u8g2_font_6x13_tr);
    u8g2.setBitmapMode(1); // Transparent mode
    u8g2.drawXBM(22, 2, OXRAM_WIDTH, OXRAM_HEIGHT, oxram_logo_bits);
  } while ( u8g2.nextPage() );
  delay(1000);
}

void loop() {
  /* Update encoder state */
  enc_curr_state = enc.read();
  /* Update target position */
  pos += 10*(enc_curr_state - enc_prev_state);
  enc_prev_state = enc_curr_state;
  positions[0] = pos;
  positions[1] = pos;
  motors.moveTo(positions);

  /* Run motors (at most one step per motor) */
  motors.run();
}
