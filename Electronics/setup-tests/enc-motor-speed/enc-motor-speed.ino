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

/* Driving a stepper motor with AccelStepper library
 *
 * This example shows:
 * - how to properly use a stepper motor powered by a driver (e.g., A4988, DRV8825)
 * - use the encoder on the RAMPS Smart Controller
 * - setup the LCD display on the RAMPS Smart Controller
 */

/* Stepper motors */
#include <AccelStepper.h>
/* Smart Controller LCD */
#include <SPI.h>
#include <U8g2lib.h>
/* Rotary encoder */
#include <Encoder.h>

#include "pins.h"
#include "logo.h"

/* Rotary encoder (RAMPS Smart Controller) */
Encoder enc(ENC1,ENC2);
/* Stepper motors */
AccelStepper motorX(AccelStepper::DRIVER, X_STEP_PIN, X_DIR_PIN);
/* LCD graphics controller */
U8G2_ST7920_128X64_1_SW_SPI u8g2(U8G2_R0, LCD_SCK , LCD_MOSI, LCD_CS);

int enc_curr_state;
int enc_prev_state;
int speed = 500;

void setup() {
  //Serial.begin(9600);

  /* Setup LED */
  pinMode(LED_PIN, OUTPUT);

  /* Rotary encoder */
  enc_prev_state = enc.read();

  /* Stepper motors
   *
   * NOTE: `enableOutputs()` is called automatically by the AccelStepper
   * constructor, but since we are resetting the enable pin and its
   * polarity, it is necessary to call it again afterwards.
   *
   * NOTE: During our tests we registered a maximum speed of ~15_000
   * microsteps per second. This should be the maximum speed we can
   * expect across all motors combined
   */
  motorX.setEnablePin(X_ENABLE_PIN);
  motorX.setPinsInverted(false,false,true);
  motorX.enableOutputs();

  motorX.setMaxSpeed(15000);
  motorX.setSpeed(speed);

  /* Splash screen */
  u8g2.begin();

  do {
    u8g2.setFont(u8g2_font_6x13_tr);
    u8g2.setBitmapMode(1); // Transparent mode
    u8g2.drawXBM(22, 2, OXRAM_WIDTH, OXRAM_HEIGHT, oxram_logo_bits);
  } while ( u8g2.nextPage() );
  delay(2000);
}

void loop() {
  /* Update encoder state */
  enc_curr_state = enc.read();

  /* Run motors
   * 
   * Use encoder rotations (scaled by 100) to change the motor's speed.
   */
  speed += 100*(enc_curr_state - enc_prev_state);
  motorX.setSpeed(speed);
  motorX.runSpeed();

  enc_prev_state = enc_curr_state; 
  }
}
