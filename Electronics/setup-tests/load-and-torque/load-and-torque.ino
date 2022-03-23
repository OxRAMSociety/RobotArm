/*
 * MIT License
 * 
 * Copyright (c) 2020-2022 OxRAM Society
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the &quot;Software&quot;), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED &quot;AS IS&quot;, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

/* Testing torque and the ability of the arm to sustain it's own weight */

/* Stepper motors */
#include <AccelStepper.h>
/* Smart Controller LCD */
#include <SPI.h>
#include <U8g2lib.h>
/* Rotary encoder */
#include <Encoder.h>

#include "pins.h";
#include "logo.h";

/* Rotary encoder (RAMPS Smart Controller) */
Encoder enc(ENC1,ENC2);
/* Stepper motors */
AccelStepper motorX(AccelStepper::DRIVER, X_STEP_PIN, X_DIR_PIN);
AccelStepper motorY(AccelStepper::DRIVER, Y_STEP_PIN, Y_DIR_PIN);
AccelStepper motorZ(AccelStepper::DRIVER, Z_STEP_PIN, Z_DIR_PIN);

/* LCD graphics controller */
U8G2_ST7920_128X64_1_SW_SPI u8g2(U8G2_R0, LCD_SCK , LCD_MOSI, LCD_CS);

int enc_curr_state;
int enc_prev_state;
int speed = 500;
int acceleration = 50, positions[] = {0, 4800}, pos_index = 1, serial_counter = 0, acc_change = 0;

void setup() {
  Serial.begin(9600);

  /* Setup LED */
  pinMode(LED_PIN, OUTPUT);

  /* Rotary encoder */
  enc_prev_state = enc.read();

  motorX.setEnablePin(X_ENABLE_PIN);
  motorX.setPinsInverted(false,false,true);
  motorX.enableOutputs();
  motorX.setMaxSpeed(speed);
  motorX.setAcceleration(acceleration);

  motorY.setEnablePin(Y_ENABLE_PIN);
  motorY.setPinsInverted(false,false,true);
  motorY.enableOutputs();
  motorY.setMaxSpeed(speed);
  motorY.setAcceleration(acceleration);

  /* A third motor is enabled to keep the arm straight */
  motorZ.setEnablePin(Z_ENABLE_PIN);
  motorZ.setPinsInverted(false,false,true);
  motorZ.enableOutputs();

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

  /* Update acceleration
   *
   * NOTE: setting acceleration for a motor is an expensive task and
   * should be done only when necessary.
   */
  acc_change = (enc_curr_state - enc_prev_state);
  if (acc_change != 0) {
    acceleration += acc_change;
    motorX.setAcceleration(acceleration);
    motorY.setAcceleration(acceleration);
  }
  enc_prev_state = enc_curr_state; 

  /* Update target and run */
  motorX.moveTo(positions[pos_index]);
  motorY.moveTo(positions[pos_index]);
  motorX.run();
  motorY.run();

  if (serial_counter%1000 == 1) {
    Serial.println(acceleration);
  }
  serial_counter++;

  /* Compute new target */
  if (!motorX.isRunning() && !motorY.isRunning())
  {
    pos_index += 1;
    pos_index = pos_index % 2;
  }
}
