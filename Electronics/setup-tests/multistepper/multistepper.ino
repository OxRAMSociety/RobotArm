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
 * - how to properly setup multiples stepper motors with the AccelStepper library
 * - set target positions for multiple steppers at once
 */

/* Stepper motors */
#include <MultiStepper.h>
#include <AccelStepper.h>
/* Smart Controller LCD */
#include <SPI.h>
#include <U8g2lib.h>

#include "pins.h"
#include "logo.h"

#define MAX_SPEED 1500

/* Stepper motors */
MultiStepper motors;
AccelStepper motorX(AccelStepper::DRIVER, X_STEP_PIN, X_DIR_PIN);
AccelStepper motorY(AccelStepper::DRIVER, Y_STEP_PIN, Y_DIR_PIN);
AccelStepper motorZ(AccelStepper::DRIVER, Z_STEP_PIN, Z_DIR_PIN);
AccelStepper motorE(AccelStepper::DRIVER, E_STEP_PIN, E_DIR_PIN);
AccelStepper motorQ(AccelStepper::DRIVER, Q_STEP_PIN, Q_DIR_PIN);
AccelStepper motorA(AccelStepper::DRIVER, A_STEP_PIN, A_DIR_PIN);

/* LCD graphics controller */
U8G2_ST7920_128X64_1_SW_SPI u8g2(U8G2_R0, LCD_SCK , LCD_MOSI, LCD_CS);

/* Positions */
size_t pos = 0;
long positions[] = {
  180,   11,  201,   -7,  675,  -74,
  927,  374,   33,  426,  414,  219,
   84, 1109,  654,  863,  237,  840,
    8,   -3, -273,  -84,  506,  567,
    7, -929,  704,   63,  247,   80,
   77,  784,  106,  742,   91,  847,
  431,  -51,  950,   71,  500,    3,
 -144,  892,   46,  467,  373,   10,
  562,   82,  974,   48,   67,  575,
   86,   71,   99,  728,    1,   89,
    0,    0,    0,    0,    0,    0, NULL
};

/* Setup a single stepper motor */
void setup_motor(AccelStepper* motor, uint8_t pin) {
  motor->setEnablePin(pin);
  motor->setPinsInverted(false,false,true);
  motor->setMaxSpeed(MAX_SPEED);
  motor->enableOutputs(); // Don't forget this!
}

void setup() {
  //pinMode(LED_PIN, OUTPUT);

  /* Stepper motors */
  setup_motor(&motorX, X_ENABLE_PIN);
  motors.addStepper(&motorX);
  setup_motor(&motorY, Y_ENABLE_PIN);
  motors.addStepper(&motorY);
  setup_motor(&motorZ, Z_ENABLE_PIN);
  motors.addStepper(&motorZ);
  setup_motor(&motorE, E_ENABLE_PIN);
  motors.addStepper(&motorE);
  setup_motor(&motorQ, Q_ENABLE_PIN);
  motors.addStepper(&motorQ);
  setup_motor(&motorA, A_ENABLE_PIN);
  motors.addStepper(&motorA);

  motors.moveTo(positions[pos]);

  /* Splash screen */
  u8g2.begin();
  u8g2.setFont(u8g2_font_6x13_tr);
  do {
    u8g2.setBitmapMode(1); // Transparent mode
    u8g2.drawXBM(22, 2, OXRAM_WIDTH, OXRAM_HEIGHT, oxram_logo_bits);
  } while ( u8g2.nextPage() );
  delay(2000);
}

void loop() {
  /* `run()` returns false when all motors in the MultiStepper object
   * have reached their target.
   */
  if (!motors.run()) {
    /* Compute new target position */
    if ((pos+=6) == NULL) {
        pos = 0;
    }
    motors.moveTo(positions[pos]);
    /* Update LCD display */
    u8g2.firstPage();
    do {
      //u8g2.clear();
      u8g2.setCursor(0,15);
      u8g2.print("Current target:");
      u8g2.setCursor(0,30);
      u8g2.print(positions[pos]);
      u8g2.print(", ");
      u8g2.print(positions[pos+1]);
      u8g2.print(", ");
      u8g2.print(positions[pos+2]);
      u8g2.setCursor(0,30);
      u8g2.print(positions[pos+3]);
      u8g2.print(", ");
      u8g2.print(positions[pos+4]);
      u8g2.print(", ");
      u8g2.print(positions[pos+5]);
    } while ( u8g2.nextPage() );
    delay(1000);
  }
}
