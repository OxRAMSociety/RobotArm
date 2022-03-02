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

void setup_motor(AccelStepper* motor, uint8_t pin) {
  motor->setEnablePin(pin);
  motor->setPinsInverted(false,false,true);
  motor->setMaxSpeed(MAX_SPEED);
}

void setup() {
  pinMode(LED_PIN, OUTPUT);

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

  //motors.moveTo({...});

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
  //motors.run();
}
