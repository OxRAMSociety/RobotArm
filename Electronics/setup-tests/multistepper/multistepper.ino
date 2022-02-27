/* Stepper motors */
#include <AccelStepper.h>
/* Smart Controller LCD */
#include <SPI.h>
#include <U8g2lib.h>

#include "pins.h"
#include "logo.h"

#define MAX_SPEED 1000

/* Stepper motors */
MultiStepper motors();
AccelStepper motorX(AccelStepper::DRIVER, X_STEP_PIN, X_DIR_PIN);
AccelStepper motorY(AccelStepper::DRIVER, Y_STEP_PIN, Y_DIR_PIN);
AccelStepper motorZ(AccelStepper::DRIVER, Z_STEP_PIN, Z_DIR_PIN);
AccelStepper motorE(AccelStepper::DRIVER, E_STEP_PIN, E_DIR_PIN);
AccelStepper motorQ(AccelStepper::DRIVER, Q_STEP_PIN, Q_DIR_PIN);
AccelStepper motorA(AccelStepper::DRIVER, A_STEP_PIN, A_DIR_PIN);
motors.addStepper(&motorX)
motors.addStepper(&motorY)
motors.addStepper(&motorZ)
motors.addStepper(&motorE)
motors.addStepper(&motorQ)
motors.addStepper(&motorA)

/* LCD graphics controller */
U8G2_ST7920_128X64_1_SW_SPI u8g2(U8G2_R0, LCD_SCK , LCD_MOSI, LCD_CS);

void setup() {
  pinMode(LED_PIN, OUTPUT);

  /* Rotary encoder */
  //pinMode(ENC1, INPUT);
  //pinMode(ENC2, INPUT);
  //enc_prev_state = digitalRead(ENC1);

  /* Stepper motors */
  motorX.setEnablePin(X_ENABLE_PIN);
  motorX.setMaxSpeed(MAX_SPEED);
  motorY.setEnablePin(Y_ENABLE_PIN);
  motorY.setMaxSpeed(MAX_SPEED);
  motorZ.setEnablePin(Z_ENABLE_PIN);
  motorZ.setMaxSpeed(MAX_SPEED);
  motorE.setEnablePin(E_ENABLE_PIN);
  motorE.setMaxSpeed(MAX_SPEED);
  motorQ.setEnablePin(Q_ENABLE_PIN);
  motorQ.setMaxSpeed(MAX_SPEED);
  motorA.setEnablePin(A_ENABLE_PIN);
  motorA.setMaxSpeed(MAX_SPEED);
  motors.moveTo({...});

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
  /* Run motors */
  motors.run();

  /* Refresh screen */
  u8g2.firstPage();
  do {
    u8g2.clear();
    u8g2.setCursor(0,15);
    //u8g2.print("Current speed: ");
    //u8g2.print(speed);
    //u8g2.print("\n");
    //u8g2.print("Current pos: ");
    //u8g2.print(motorX.currentPosition());
    //u8g2.print("\n");
  } while ( u8g2.nextPage() );
}
