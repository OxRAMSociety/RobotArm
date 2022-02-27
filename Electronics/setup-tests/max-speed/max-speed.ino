/* Stepper motors */
#include <AccelStepper.h>
/* Smart Controller LCD */
#include <SPI.h>
#include <U8g2lib.h>

#include "pins.h"
#include "logo.h"

/* Stepper motors */
AccelStepper motorX(AccelStepper::DRIVER, X_STEP_PIN, X_DIR_PIN);

/* LCD graphics controller */
U8G2_ST7920_128X64_1_SW_SPI u8g2(U8G2_R0, LCD_SCK , LCD_MOSI, LCD_CS);

bool clockwise = true;
int enc_curr_state;
int enc_prev_state;
int speed = 0;
int accel = 10;

void setup() {
  /* Setup buzzer */
  //pinMode(BUZZER_PIN, OUTPUT);
  //digitalWrite(BUZZER_PIN, LOW);
  pinMode(LED_PIN, OUTPUT);

  /* Rotary encoder */
  pinMode(ENC1, INPUT);
  pinMode(ENC2, INPUT);
  enc_prev_state = digitalRead(ENC1);

  /* Stepper motors */
  motorX.setEnablePin(X_ENABLE_PIN);
  motorX.setMaxSpeed(10000);
  motorX.setSpeed(500);

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
  /* A rotation on the rotaty encoder is detected by a change of
   * state in the CLK pin */
  enc_curr_state = digitalRead(ENC1);
  if (enc_curr_state != enc_prev_state) { 
    /* Rotation is (counter)clockwise if the DT input matches (is
     * different than) the CLK pin */
    clockwise = digitalRead(ENC2) == enc_curr_state;
    digitalWrite(LED_PIN, HIGH);
    speed += ((clockwise) ? 1 : -1) * accel;
    motorX.setSpeed(speed);
  } else {
    digitalWrite(LED_PIN, LOW);
  }
  enc_prev_state = enc_curr_state; 

  /* Run motors */
  motorX.runSpeed();

  /* Refresh screen */
  u8g2.firstPage();
  do {
    u8g2.clear();
    u8g2.setCursor(0,15);
    u8g2.print("Current speed: ");
    u8g2.print(speed);
    u8g2.print("\n");
    u8g2.print("Current pos: ");
    u8g2.print(motorX.currentPosition());
    u8g2.print("\n");
  } while ( u8g2.nextPage() );
}
