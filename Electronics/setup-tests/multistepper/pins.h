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

/* Pin setup */

/* RAMPS 1.4 */
#define X_STEP_PIN         54
#define X_DIR_PIN          55
#define X_ENABLE_PIN       38
//#define X_MIN_PIN         3
//#define X_MAX_PIN         2

#define Y_STEP_PIN         60
#define Y_DIR_PIN          61
#define Y_ENABLE_PIN       56
//#define Y_MIN_PIN        14
//#define Y_MAX_PIN        15

#define Z_STEP_PIN         46
#define Z_DIR_PIN          48
#define Z_ENABLE_PIN       62
//#define Z_MIN_PIN        18
//#define Z_MAX_PIN        19

#define E_STEP_PIN         26
#define E_DIR_PIN          28
#define E_ENABLE_PIN       24

#define Q_STEP_PIN         36
#define Q_DIR_PIN          34
#define Q_ENABLE_PIN       30

/* Additional stepper motor */
#define A_STEP_PIN         42
#define A_DIR_PIN          40
#define A_ENABLE_PIN       44

//#define SDPOWER          -1
//#define SDSS             53
#define LED_PIN            13

//#define FAN_PIN           9

//#define PS_ON_PIN        12
//#define KILL_PIN         -1

//#define HEATER_0_PIN     10
//#define HEATER_1_PIN      8
//#define TEMP_0_PIN       13
//#define TEMP_1_PIN       14

/* REPRAP Discount Smart Controller */
#define LCD_CS             16 // 4th pin from the right (normally marked as RS)
#define LCD_MOSI           17 // 5th pin from the right (normally marked as R/W)
#define LCD_SCK            23 // 6th pin from the right (normally marked as E)

#define LCD_BACKLIGHT_PIN  39
#define BUZZER_PIN         37
#define SD_DETECT_PIN      49 
#define KILL_PIN           41

#define ENC1               31
#define ENC2               33
#define ENC                35

/* Screen size */
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
