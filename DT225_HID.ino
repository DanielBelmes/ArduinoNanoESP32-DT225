#ifndef ARDUINO_USB_MODE
#error This ESP32 SoC has no Native USB interface
#elif ARDUINO_USB_MODE == 1
#warning This sketch should be used when USB is in OTG mode
void setup() {}
void loop() {}
#else
#include "USB.h"
#include "USBHIDMouse.h"
#include "esp32-hal-gpio.h"
#include "hal/gpio_hal.h"
#include "hal/gpio_types.h"
#include "soc/gpio_struct.h"
#include "hal/gpio_ll.h"
USBHIDMouse Mouse;

/* ================================================================================
   Original code by GuilleAcoustic
   ================================================================================
   Modifier: Rocco16v 
   Date    : 2025-01-27
   Revision: V1.4
   Purpose : Opto-mechanical trackball firmware for CH Products DT225. This firmware 
              has been written with the intent to override the PIC microcontroller in
              the device. Provides access to the 4th button as a function button thus
              creating an additional "layer" for the 3 main buttons. It adds acceleration 
              to allow usage on high res screens while keeping precision on small movements.
              The original Arduino Mouse library (Mouse.h and Mouse.cpp) have been modified
              to allow up to 8 buttons and horizontal scrolling.
   --------------------------------------------------------------------------------
   Wiring informations: Sparkfun Pro Micro (Atmega32u4)
   --------------------------------------------------------------------------------
     - Gnd                          |   Pin: Gnd [GND]
     - Vcc (+5V)                    |   Pin: Vcc [Vcc]
     - X axis encoder / channel A   |   Pin: PD3 [TXO]   (INT0)
     - X axis encoder / channel B   |   Pin: PD2 [RXI]   (INT1)
     - Y axis encoder / channel A   |   Pin: PD0 [3]     (INT2)
     - Y axis encoder / channel B   |   Pin: PD1 [2]     (INT3)
     - Switch 1                     |   Pin: PB3 [9]
     - Switch 2                     |   Pin: PB2 [10]
     - Switch 3                     |   Pin: PB1 [11]
     - Switch 4                     |   Pin: PB4 [12]
   ================================================================================ */

// =================================================================================
// Type definition
// =================================================================================

#ifndef DEBOUNCE_THREASHOLD
#define DEBOUNCE_THREASHOLD 50
#endif

// =================================================================================
// Structure definition
// =================================================================================
typedef struct ENCODER_ {
  int8_t coordinate;
  uint8_t index;
} ENCODER_;

typedef struct BUTTON_ {
  int state;
  boolean needUpdate;
  uint8_t button;
  uint8_t fnButton;
  int pin;
  long lastDebounceTime;
} BUTTON_;

// =================================================================================
// Constant definition
// =================================================================================
const int8_t lookupTable[] = { 0, 1, -1, 0, -1, 0, 0, 1, 1, 0, 0, -1, 0, -1, 1, 0 };

// =================================================================================
// Volatile variables
// =================================================================================
volatile ENCODER_ xAxis = { 0, 0 };
volatile ENCODER_ yAxis = { 0, 0 };
volatile int Acceleration_X = 3;
volatile int Acceleration_Y = 3;

// =================================================================================
// Global variables
// =================================================================================
BUTTON_ leftButton = { false, false, MOUSE_LEFT, MOUSE_BACKWARD, 11, 0 };
BUTTON_ middleButton = { false, false, MOUSE_MIDDLE, MOUSE_LEFT, 9, 0 };
BUTTON_ rightButton = { false, false, MOUSE_RIGHT, MOUSE_FORWARD, 12, 0 };
BUTTON_ fnButton = { false, false, MOUSE_LEFT,MOUSE_LEFT, 10, 0 };
#define INT0_PIN  0
#define INT1_PIN  1
#define INT2_PIN  2
#define INT3_PIN  3

// =================================================================================
// Setup function
// =================================================================================
void setup() {
  // Attach interruption to encoders channels
  pinMode(INT0_PIN, INPUT);
  pinMode(INT1_PIN, INPUT);
  pinMode(INT2_PIN, INPUT);
  pinMode(INT3_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(INT0_PIN), ISR_HANDLER_Y, CHANGE);
  attachInterrupt(digitalPinToInterrupt(INT1_PIN), ISR_HANDLER_Y, CHANGE);
  attachInterrupt(digitalPinToInterrupt(INT2_PIN), ISR_HANDLER_X, CHANGE);
  attachInterrupt(digitalPinToInterrupt(INT3_PIN), ISR_HANDLER_X, CHANGE);
  pinMode(leftButton.pin, INPUT);
  pinMode(middleButton.pin, INPUT);
  pinMode(rightButton.pin, INPUT);
  pinMode(fnButton.pin, INPUT);

  Serial.begin(115200);

  // Start the mouse function
  Mouse.begin();
  
}

// =================================================================================
// Main program loop
// =================================================================================
void loop() {
  if (fnButton.state && (xAxis.coordinate != 0 || yAxis.coordinate != 0)) {
    // Update the Vertical Scroll Wheel Position. Multiply by -1 to correct the direction of scroll
    Mouse.move(0, 0, yAxis.coordinate * -1, xAxis.coordinate);
    
    xAxis.coordinate = 0;
    yAxis.coordinate = 0;

  } else {
    // Update mouse coordinates
    if (xAxis.coordinate != 0 || yAxis.coordinate != 0) {
      Mouse.move(xAxis.coordinate * Acceleration_X, yAxis.coordinate * Acceleration_Y, 0);
      xAxis.coordinate = 0;
      yAxis.coordinate = 0;
    }
  }

  // ---------------------------------
  // Left mouse button state update
  // ---------------------------------
  ReadButton(leftButton); 
  //UpdateButton(leftButton);

  // ---------------------------------
  // Right mouse button state update
  // ---------------------------------
  ReadButton(rightButton);
  //UpdateButton(rightButton);

  // ---------------------------------
  // Middle mouse button state update
  // ---------------------------------
  ReadButton(middleButton);
  //UpdateButton(middleButton);

  // ---------------------------------
  // Scroll mouse button state update
  // ---------------------------------
  //ReadButton(fnButton);

  delay(3);
}

// =================================================================================
// Interrupt handlers
// =================================================================================
void ISR_HANDLER_X() {                  // wait for a second
  // Build the LUT index from previous and new data
  // X is pin 2, 3
  // .index = 0b0000
  // where first two are the last a, b values and last 2 are current
  uint8_t ab = gpio_ll_get_level(&GPIO, GPIO_NUM_6) << 1 | gpio_ll_get_level(&GPIO, GPIO_NUM_5) << 0;
  xAxis.index = (xAxis.index << 2) | (ab >> 0);
  xAxis.coordinate += lookupTable[xAxis.index & 0b00001111];
}

void ISR_HANDLER_Y() {
  // Build the LUT index from previous and new data
  uint8_t ab = gpio_ll_get_level(&GPIO, GPIO_NUM_43) << 1 | gpio_ll_get_level(&GPIO, GPIO_NUM_44) << 0;
  yAxis.index = (yAxis.index << 2) | (ab  >> 0);
  yAxis.coordinate += lookupTable[yAxis.index & 0b00001111];
}

// =================================================================================
// Functions
// =================================================================================
void ReadButton(BUTTON_& button) {
  int switchState;

  // Get current switch state
  switchState = digitalRead(button.pin);

  if (switchState == LOW) {
    // if the mouse is not pressed, press it:
    if (!Mouse.isPressed(button.button)) {
      Mouse.press(button.button);
    }
  }
  // else the mouse button is not pressed:
  else {
    // if the mouse is pressed, release it:
    if (Mouse.isPressed(button.button)) {
      Mouse.release(button.button);
    }
  }
}

#endif