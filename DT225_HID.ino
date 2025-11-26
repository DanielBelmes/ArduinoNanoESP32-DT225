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
#include "hal/misc.h"
USBHIDMouse Mouse;

/* ================================================================================
   Original code by GuilleAcoustic and Rocco16v
   ================================================================================
   Modifier: DanielBelmes 
   Date    : 2025-11-25
   Revision: V1.5
   Purpose : Opto-mechanical trackball firmware for CH Products DT225. This firmware 
              has been written with the intent to override the PIC microcontroller in
              the device.
   --------------------------------------------------------------------------------
   Wiring informations: Ardunio Nano ESP32
   --------------------------------------------------------------------------------
     - Gnd                          |   Pin: Gnd [GND]
     - Vcc (+5V)                    |   Pin: Vcc [Vcc]
     - X axis encoder / channel A   |   Pin: GPIO43 [TXO]   (INT0)
     - X axis encoder / channel B   |   Pin: GPIO44 [RX0]   (INT1)
     - Y axis encoder / channel A   |   Pin: GPIO6 [3]     (INT2)
     - Y axis encoder / channel B   |   Pin: GPIO5 [2]     (INT3)
     - Switch 1                     |   Pin: GPIO18 [9]
     - Switch 2                     |   Pin: GPIO21 [10]
     - Switch 3                     |   Pin: GPIO38 [11]
     - Switch 4                     |   Pin: GPIO47 [12]
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
  boolean state;
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
//volatile int Acceleration_X = 1;
//volatile int Acceleration_Y = 1;

// =================================================================================
// Global variables
// =================================================================================
BUTTON_ leftButton = { false, false, MOUSE_LEFT, MOUSE_BACKWARD, 11, 0 };
BUTTON_ middleButton = { false, false, MOUSE_MIDDLE, 0, 9, 0 };
BUTTON_ rightButton = { false, false, MOUSE_RIGHT, MOUSE_FORWARD, 12, 0 };
BUTTON_ fnButton = { false, false, 0,0, 10, 0 };
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
  attachInterrupt(digitalPinToInterrupt(INT0_PIN), ISR_HANDLER_Y, RISING);
  attachInterrupt(digitalPinToInterrupt(INT1_PIN), ISR_HANDLER_Y, RISING);
  attachInterrupt(digitalPinToInterrupt(INT2_PIN), ISR_HANDLER_X, RISING);
  attachInterrupt(digitalPinToInterrupt(INT3_PIN), ISR_HANDLER_X, RISING);
  pinMode(leftButton.pin, INPUT_PULLDOWN);
  pinMode(middleButton.pin, INPUT_PULLDOWN);
  pinMode(rightButton.pin, INPUT_PULLDOWN);
  pinMode(fnButton.pin, INPUT_PULLDOWN);

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
      Mouse.move(xAxis.coordinate, yAxis.coordinate, 0);
      xAxis.coordinate = 0;
      yAxis.coordinate = 0;
    }
  }

  // ---------------------------------
  // Left mouse button state update
  // ---------------------------------
  ReadButton(leftButton); 

  // ---------------------------------
  // Right mouse button state update
  // ---------------------------------
  ReadButton(rightButton);

  // ---------------------------------
  // Middle mouse button state update
  // ---------------------------------
  ReadButton(middleButton);

  // ---------------------------------
  // Scroll mouse button state update
  // ---------------------------------
  ReadButton(fnButton);

  delay(3);
}

static inline int gpio_ll_get_bitmask(gpio_dev_t *hw, uint32_t gpio_mask, boolean greaterThan32)
{
    if (!greaterThan32) {
        return hw->in & gpio_mask;
    } else {
        return HAL_FORCE_READ_U32_REG_FIELD(hw->in1, data) & gpio_mask;
    }
}

// =================================================================================
// Interrupt handlers
// =================================================================================
void ISR_HANDLER_X() {
  // Build the LUT index from previous and new data
  // X is pin 2, 3
  // .index = 0b0000
  // where first two are the last a, b values and last 2 are current
  uint8_t ab = gpio_ll_get_bitmask(&GPIO, (1ULL << 6) | (1ULL << 5), false) >> 5; //Read Input Matrix <GPIO32 and mask out only GPIO5,6 as they correspond to 2 and 3
  xAxis.index = (xAxis.index << 2) | ab;
  xAxis.coordinate += lookupTable[xAxis.index & 0b00001111];
}

void ISR_HANDLER_Y() {
  // Build the LUT index from previous and new data
  uint8_t ab = gpio_ll_get_bitmask(&GPIO, (1ULL << GPIO_NUM_44-32) | (1ULL << GPIO_NUM_43-32), true) >> GPIO_NUM_43-32; //Read Input Matrix>GPIO32 and mask out only GPIO43,44 as they correspond to tx0 and rx0
  yAxis.index = (yAxis.index << 2) | ab;
  yAxis.coordinate -= lookupTable[yAxis.index & 0b00001111]; //had to invert Y because I have the wiring backwards :(
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
      button.state = true;
    }
  }
  // else the mouse button is not pressed:
  else {
    // if the mouse is pressed, release it:
    if (Mouse.isPressed(button.button) || button.state) {
      Mouse.release(button.button);
      button.state = false;
    }
  }
}

#endif