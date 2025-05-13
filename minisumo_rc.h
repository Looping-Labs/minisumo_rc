#pragma once
#include <stdint.h>

// 17 18 19 21 22 23
// Define motor control pins
#define R_MOTOR_IN1   23                  // TB6612FNG IN1 pin
#define R_MOTOR_IN2   22                  // TB6612FNG IN2 pin
#define R_MOTOR_PWM   21                  // TB6612FNG PWM input pin (PWMA or PWMB)
#define L_MOTOR_IN1   17                  // TB6612FNG IN1 pin
#define L_MOTOR_IN2   18                  // TB6612FNG IN2 pin
#define L_MOTOR_PWM   19                  // TB6612FNG PWM input pin (PWMA or PWMB)
#define DPAD_UP       0X01                // D-PAD UP button
#define DPAD_DOWN     0X02                // D-PAD DOWN button
#define DPAD_LEFT     0X04                // D-PAD LEFT button
#define DPAD_RIGHT    0X08                // D-PAD RIGHT button
#define DPAD_DEFAULT  0X00                // D-PAD DEFAULT value
#define TRIANGLE      0X0008              // TRIANGLE button
#define CIRCLE        0X0002              // CIRCLE button
#define CROSS         0X0001              // CROSS button
#define SQUARE        0X0004              // SQUARE button
#define ACT_DEFAULT   0X0000              // ACTIONS DEFAULT value
#define LED_CONNECTED 2                   // LED pin for connected gamepad

// Definiciones para el LED con PWM
const uint8_t LED_PWM_CHANNEL    = 2;     // Canal PWM para el LED (diferente de los motores)
const uint16_t LED_PWM_FREQ      = 5000;  // Frecuencia PWM para el LED
const uint8_t LED_PWM_RESOLUTION = 8;     // Resolución (8 bits: 0-255)
const uint8_t LED_MAX_DUTY       = 255;   // Valor máximo del ciclo de trabajo
const uint8_t LED_FADE_STEP      = 5;     // Incremento en cada paso del fade

// Define motor control parameters  
const uint16_t FREQUENCY = 10000;         // PWM frequency 10kHz
const uint8_t RESOLUTION = 10;            // PWM resolution 10 bits (0-1023)
const uint8_t R_CHANNEL  = 0;             // PWM channel number (0-15)
const uint8_t L_CHANNEL  = 1;             // PWM channel number (0-15)
const uint16_t MAX_PWM   = 1023;          // Maximum speed (0-1023)
const uint8_t MIN_PWM    = 0;             // Minimum speed (0-1023)
const uint8_t MOTOR_START_OFFSET = 150;   // Motor driver offset 170 to start

// Define gamepad parameters
const uint8_t FORGET_GAMEPAD_PIN  = 13;   // Forget gamepad pin (input)
const uint16_t MAX_THROTTLE       = 1020; // Maximum throttle value (0-1020)
const uint8_t MIN_THROTTLE        = 0;    // Minimum throttle value (0-1020)
const uint16_t MAX_BRAKE          = 1020; // Maximum brake value (0-1020)
const uint8_t MIN_BRAKE           = 0;    // Minimum brake value (0-1020)
const int16_t MIN_LEFT_STICK_X    = -508; // Minimum left stick X-axis value (0-(-508))
const uint16_t MAX_LEFT_STICK_X   = 512;  // Maximum left stick X-axis value (0-512)
const uint8_t LEFT_STICK_X_OFFSET = 5;    // Left stick offset X value = 4
const uint8_t DEADBAND_VALUE      = 20;   // Deadband value for stick, throttle, and brake
const float STEERING_EXPONENT     = 2.0;  // Change this value to adjust the steering sensitivity. Higher values make steering more precise to center, more agressive at the extremes

// Define gamepad variables
uint16_t throttle_value = 0;              // Throttle value (0-1020)
int16_t brake_value     = 0;              // Brake value (0-1020)
int16_t left_stick_x    = 0;              // Left stick X-axis value (0-1020)
uint16_t dpad           = 0x00;           // D-PAD value (0x00-0x08)
bool gamepad_connected  = false;          // Gamepad connection status

// Define motor control variables
int16_t left_pwm  = 0;
int16_t right_pwm = 0;

enum ControllerType {
  RC = 0,
  APP
};

enum GamepadStatus {
  DISCONNECTED = 0,
  CONNECTED,
  FORGOTTEN,
};

volatile GamepadStatus gamepad_status = DISCONNECTED;

void onConnectedController(ControllerPtr ctl);
void onDisconnectedController(ControllerPtr ctl);
void processGamepad(ControllerPtr gamepad);
int16_t applyDeadband(int16_t value, int8_t deadband_threshold, int8_t center_offset);
int16_t applyExponentialSteering(int16_t value, float exponent, int16_t max_value);
int16_t getDirection(uint16_t throttle_value, uint16_t brake_value, int16_t base_pwm, uint16_t forward_pwm, uint16_t reverse_pwm);