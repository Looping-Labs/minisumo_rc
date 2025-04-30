#include <MotorController.h>
using namespace MotorControl;

// Define motor control pins
#define R_MOTOR_IN1 19 // TB6612FNG IN1 pin
#define R_MOTOR_IN2 18 // TB6612FNG IN2 pin
#define R_MOTOR_PWM 5  // TB6612FNG PWM input pin (PWMA or PWMB)
#define L_MOTOR_IN1 17 // TB6612FNG IN1 pin
#define L_MOTOR_IN2 16 // TB6612FNG IN2 pin
#define L_MOTOR_PWM 4  // TB6612FNG PWM input pin (PWMA or PWMB)

const uint16_t FREQUENCY = 10000; // PWM frequency 10kHz
const uint8_t RESOLUTION = 10;    // PWM resolution 10 bits (0-1023)
const uint8_t CHANNEL = 0;        // PWM channel number (0-15)
const uint16_t MAX_SPEED = 1023;  // Maximum speed (0-1023)
const uint16_t MIN_SPEED = 0;     // Minimum speed (0-1023)

// Create motor controller objects
MotorController r_motor(R_MOTOR_IN1, R_MOTOR_IN2, R_MOTOR_PWM, CHANNEL);
MotorController l_motor(L_MOTOR_IN1, L_MOTOR_IN2, L_MOTOR_PWM, CHANNEL);

enum CONTROL_TYPE {
  RC = 0,
  APP
};

void setup() {
  Serial.begin(115200); // Initialize serial communication

  if (!r_motor.begin(FREQUENCY, RESOLUTION)) {
    Serial.println("Failed to initialize right motor controller!");
    while (1)
      ; // Don't continue if initialization failed
  }

  if (!l_motor.begin(FREQUENCY, RESOLUTION)) {
    Serial.println("Failed to initialize left motor controller!");
    while (1)
      ; // Don't continue if initialization failed
  }

  Serial.println("Motors controller initialized successfully");
}

void loop() {
  r_motor.motorGo(MAX_SPEED); // 1023/1023 = 100%
  l_motor.motorGo(MAX_SPEED); // 1023/1023 = 100%
}
