#include <Bluepad32.h>
#include <MotorController.h>

using namespace MotorControl;

// Define motor control pins
#define R_MOTOR_IN1 19         // TB6612FNG IN1 pin
#define R_MOTOR_IN2 18         // TB6612FNG IN2 pin
#define R_MOTOR_PWM 5          // TB6612FNG PWM input pin (PWMA or PWMB)
#define L_MOTOR_IN1 17         // TB6612FNG IN1 pin
#define L_MOTOR_IN2 16         // TB6612FNG IN2 pin
#define L_MOTOR_PWM 4          // TB6612FNG PWM input pin (PWMA or PWMB)
#define DPAD_UP 0X01           // D-PAD UP button
#define DPAD_DOWN 0X02         // D-PAD DOWN button
#define DPAD_LEFT 0X04         // D-PAD LEFT button
#define DPAD_RIGHT 0X08        // D-PAD RIGHT button
#define DPAD_DEFAULT 0X00      // D-PAD DEFAULT value
#define TRIANGLE 0X0008        // TRIANGLE button
#define CIRCLE 0X0002          // CIRCLE button
#define CROSS 0X0001           // CROSS button
#define SQUARE 0X0004          // SQUARE button
#define ACTIONS_DEFAULT 0X0000 // ACTIONS DEFAULT value
#define LED_CONNECTED 2        // LED pin for connected gamepad

// Define motor control parameters
const uint16_t FREQUENCY = 10000;      // PWM frequency 10kHz
const uint8_t RESOLUTION = 10;         // PWM resolution 10 bits (0-1023)
const uint8_t CHANNEL = 0;             // PWM channel number (0-15)
const uint16_t MAX_PWM = 1023;         // Maximum speed (0-1023)
const uint8_t MIN_PWM = 150;           // Minimum speed (0-1023)
const uint8_t MOTORS_OFFSET = 150;     // Motor driver offset 170 to start

// Define gamepad parameters
const uint8_t FORGET_GAMEPAD_PIN = 13; // Forget gamepad pin (input)
const uint16_t MAX_THROTTLE = 1020;    // Maximum throttle value (0-1020)
const uint8_t MIN_THROTTLE = 0;        // Minimum throttle value (0-1020)
const uint16_t MAX_BRAKE = 1020;       // Maximum brake value (0-1020)
const uint8_t MIN_BRAKE = 0;           // Minimum brake value (0-1020)
const int16_t MIN_LEFT_STICK_X = -508;  // Minimum left stick X-axis value (0-(-508))
const uint16_t MAX_LEFT_STICK_X = 512;   // Maximum left stick X-axis value (0-512)
const uint8_t LEFT_STICK_X_OFFSET = 4;   // Left stick offset X value = 4
const uint8_t DEADBAND_VALUE = 20;   // Deadband value for stick, throttle, and brake

// Define gamepad variables
uint16_t throttle_value = 0;           // Throttle value (0-1020)
uint16_t brake_value = 0;              // Brake value (0-1020)
int16_t left_stick_x = 0;              // Left stick X-axis value (0-1020)
uint16_t dpad = 0x00;                  // D-PAD value (0x00-0x08)

// Exponential steering variables
const float STEERING_EXPONENT = 2.0; // Change this value to adjust the steering sensitivity. Higher values make steering more precise to center, more agressive at the extremes
const uint8_t MAX_ACCELERATION = 30; // Maximum change in PWM per loop cycle
const uint8_t MAX_STEERING_ACCELERATION = 40; // Maximum change in steering-induced PWM difference per cycle

// Create motor controller objects
MotorController r_motor(R_MOTOR_IN1, R_MOTOR_IN2, R_MOTOR_PWM, CHANNEL);
MotorController l_motor(L_MOTOR_IN1, L_MOTOR_IN2, L_MOTOR_PWM, CHANNEL);

ControllerPtr gamepads[BP32_MAX_GAMEPADS];

enum CONTROL_TYPE {
  RC = 0,
  APP
};

void setup() {
  Serial.begin(115200);                      // Initialize serial communication
  delay(1000);                               // Wait for serial monitor to open. Only use for debugging 
  pinMode(FORGET_GAMEPAD_PIN, INPUT_PULLUP); // Set pin for forgetting gamepad
  pinMode(LED_CONNECTED, OUTPUT);            // Set pin for LED indication

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

  Serial.printf("Firmware: %s\n", BP32.firmwareVersion());
  const uint8_t *addr = BP32.localBdAddress();
  Serial.print("BD Address: ");
  for (int i = 0; i < 6; i++) {
    Serial.print(addr[i], HEX);
    if (i < 5)
      Serial.print(":");
    else
      Serial.println();
  }

  // Setup the Bluepad32 callbacks
  BP32.setup(&onConnectedController, &onDisconnectedController);
  BP32.forgetBluetoothKeys();
  // TODO: Uncomment the following lines to enable gamepad forgetting functionality
  // bool forgetState = digitalRead(FORGET_GAMEPAD_PIN);
  // if(!forgetState) {
  //   Serial.println("Forgetting gamepad...");
  //   BP32.forgetBluetoothKeys();
  // } else {
  //   Serial.println("No gamepad to forget.");
  // }
}

void loop() {
  BP32.update();

  for (int i = 0; i < BP32_MAX_CONTROLLERS; i++) {
    ControllerPtr ps4 = gamepads[i];

    if (ps4 && ps4->isConnected()) {
      if (ps4->isGamepad())
        processGamepad(ps4);
    }
  }

  delay(150);
}

void onConnectedController(ControllerPtr ctl) {
  bool foundEmptySlot = false;
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (gamepads[i] == nullptr) {
      Serial.printf("CALLBACK: Controller is connected, index=%d\n", i);
      digitalWrite(LED_CONNECTED, HIGH); // Turn on LED when a controller is connected

      ControllerProperties properties = ctl->getProperties();
      Serial.printf("Controller model: %s, VID=0x%04x, PID=0x%04x\n", ctl->getModelName().c_str(), properties.vendor_id, properties.product_id);
      gamepads[i] = ctl;
      foundEmptySlot = true;
      break;
    }
  }
  if (!foundEmptySlot) {
    Serial.println("CALLBACK: Controller connected, but could not found empty slot");
  }
}

void onDisconnectedController(ControllerPtr ctl) {
  bool foundController = false;

  r_motor.softStop();
  l_motor.softStop();
  digitalWrite(LED_CONNECTED, LOW); // Turn off LED when a controller is disconnected

  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (gamepads[i] == ctl) {
      Serial.printf("CALLBACK: Controller disconnected from index=%d\n", i);
      gamepads[i] = nullptr;
      foundController = true;
      break;
    }
  }

  if (!foundController) {
    Serial.println("CALLBACK: Controller disconnected, but not found in myControllers");
    r_motor.softStop();
    l_motor.softStop();
    digitalWrite(LED_CONNECTED, LOW); // Turn off LED when a controller is disconnected
  }
}

void processGamepad(ControllerPtr gamepad) {
  throttle_value = gamepad->throttle();
  brake_value = gamepad->brake();
  left_stick_x = gamepad->axisX();
  dpad = gamepad->dpad();

  // Apply deadband to throttle and brake values
  throttle_value = applyDeadband(throttle_value, DEADBAND_VALUE, 0);                                     // Apply deadband to throttle value
  brake_value = applyDeadband(brake_value, DEADBAND_VALUE, 0);                                           // Apply deadband to brake value
  left_stick_x = applyDeadband(left_stick_x, DEADBAND_VALUE, LEFT_STICK_X_OFFSET);                       // Apply deadband to left stick X-axis value
                       
  uint16_t forward_pwm = map(throttle_value, MIN_THROTTLE, MAX_THROTTLE, MIN_PWM, MAX_PWM);              // Map throttle value to PWM range [0 - 1020] to [0 - 1023]
  uint16_t reverse_pwm = map(brake_value, MIN_BRAKE, MAX_BRAKE, MIN_PWM, MAX_PWM);                       // Map brake value to PWM range [0 - 1020] to [0 - 1023]
  int16_t map_left_stick_x = map(left_stick_x, MIN_LEFT_STICK_X, MAX_LEFT_STICK_X, -MAX_PWM, MAX_PWM);   // Map left stick X-axis value to PWM range [0 - 1020] to [-1023 - 1023]

  uint16_t steering_value = applyExponentialSteering(map_left_stick_x, STEERING_EXPONENT, MAX_PWM);      // Apply exponential response to steering for better control feel

  // Determine if we're going forward, backward, or stopped
  int16_t base_pwm = 0;
  base_pwm = getDirection(throttle_value, brake_value, base_pwm); 

  // Apply proportional steering (more effect at higher speeds)
  float speed_factor = static_cast<float>(abs(base_pwm)) / MAX_PWM;
  int16_t steering_pwm = static_cast<int16_t>(steering_value * speed_factor);

  // Calculate the raw PWM values for each motor

  Serial.println("Thtottle: " + String(map_throttle) + ", Brake: " + String(map_brake) + ", Left stick X: " + String(map_left_stick_x));
}

/**
 * @brief This function applies a deadband to the given value by the joystick
 * 
 * @param value: The value given by the joystick
 * @param deadband_threshold: The deadband threshold value 
 * @param center_offset: The center offset value 
 * @return int16_t 
 */
int16_t applyDeadband(int16_t value, int8_t deadband_threshold, int8_t center_offset) {
  // Apply offset correction
  value -= center_offset;

  // Apply deadband
  if(abs(value) < deadband_threshold) {
    return 0; // Deadband applied
  } else {
    // Return the value, keeping the sign but adjusting for the deadband threshold
    // This ensures there's no "jump" when crossing the deadband
    return (value > 0) ? (value - deadband_threshold) : (value + deadband_threshold);
  }
}

/**
 * @brief This function applies an exponential curve to the given value
 * 
 * @param value: The value given by the joystick
 * @param exponent: The exponent value for the exponential curve
 * @param max_value: The maximum value for the joystick
 * @return int16_t 
 */
int16_t applyExponentialSteering(in16_t value, float exponent, int16_t max_value) {
  // Normalize to -1.0 to 1.0
  float normalized_value = static_cast<float>(value) / max_value;
  // Apply exponential curve while keeping the sign
  // float exponential = pow(abs(normalized), exponent) * (normalized >= 0 ? 1.0 : -1.0);
  float exp_value = copysign(pow(abs(normalized_value), exponent), normalized_value);
  // Scale back to original range
  return static_cast<int16_t>(exp_value * max_value);
}

/**
 * @brief This function determines the direction of the motors based on throttle and brake values
 * 
 * @param throttle_value: The value given by the throttle
 * @param brake_value: The value given by the brake
 * @param base_pwm: The base PWM value for the motors
 * @return int16_t 
 */
int16_t getDirection(uint16_t throttle_value, uint16_t brake_value, int16_t base_pwm) {
  if(throttle_value > brake_value) {
    base_pwm = forward_pwm;
  } else if(brake_value > throttle_value) {
    base_pwm = -reverse_pwm;
  } else {
    base_pwm = 0; 
  }

  return base_pwm;
}