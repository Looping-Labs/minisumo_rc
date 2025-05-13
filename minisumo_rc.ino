#include <Bluepad32.h>
#include <MotorController.h>
#include "minisumo_rc.h"

using namespace MotorControl;

// Create motor controller objects
MotorController right_motor(R_MOTOR_IN1, R_MOTOR_IN2, R_MOTOR_PWM, R_CHANNEL);
MotorController left_motor(L_MOTOR_IN1, L_MOTOR_IN2, L_MOTOR_PWM, L_CHANNEL);

ControllerPtr gamepads[BP32_MAX_GAMEPADS];

void setup() {
  Serial.begin(115200);                      // Initialize serial communication
  delay(1000);                               // Wait for serial monitor to open. Only use for debugging
  pinMode(FORGET_GAMEPAD_PIN, INPUT_PULLUP); // Set pin for forgetting gamepad
  pinMode(LED_CONNECTED, OUTPUT);            // Set pin for LED indication

  if (!right_motor.begin(FREQUENCY, RESOLUTION)) {
    Serial.println("Failed to initialize right motor controller!");
    while (1)
      ; // Don't continue if initialization failed
  }

  if (!left_motor.begin(FREQUENCY, RESOLUTION)) {
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

  // delay(150);
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

  right_motor.softStop();
  left_motor.softStop();
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
    right_motor.softStop();
    left_motor.softStop();
    digitalWrite(LED_CONNECTED, LOW); // Turn off LED when a controller is disconnected
  }
}

/**
 * @brief This function processes the gamepad input and controls the motors accordingly
 *
 * @param gamepad: The gamepad object
 */
void processGamepad(ControllerPtr gamepad) {
  throttle_value = gamepad->throttle();
  brake_value = gamepad->brake();
  left_stick_x = gamepad->axisX();
  dpad = gamepad->dpad();

  // Apply deadband to throttle and brake values
  throttle_value = applyDeadband(throttle_value, DEADBAND_VALUE, 0);               // Apply deadband to throttle value
  brake_value = applyDeadband(brake_value, DEADBAND_VALUE, 0);                     // Apply deadband to brake value
  left_stick_x = applyDeadband(left_stick_x, DEADBAND_VALUE, LEFT_STICK_X_OFFSET); // Apply deadband to left stick X-axis value

  uint16_t forward_pwm = map(throttle_value, MIN_THROTTLE, MAX_THROTTLE, MIN_PWM, MAX_PWM);            // Map throttle value to PWM range [0 - 1020] to [0 - 1023]
  int16_t reverse_pwm = map(brake_value, MIN_BRAKE, MAX_BRAKE, MIN_PWM, -MAX_PWM);                     // Map brake value to PWM range [0 - 1020] to [0 - 1023]
  int16_t map_left_stick_x = map(left_stick_x, MIN_LEFT_STICK_X, MAX_LEFT_STICK_X, -MAX_PWM, MAX_PWM); // Map left stick X-axis value to PWM range [0 - 1020] to [-1023 - 1023]

  int16_t steering_value = applyExponentialSteering(map_left_stick_x, STEERING_EXPONENT, MAX_PWM); // Apply exponential response to steering for better control feel

  // // Determine if we're going forward, backward, or stopped
  int16_t base_pwm = 0;
  base_pwm = getDirection(throttle_value, brake_value, base_pwm, forward_pwm, reverse_pwm);

  // Apply proportional steering (more effect at higher speeds)
  float speed_factor = static_cast<float>(abs(base_pwm)) / MAX_PWM;
  int16_t scaled_steering = static_cast<int16_t>(steering_value * speed_factor);

  // Calculate the PWM values for each motor
  if (base_pwm > 0) {
    left_pwm = base_pwm + scaled_steering;
    right_pwm = base_pwm - scaled_steering;
  } else {
    left_pwm = base_pwm - scaled_steering;
    right_pwm = base_pwm + scaled_steering;
  }

  // Preserve steering ratio if motors exceed limits
  if(abs(left_pwm) > MAX_PWM || abs(right_pwm) > MAX_PWM) {
    // Determine which motor is exceeding the limit
    float left_excess = static_cast<float>(abs(left_pwm)) / MAX_PWM;
    float right_excess = static_cast<float>(abs(right_pwm)) / MAX_PWM;
    float scale_factor = 1.0 / max(left_excess, right_excess);

    // Scale both motors to maintain the steering ratio
    left_pwm *= scale_factor;
    right_pwm *= scale_factor;
  }

  // Constrain the PWM values to the valid range
  left_pwm = constrain(left_pwm, -MAX_PWM, MAX_PWM);
  right_pwm = constrain(right_pwm, -MAX_PWM, MAX_PWM);

   // TODO: The joystick offset is giving -5.

  // Apply motor start offset for dead zone of motors
  if(left_pwm > 0 && left_pwm < MOTOR_START_OFFSET) {
    left_pwm = MOTOR_START_OFFSET;
  } else if(left_pwm < 0 && left_pwm > -MOTOR_START_OFFSET) {
    left_pwm = -MOTOR_START_OFFSET;
  }
  if(right_pwm > 0 && right_pwm < MOTOR_START_OFFSET) {
    right_pwm = MOTOR_START_OFFSET;
  } else if(right_pwm < 0 && right_pwm > -MOTOR_START_OFFSET) {
    right_pwm = -MOTOR_START_OFFSET;
  }

  // If very small values, just stop the motors completely
  if(abs(base_pwm) < MOTOR_START_OFFSET) {
    left_pwm = 0;
    right_pwm = 0;
  }

  // Control motors using pwm values
  left_motor.motorGo(left_pwm);
  right_motor.motorGo(right_pwm);

  Serial.println("Throttle: " + String(throttle_value) + ", Brake: " + String(brake_value) + ", Left stick X: " + String(map_left_stick_x) + " left_pwm: " + String(left_pwm) + ", right_pwm: " + String(right_pwm));
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
  (value > 0) ? value -= center_offset : value += center_offset;

  // Apply deadband
  if (abs(value) < deadband_threshold) {
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
int16_t applyExponentialSteering(int16_t value, float exponent, int16_t max_value) {
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
int16_t getDirection(uint16_t throttle_value, uint16_t brake_value, int16_t base_pwm, uint16_t forward_pwm, uint16_t reverse_pwm) {
  if (throttle_value > brake_value) {
    base_pwm = forward_pwm;
  } else if (brake_value > throttle_value) {
    base_pwm = reverse_pwm;
  } else {
    base_pwm = 0;
  }
  return base_pwm;
}
