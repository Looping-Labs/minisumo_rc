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
const uint8_t LEFT_STICK_Y_OFFSET = 4;   // Left stick offset Y value = 4

// Define gamepad variables
uint16_t throttle_value = 0;           // Throttle value (0-1020)
uint16_t brake_value = 0;              // Brake value (0-1020)
int16_t left_stick_x = 0;              // Left stick X-axis value (0-1020)
uint16_t dpad = 0x00;                  // D-PAD value (0x00-0x08)

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
                       
  uint16_t map_throttle = map(throttle_value, MIN_THROTTLE, MAX_THROTTLE, MIN_PWM, MAX_PWM);             // Map throttle value to PWM range [0 - 1020] to [0 - 1023]
  uint16_t map_brake = map(brake_value, MIN_BRAKE, MAX_BRAKE, MIN_PWM, MAX_PWM);                         // Map brake value to PWM range [0 - 1020] to [0 - 1023]
  int16_t map_left_stick_x = map(left_stick_x, MIN_LEFT_STICK_X, MAX_LEFT_STICK_X, -MAX_PWM, MAX_PWM);   // Map left stick X-axis value to PWM range [0 - 1020] to [-1023 - 1023]

  Serial.println("Thtottle: " + String(map_throttle) + ", Brake: " + String(map_brake) + ", Left stick X: " + String(map_left_stick_x));
}