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

const uint8_t FORGET_GAMEPAD_PIN = 13; // Forget gamepad
const uint16_t FREQUENCY = 10000;      // PWM frequency 10kHz
const uint8_t RESOLUTION = 10;         // PWM resolution 10 bits (0-1023)
const uint8_t CHANNEL = 0;             // PWM channel number (0-15)
const uint16_t MAX_PWM = 1023;         // Maximum speed (0-1023)
const uint8_t MIN_PWM = 150;           // Minimum speed (0-1023)
const uint8_t MOTORS_OFFSET = 150;     // Motor driver offset 170 start

// Create motor controller objects
MotorController r_motor(R_MOTOR_IN1, R_MOTOR_IN2, R_MOTOR_PWM, CHANNEL);
MotorController l_motor(L_MOTOR_IN1, L_MOTOR_IN2, L_MOTOR_PWM, CHANNEL);

ControllerPtr ps4[BP32_MAX_GAMEPADS];

enum CONTROL_TYPE {
  RC = 0,
  APP
};

void setup() {
  Serial.begin(115200);                      // Initialize serial communication
  pinMode(FORGET_GAMEPAD_PIN, INPUT_PULLUP); // Set pin for forgetting gamepad

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
  for (int speed = 0; speed <= 1023; speed += 10) {
    r_motor.motorGo(RM_OFFSET + speed); // Set right motor speed
    l_motor.motorGo(LM_OFFSET + speed); // Set left motor speed
    delay(100);                         // Wait for 100ms
  }
}

void onConnectedController(ControllerPtr ctl) {
  bool foundEmptySlot = false;
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (ps4[i] == nullptr) {
      Serial.printf("CALLBACK: Controller is connected, index=%d\n", i);
      // Additionally, you can get certain gamepad properties like:
      // Model, VID, PID, BTAddr, flags, etc.
      ControllerProperties properties = ctl->getProperties();
      Serial.printf("Controller model: %s, VID=0x%04x, PID=0x%04x\n", ctl->getModelName().c_str(), properties.vendor_id,
                    properties.product_id);
      ps4[i] = ctl;
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

  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (ps4[i] == ctl) {
      Serial.printf("CALLBACK: Controller disconnected from index=%d\n", i);
      ps4[i] = nullptr;
      foundController = true;
      break;
    }
  }

  if (!foundController) {
    Serial.println("CALLBACK: Controller disconnected, but not found in myControllers");
  }
}
