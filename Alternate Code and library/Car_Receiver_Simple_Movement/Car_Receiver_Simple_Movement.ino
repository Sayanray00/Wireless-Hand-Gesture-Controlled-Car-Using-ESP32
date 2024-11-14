#include <esp_now.h>
#include <WiFi.h>

// Movement commands
#define FORWARD 1
#define BACKWARD 2
#define LEFT 3
#define RIGHT 4
#define FORWARD_LEFT 5
#define FORWARD_RIGHT 6
#define BACKWARD_LEFT 7
#define BACKWARD_RIGHT 8
#define TURN_LEFT 9
#define TURN_RIGHT 10
#define STOP 0

// Motor identifiers
#define BACK_RIGHT_MOTOR 0
#define BACK_LEFT_MOTOR 1
#define FRONT_RIGHT_MOTOR 2
#define FRONT_LEFT_MOTOR 3

// Motor pins structure
struct MOTOR_PINS {
  int pinIN1;
  int pinIN2;
  int pinEn; 
  int pwmSpeedChannel;
};

// Motor pins initialization
std::vector<MOTOR_PINS> motorPins = {
  {16, 17, 22, 4},  // BACK_RIGHT_MOTOR
  {18, 19, 23, 5},  // BACK_LEFT_MOTOR
  {26, 27, 14, 6},  // FRONT_RIGHT_MOTOR
  {33, 25, 32, 7}   // FRONT_LEFT_MOTOR   
};

#define MAX_MOTOR_SPEED 200

const int PWMFreq = 1000; // 1 KHz PWM frequency
const int PWMResolution = 8; // PWM resolution

#define SIGNAL_TIMEOUT 1000 // Signal timeout in milliseconds
unsigned long lastRecvTime = 0; // Last received time

// Data packet structure
struct PacketData {
  byte xAxisValue;
  byte yAxisValue;
};
PacketData receiverData;

// Callback function for received data
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  if (len == 0) {
    return;
  }
  memcpy(&receiverData, incomingData, sizeof(receiverData));
  String inputData;
  inputData = inputData + "values " + receiverData.xAxisValue + "  " + receiverData.yAxisValue;
  Serial.println(inputData);

  // Determine movement based on received data
  if (receiverData.xAxisValue > 175) {
    processCarMovement(TURN_RIGHT);
  } else if (receiverData.xAxisValue < 75) {
    processCarMovement(TURN_LEFT);
  } else if (receiverData.yAxisValue < 75) {
    processCarMovement(FORWARD);
  } else if (receiverData.yAxisValue > 175) {
    processCarMovement(BACKWARD);
  } else {
    processCarMovement(STOP);
  }

  lastRecvTime = millis(); // Update last received time
}

// Process car movement
void processCarMovement(int inputValue) {
  switch (inputValue) {
    case FORWARD:
      rotateMotor(FRONT_RIGHT_MOTOR, MAX_MOTOR_SPEED);
      rotateMotor(BACK_RIGHT_MOTOR, MAX_MOTOR_SPEED);
      rotateMotor(FRONT_LEFT_MOTOR, MAX_MOTOR_SPEED);
      rotateMotor(BACK_LEFT_MOTOR, MAX_MOTOR_SPEED);
      break;

    case BACKWARD:
      rotateMotor(FRONT_RIGHT_MOTOR, -MAX_MOTOR_SPEED);
      rotateMotor(BACK_RIGHT_MOTOR, -MAX_MOTOR_SPEED);
      rotateMotor(FRONT_LEFT_MOTOR, -MAX_MOTOR_SPEED);
      rotateMotor(BACK_LEFT_MOTOR, -MAX_MOTOR_SPEED);
      break;

    case TURN_LEFT:
      rotateMotor(FRONT_RIGHT_MOTOR, MAX_MOTOR_SPEED);
      rotateMotor(BACK_RIGHT_MOTOR, MAX_MOTOR_SPEED);
      rotateMotor(FRONT_LEFT_MOTOR, -MAX_MOTOR_SPEED);
      rotateMotor(BACK_LEFT_MOTOR, -MAX_MOTOR_SPEED);
      break;

    case TURN_RIGHT:
      rotateMotor(FRONT_RIGHT_MOTOR, -MAX_MOTOR_SPEED);
      rotateMotor(BACK_RIGHT_MOTOR, -MAX_MOTOR_SPEED);
      rotateMotor(FRONT_LEFT_MOTOR, MAX_MOTOR_SPEED);
      rotateMotor(BACK_LEFT_MOTOR, MAX_MOTOR_SPEED);
      break;

    case STOP:
      rotateMotor(FRONT_RIGHT_MOTOR, STOP);
      rotateMotor(BACK_RIGHT_MOTOR, STOP);
      rotateMotor(FRONT_LEFT_MOTOR, STOP);
      rotateMotor(BACK_LEFT_MOTOR, STOP);
      break;

    default:
      rotateMotor(FRONT_RIGHT_MOTOR, STOP);
      rotateMotor(BACK_RIGHT_MOTOR, STOP);
      rotateMotor(FRONT_LEFT_MOTOR, STOP);
      rotateMotor(BACK_LEFT_MOTOR, STOP);
      break;
  }
}

// Rotate motor based on speed
void rotateMotor(int motorNumber, int motorSpeed) {
  if (motorSpeed < 0) {
    digitalWrite(motorPins[motorNumber].pinIN1, LOW);
    digitalWrite(motorPins[motorNumber].pinIN2, HIGH);
  } else if (motorSpeed > 0) {
    digitalWrite(motorPins[motorNumber].pinIN1, HIGH);
    digitalWrite(motorPins[motorNumber].pinIN2, LOW);
  } else {
    digitalWrite(motorPins[motorNumber].pinIN1, LOW);
    digitalWrite(motorPins[motorNumber].pinIN2, LOW);
  }

  ledcWrite(motorPins[motorNumber].pwmSpeedChannel, abs(motorSpeed));
}

// Set up pin modes and PWM channels
void setUpPinModes() {
  for (int i = 0; i < motorPins.size(); i++) {
    pinMode(motorPins[i].pinIN1, OUTPUT);
    pinMode(motorPins[i].pinIN2, OUTPUT);
    ledcSetup(motorPins[i].pwmSpeedChannel, PWMFreq, PWMResolution);
    ledcAttachPin(motorPins[i].pinEn, motorPins[i].pwmSpeedChannel);
    rotateMotor(i, STOP);
  }
}

// Setup function
void setup() {
  setUpPinModes();

  Serial.begin(115200);
  WiFi.mode(WIFI_STA);

  // Initialize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  } else {
    Serial.println("Success: Initialized ESP-NOW");
  }

  // Register data receive callback
  esp_now_register_recv_cb(OnDataRecv);
}

// Main loop
void loop() {
  // Check for signal timeout
  unsigned long now = millis();
  if (now - lastRecvTime > SIGNAL_TIMEOUT) {
    processCarMovement(STOP);
  }
}