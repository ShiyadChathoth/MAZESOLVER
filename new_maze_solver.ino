#include <SoftwareSerial.h>

// --- Pin Definitions ---
SoftwareSerial bluetooth(2, 3); // RX, TX
const int ENA = 9, ENB = 10;
const int IN1 = 4, IN2 = 5, IN3 = 6, IN4 = 7;
const int SENSOR_SL = A0; // Side-Left
const int SENSOR_FL = A1; // Front-Left
const int SENSOR_FR = A2; // Front-Right
const int SENSOR_SR = A3; // Side-Right

// --- Tunable Parameters ---
const int moveSpeed = 60;
const int turnSpeed = 120;
const int turnDelay90 = 430;      // Adjust for accurate 90-degree turns
const int stepForwardDelay = 200; // Push to clear the previous node
const int centerOnNodeDelay = 1400; // << NEW: Small push to center on a node before turning

void setup() {
  Serial.begin(9600);
  bluetooth.begin(9600);
  pinMode(ENA, OUTPUT); pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  pinMode(SENSOR_SL, INPUT); pinMode(SENSOR_FL, INPUT);
  pinMode(SENSOR_FR, INPUT); pinMode(SENSOR_SR, INPUT);
  Serial.println("Maze Solving Robot Ready.");
  stopMotors();
}

void loop() {
  if (bluetooth.available()) {
    char command = bluetooth.read();
    executeCommand(command);
  }
}

/**
 * @brief Executes a command received via Bluetooth.
 * This is the main control function.
 */
void executeCommand(char command) {
  switch (command) {
    case 'F':
      Serial.println("Command: Move to Next Node");
      moveForwardUntilNode();
      bluetooth.print('N'); // Report: Node reached
      break;
    case 'L':
      Serial.println("Command: Turn Left");
      turnLeft(); // Turn immediately
      bluetooth.print('C'); // Report: Command complete
      break;
    case 'R':
      Serial.println("Command: Turn Right");
      turnRight(); // Turn immediately
      bluetooth.print('C'); // Report: Command complete
      break;
  }
}

// --- Core Movement Logic ---

/**
 * @brief Moves the robot forward, following the line, until a node is detected.
 */
void moveForwardUntilNode() {
  // Give a small push forward to ensure we are clear of the last node
  moveForward(stepForwardDelay);

  while (true) {
    // A LOW signal means the sensor is over the black line.
    bool sL_onPath = (digitalRead(SENSOR_SL) == HIGH);
    bool sR_onPath = (digitalRead(SENSOR_SR) == HIGH);
    bool fL_onPath = (digitalRead(SENSOR_FL) == HIGH);
    bool fR_onPath = (digitalRead(SENSOR_FR) == HIGH);

    // PRIORITY 1: Check for a junction (any side path is detected).
    if (sL_onPath && sR_onPath) {
      Serial.println("Node detected (T juction).");
      // MODIFIED: Just stop. The centering push is now in the turn functions.
      stopMotors();
      break; // Exit the loop and report 'N'
    }
    else if (sL_onPath || sR_onPath) {
      Serial.println("Node detected (side path).");
      // MODIFIED: Just stop. The centering push is now in the turn functions.
      stopMotors();
      break; // Exit the loop and report 'N'
    }

    // PRIORITY 2: Check for the end of the maze (a solid horizontal line).
    if (fL_onPath && fR_onPath) {
      Serial.println("End of maze detected. Stopping.");
      stopMotors();
      break; // Exit the loop and report 'N'
    }

    // PRIORITY 3: If no node is detected, just follow the line.
    followCorridor();
  }
}

/**
 * @brief Corrects the robot's path to keep the front two sensors on the line.
 */
void followCorridor() {
  bool pathOnLeft_F = (digitalRead(SENSOR_FL) == HIGH);
  bool pathOnRight_F = (digitalRead(SENSOR_FR) == HIGH);

  // Set motor direction to forward for all correction cases
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);

  if (!pathOnLeft_F && pathOnRight_F) {
    // Veered LEFT, right sensor is on the line. Pivot RIGHT to correct.
    analogWrite(ENA, moveSpeed);
    analogWrite(ENB, 0);
  } else if (pathOnLeft_F && !pathOnRight_F) {
    // Veered RIGHT, left sensor is on the line. Pivot LEFT to correct.
    analogWrite(ENA, 0);
    analogWrite(ENB, moveSpeed);
  } else {
    // Go straight.
    analogWrite(ENA, moveSpeed);
    analogWrite(ENB, moveSpeed);
  }
}

// --- Basic Motor Functions ---

void turnLeft() {
  // << NEW: Move forward slightly to center on the node's axis
  moveForward(centerOnNodeDelay);
  
  analogWrite(ENA, turnSpeed); analogWrite(ENB, turnSpeed);
  digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);  // Left motor backward
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW); // Right motor forward
  delay(turnDelay90);
  stopMotors();
}

void turnRight() {
  // << NEW: Move forward slightly to center on the node's axis
  moveForward(centerOnNodeDelay);
  
  analogWrite(ENA, turnSpeed); analogWrite(ENB, turnSpeed);
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);   // Left motor forward
  digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);  // Right motor backward
  delay(turnDelay90);
  stopMotors();
}

void moveForward(int duration) {
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  analogWrite(ENA, moveSpeed); analogWrite(ENB, moveSpeed);
  delay(duration);
}

void stopMotors() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
  analogWrite(ENA, 0); analogWrite(ENB, 0);
}
