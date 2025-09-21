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
// ** NEW ** speed variables for two-stage forward movement
const int stepForwardSpeed = 100; // A faster speed for the initial push to clear a node.
const int followSpeed = 60;       // The normal, slower speed for accurately following the line.
const int turnSpeed = 80;
const int stepForwardDelay = 190;   // Duration of the fast initial push. You may need to tune this.
const int centerOnNodeDelay = 1400; // Small push to center on a node before turning.

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
  // ** MODIFIED **: Give a fast push forward to clear the last node.
  moveForwardAtSpeed(stepForwardSpeed, stepForwardDelay);

  while (true) {
    // A HIGH signal means the sensor is over the black line.
    bool sL_onPath = (digitalRead(SENSOR_SL) == HIGH);
    bool sR_onPath = (digitalRead(SENSOR_SR) == HIGH);
    bool fL_onPath = (digitalRead(SENSOR_FL) == HIGH);
    bool fR_onPath = (digitalRead(SENSOR_FR) == HIGH);

    // PRIORITY 1: Check for a junction (any side path is detected).
    if (sL_onPath || sR_onPath) {
      Serial.println("Potential node... checking for T-Junction.");
      
      // Pause briefly to allow the robot to move forward enough for
      // both sensors to see the T-junction if it exists.
      delay(80);

      // Re-read the side sensors after the pause to get the final state.
      bool final_sL = (digitalRead(SENSOR_SL) == HIGH);
      bool final_sR = (digitalRead(SENSOR_SR) == HIGH);

      // Now, if ANY side sensor is active, we confirm it is a single node.
      if (final_sL || final_sR) {
        if (final_sL && final_sR) {
          Serial.println("Confirmed: Node is a T-Junction.");
        } else {
          Serial.println("Confirmed: Node is a side turn.");
        }
        
        // Stop the motors and break the loop.
        stopMotors();
        break; // Exit the loop and report 'N' to the app.
      }
      // If after the delay no sensors are on the path, it was a false reading.
    }
    
    // PRIORITY 2: Check for the end of the maze (a solid horizontal line).
    if (fL_onPath && fR_onPath) {
      Serial.println("End of maze detected. Stopping.");
      stopMotors();
      break; // Exit the loop and report 'N'
    }

    // PRIORITY 3: If no node is detected, just follow the line at the slower speed.
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
    analogWrite(ENA, followSpeed);
    analogWrite(ENB, 0);
  } else if (pathOnLeft_F && !pathOnRight_F) {
    // Veered RIGHT, left sensor is on the line. Pivot LEFT to correct.
    analogWrite(ENA, 0);
    analogWrite(ENB, followSpeed);
  } else {
    // Go straight.
    analogWrite(ENA, followSpeed);
    analogWrite(ENB, followSpeed);
  }
}

// --- Basic Motor Functions ---

/**
 * @brief Turns left using a more reliable two-stage method.
 */
void turnLeft() {
  // Move forward to the center of the intersection's axis
  moveForward(centerOnNodeDelay);

  // --- STAGE 1: FAST TURN ---
  const int fastTurnDelay = 350;

  analogWrite(ENA, turnSpeed);  
  analogWrite(ENB, turnSpeed);
  digitalWrite(IN1, LOW);  
  digitalWrite(IN2, HIGH);   // Left motor backward
  digitalWrite(IN3, HIGH);  
  digitalWrite(IN4, LOW);     // Right motor forward
  delay(fastTurnDelay);
  
  // --- STAGE 2: SLOW SEARCH ---
  const int slowTurnSpeed = 60;
  analogWrite(ENA, slowTurnSpeed);
  analogWrite(ENB, slowTurnSpeed);

  while (digitalRead(SENSOR_FL) == LOW) {
    // Wait for the sensor to detect the black line
  }

  stopMotors();
}


/**
 * @brief Turns right using a more reliable two-stage method.
 */
void turnRight() {
  // Move forward to the center of the intersection's axis
  moveForward(centerOnNodeDelay);

  // --- STAGE 1: FAST TURN ---
  const int fastTurnDelay = 350;

  analogWrite(ENA, turnSpeed);  
  analogWrite(ENB, turnSpeed);
  digitalWrite(IN1, HIGH);  
  digitalWrite(IN2, LOW);     // Left motor forward
  digitalWrite(IN3, LOW);  
  digitalWrite(IN4, HIGH);   // Right motor backward
  delay(fastTurnDelay);
  
  // --- STAGE 2: SLOW SEARCH ---
  const int slowTurnSpeed = 60;
  analogWrite(ENA, slowTurnSpeed);
  analogWrite(ENB, slowTurnSpeed);

  while (digitalRead(SENSOR_FR) == LOW) {
    // Wait for the sensor to detect the black line
  }

  stopMotors();
}

/**
 * @brief ** NEW ** Moves the robot forward at a specified speed for a set duration.
 * This is the new primary function for timed forward movement.
 */
void moveForwardAtSpeed(int speed, int duration) {
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  analogWrite(ENA, speed); analogWrite(ENB, speed);
  delay(duration);
}

/**
 * @brief ** MODIFIED ** Moves forward at the default 'followSpeed'.
 * This is now used for the slower, more controlled push before turning.
 */
void moveForward(int duration) {
  moveForwardAtSpeed(followSpeed, duration);
}

void stopMotors() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
  analogWrite(ENA, 0); analogWrite(ENB, 0);
}
