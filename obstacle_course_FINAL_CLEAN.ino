/*
 * ============================================================================
 * OBSTACLE COURSE NAVIGATION SYSTEM
 * ============================================================================
 * 
 * PROJECT:     Arduino Robot Obstacle Course Challenge
 * HARDWARE:    Arduino R4 UNO Minima
 * VERSION:     2.0
 * 
 * DESCRIPTION:
 * This program controls a two-wheeled robot through a high-speed obstacle 
 * course. The robot uses IR sensors for line following, a color sensor for
 * start/finish detection, and PID control for smooth, fast navigation.
 * 
 * COMPETITION OBJECTIVE:
 * Complete the winding obstacle course as quickly as possible while 
 * maintaining accurate line following. Faster completion = higher score.
 * 
 * AUTHOR:      Broken Wheels Team 39
 * DATE:        Feb 1 2026
 * ============================================================================
 */


// ============================================================================
// HARDWARE PIN CONFIGURATION
// ============================================================================

// Motor Driver Pin Assignments
// These pins control the left and right DC motors via an H-bridge driver
#define MOTOR_LEFT_PWM    5    // PWM pin for left motor speed control (0-255)
#define MOTOR_LEFT_DIR1   4    // Direction control pin 1 for left motor
#define MOTOR_LEFT_DIR2   3    // Direction control pin 2 for left motor
#define MOTOR_RIGHT_PWM   6    // PWM pin for right motor speed control (0-255)
#define MOTOR_RIGHT_DIR1  7    // Direction control pin 1 for right motor
#define MOTOR_RIGHT_DIR2  8    // Direction control pin 2 for right motor

// IR Sensor Pin Assignments
// These analog sensors detect the black line on a white surface
#define IR_LEFT   A0   // Left IR sensor analog input
#define IR_RIGHT  A1   // Right IR sensor analog input

// Color Sensor Pin Assignments (TCS3200/TCS230)
// This sensor detects colored tape (green=start, blue=finish)
#define COLOR_S0      9    // Frequency scaling control bit 0
#define COLOR_S1     10    // Frequency scaling control bit 1
#define COLOR_S2     11    // Color filter selection bit 0 (Red/Green/Blue)
#define COLOR_S3     12    // Color filter selection bit 1 (Red/Green/Blue)
#define COLOR_OUT    13    // Frequency output from sensor


// ============================================================================
// ROBOT SPEED CONFIGURATION
// ============================================================================
// These values control how fast the robot moves in different situations
// Higher values = faster movement (range: 0-255)

#define SPEED_BASE    180   // Normal cruising speed on straightaways
#define SPEED_FAST    220   // Maximum speed when centered on line
#define SPEED_SLOW    100   // Recovery speed when searching for line
#define SPEED_TURN    140   // Speed during sharp turns


// ============================================================================
// SENSOR CALIBRATION THRESHOLDS
// ============================================================================

// IR Line Sensor Threshold
// This value separates "on line" (dark) from "off line" (light)
// CALIBRATION REQUIRED: Set to midpoint between your white and black readings
#define LINE_THRESHOLD 500  // Values BELOW this = on line, ABOVE = off line

// Color Sensor Thresholds - GREEN TAPE (Start Line)
// Lower frequency = stronger color response
// CALIBRATION REQUIRED: Update based on your green tape readings
#define GREEN_THRESHOLD_R  80   // Red component must be ABOVE this
#define GREEN_THRESHOLD_G  40   // Green component must be BELOW this (strong green)
#define GREEN_THRESHOLD_B  80   // Blue component must be ABOVE this

// Color Sensor Thresholds - BLUE TAPE (Finish Line)
// CALIBRATION REQUIRED: Update based on your blue tape readings
#define BLUE_THRESHOLD_R   80   // Red component must be ABOVE this
#define BLUE_THRESHOLD_G   80   // Green component must be ABOVE this
#define BLUE_THRESHOLD_B   35   // Blue component must be BELOW this (strong blue)

// Finish Line Detection Debouncing
// How many consecutive blue detections required to confirm finish
#define BLUE_CONFIRM_COUNT 10   // Prevents false triggers from reflections


// ============================================================================
// PID CONTROLLER TUNING PARAMETERS
// ============================================================================
// PID (Proportional-Integral-Derivative) control creates smooth steering
// by predicting and correcting line-following errors

// Kp: Proportional Gain - Immediate response to current error
//     Higher = stronger/faster correction, but may cause oscillation
//     Lower = gentler correction, but may be sluggish
float Kp = 1.0;

// Ki: Integral Gain - Corrects accumulated error over time
//     Usually kept at 0 for line following to avoid integral windup
//     Only needed if robot consistently drifts one direction
float Ki = 0.0;

// Kd: Derivative Gain - Predicts future error based on rate of change
//     Higher = more dampening, smoother motion
//     Lower = less dampening, may overshoot
float Kd = 0.5;


// ============================================================================
// GLOBAL STATE VARIABLES
// ============================================================================

// PID Controller State
float lastError = 0.0;      // Previous error value (for derivative calculation)
float integral = 0.0;       // Accumulated error over time (for integral term)

// Color Sensor Reading Storage
int redFrequency = 0;       // Red color component frequency
int greenFrequency = 0;     // Green color component frequency
int blueFrequency = 0;      // Blue color component frequency

// Race Timing
unsigned long raceStartTime = 0;     // Timestamp when race begins (milliseconds)
bool raceComplete = false;           // Flag indicating race has finished

// Start/Finish Detection State
bool waitingForStart = true;         // True until green start tape detected
int blueDetectionCounter = 0;        // Counts consecutive blue detections


// ============================================================================
// ARDUINO SETUP FUNCTION
// ============================================================================
// This function runs ONCE when the Arduino powers on or resets
// It initializes all hardware and prepares the robot for racing

void setup() {
  
  // Initialize Serial Communication for debugging and calibration
  Serial.begin(9600);
  
  // Configure Motor Control Pins as Outputs
  pinMode(MOTOR_LEFT_PWM, OUTPUT);
  pinMode(MOTOR_LEFT_DIR1, OUTPUT);
  pinMode(MOTOR_LEFT_DIR2, OUTPUT);
  pinMode(MOTOR_RIGHT_PWM, OUTPUT);
  pinMode(MOTOR_RIGHT_DIR1, OUTPUT);
  pinMode(MOTOR_RIGHT_DIR2, OUTPUT);
  
  // Configure IR Sensor Pins as Inputs
  pinMode(IR_LEFT, INPUT);
  pinMode(IR_RIGHT, INPUT);
  
  // Configure Color Sensor Pins
  pinMode(COLOR_S0, OUTPUT);
  pinMode(COLOR_S1, OUTPUT);
  pinMode(COLOR_S2, OUTPUT);
  pinMode(COLOR_S3, OUTPUT);
  pinMode(COLOR_OUT, INPUT);
  
  // Set Color Sensor Frequency Scaling to 20%
  // This provides readable frequency values for color detection
  // S0=HIGH, S1=LOW sets output frequency scaling to 20%
  digitalWrite(COLOR_S0, HIGH);
  digitalWrite(COLOR_S1, LOW);
  
  // Ensure motors are stopped at startup
  stopAllMotors();
  
  // Display startup banner
  printStartupBanner();
  
  // Run calibration mode for first 5 seconds
  // This helps users calibrate their color sensor thresholds
  runCalibrationMode();
  
  // Display ready message and wait for green start tape
  Serial.println("");
  Serial.println("======================================");
  Serial.println("  READY TO RACE!");
  Serial.println("======================================");
  Serial.println("Place robot on GREEN starting tape...");
  Serial.println("Race will begin automatically.");
  Serial.println("");
}


// ============================================================================
// ARDUINO MAIN LOOP FUNCTION
// ============================================================================
// This function runs CONTINUOUSLY after setup() completes
// It contains the main robot control logic

void loop() {
  
  // ========== PHASE 1: WAITING FOR START ==========
  // Robot waits motionless until green start tape is detected
  
  if (waitingForStart) {
    
    // Check if green start tape is detected
    if (detectGreenTape()) {
      
      // Green detected - begin the race!
      Serial.println("");
      Serial.println(">>> GREEN START TAPE DETECTED <<<");
      Serial.println(">>> RACE STARTING NOW! <<<");
      Serial.println("");
      
      waitingForStart = false;      // Exit waiting phase
      raceStartTime = millis();     // Record race start timestamp
      delay(500);                   // Brief pause for stability
      
    } else {
      
      // Still waiting - keep motors stopped
      stopAllMotors();
      delay(100);  // Small delay to avoid overwhelming serial output
      return;      // Skip rest of loop until start detected
      
    }
  }
  
  
  // ========== PHASE 2: RACING ==========
  // Main race logic - follow line until finish detected
  
  if (!raceComplete) {
    
    // Execute PID-controlled line following
    followLineWithPID();
    
    // Check for blue finish tape with debouncing
    if (detectBlueTape()) {
      
      // Increment consecutive detection counter
      blueDetectionCounter++;
      
      // Display detection progress (for debugging)
      if (blueDetectionCounter % 5 == 0) {  // Print every 5th detection
        Serial.print("Blue tape detected (");
        Serial.print(blueDetectionCounter);
        Serial.print("/");
        Serial.print(BLUE_CONFIRM_COUNT);
        Serial.println(")");
      }
      
      // Check if we've confirmed finish line
      if (blueDetectionCounter >= BLUE_CONFIRM_COUNT) {
        Serial.println("");
        Serial.println(">>> FINISH LINE CONFIRMED <<<");
        raceComplete = true;
        handleRaceFinish();
      }
      
    } else {
      
      // Blue not detected - reset counter
      // This implements debouncing to prevent false finish triggers
      blueDetectionCounter = 0;
      
    }
    
  } else {
    
    // ========== PHASE 3: RACE COMPLETE ==========
    // Race finished - keep motors stopped
    stopAllMotors();
    
  }
}


// ============================================================================
// LINE FOLLOWING FUNCTIONS
// ============================================================================

/**
 * followLineWithPID()
 * 
 * Main line-following algorithm using PID control for smooth steering.
 * This function:
 *   1. Reads IR sensors to determine robot position relative to line
 *   2. Calculates error (how far off-center the robot is)
 *   3. Uses PID math to compute smooth steering correction
 *   4. Adjusts motor speeds to steer back toward line
 * 
 * PID provides smoother, faster line following than simple on/off control.
 */
void followLineWithPID() {
  
  // Read IR sensor values
  // Lower values = darker surface (on line)
  // Higher values = lighter surface (off line)
  int leftSensorValue = analogRead(IR_LEFT);
  int rightSensorValue = analogRead(IR_RIGHT);
  
  
  // ========== DETERMINE POSITION ERROR ==========
  // Calculate how far robot is from centered on line
  // Error range: -1 (need left turn) to +1 (need right turn)
  
  int positionError = 0;
  
  if (leftSensorValue < LINE_THRESHOLD && rightSensorValue > LINE_THRESHOLD) {
    
    // LEFT sensor on line (dark), RIGHT sensor off line (light)
    // Robot is positioned too far RIGHT of center
    // Need to steer LEFT to recenter
    positionError = -1;
    
  } else if (rightSensorValue < LINE_THRESHOLD && leftSensorValue > LINE_THRESHOLD) {
    
    // RIGHT sensor on line (dark), LEFT sensor off line (light)
    // Robot is positioned too far LEFT of center
    // Need to steer RIGHT to recenter
    positionError = 1;
    
  } else if (leftSensorValue < LINE_THRESHOLD && rightSensorValue < LINE_THRESHOLD) {
    
    // BOTH sensors on line (both dark)
    // Robot is perfectly centered - continue straight!
    positionError = 0;
    
  } else {
    
    // BOTH sensors off line (both light)
    // Line lost! Maintain last known direction to search for it
    positionError = lastError;
    
  }
  
  
  // ========== PID CALCULATION ==========
  // Compute smooth steering correction using PID algorithm
  
  // INTEGRAL: Accumulate error over time
  // Helps correct persistent drift in one direction
  integral += positionError;
  
  // Prevent integral windup (integral term growing too large)
  // Constrain accumulated error to reasonable bounds
  integral = constrain(integral, -50.0, 50.0);
  
  // DERIVATIVE: Calculate rate of change of error
  // Predicts future error to smooth out corrections
  float derivative = positionError - lastError;
  
  // COMBINED PID OUTPUT: Weighted sum of P, I, and D terms
  float steeringCorrection = (Kp * positionError) + 
                            (Ki * integral) + 
                            (Kd * derivative);
  
  
  // ========== CALCULATE MOTOR SPEEDS ==========
  // Apply steering correction to individual motor speeds
  
  int leftMotorSpeed;
  int rightMotorSpeed;
  
  if (abs(positionError) > 0) {
    
    // Robot is turning - use moderate speeds with correction
    // Negative correction = slow down left motor (turn left)
    // Positive correction = slow down right motor (turn right)
    leftMotorSpeed = constrain(SPEED_BASE - (steeringCorrection * 60), 
                               SPEED_SLOW, SPEED_FAST);
    rightMotorSpeed = constrain(SPEED_BASE + (steeringCorrection * 60), 
                                SPEED_SLOW, SPEED_FAST);
    
  } else {
    
    // Robot is centered - maximum speed straight ahead!
    leftMotorSpeed = SPEED_FAST;
    rightMotorSpeed = SPEED_FAST;
    
  }
  
  
  // ========== APPLY MOTOR SPEEDS ==========
  // Send calculated speeds to motors
  setMotorSpeeds(leftMotorSpeed, rightMotorSpeed);
  
  // Save current error for next derivative calculation
  lastError = positionError;
}


// ============================================================================
// MOTOR CONTROL FUNCTIONS
// ============================================================================

/**
 * setMotorSpeeds(leftSpeed, rightSpeed)
 * 
 * Sets individual speed and direction for each motor.
 * Positive speeds = forward, negative speeds = backward.
 * 
 * Parameters:
 *   leftSpeed  - Speed for left motor (-255 to 255)
 *   rightSpeed - Speed for right motor (-255 to 255)
 */
void setMotorSpeeds(int leftSpeed, int rightSpeed) {
  
  // ========== LEFT MOTOR ==========
  if (leftSpeed >= 0) {
    
    // Forward direction
    digitalWrite(MOTOR_LEFT_DIR1, HIGH);
    digitalWrite(MOTOR_LEFT_DIR2, LOW);
    analogWrite(MOTOR_LEFT_PWM, leftSpeed);
    
  } else {
    
    // Backward direction (for negative speeds)
    digitalWrite(MOTOR_LEFT_DIR1, LOW);
    digitalWrite(MOTOR_LEFT_DIR2, HIGH);
    analogWrite(MOTOR_LEFT_PWM, abs(leftSpeed));
    
  }
  
  
  // ========== RIGHT MOTOR ==========
  if (rightSpeed >= 0) {
    
    // Forward direction
    digitalWrite(MOTOR_RIGHT_DIR1, HIGH);
    digitalWrite(MOTOR_RIGHT_DIR2, LOW);
    analogWrite(MOTOR_RIGHT_PWM, rightSpeed);
    
  } else {
    
    // Backward direction (for negative speeds)
    digitalWrite(MOTOR_RIGHT_DIR1, LOW);
    digitalWrite(MOTOR_RIGHT_DIR2, HIGH);
    analogWrite(MOTOR_RIGHT_PWM, abs(rightSpeed));
    
  }
}


/**
 * moveForward(speed)
 * 
 * Drives both motors forward at specified speed.
 * 
 * Parameters:
 *   speed - Motor speed (0-255)
 */
void moveForward(int speed) {
  digitalWrite(MOTOR_LEFT_DIR1, HIGH);
  digitalWrite(MOTOR_LEFT_DIR2, LOW);
  analogWrite(MOTOR_LEFT_PWM, speed);
  
  digitalWrite(MOTOR_RIGHT_DIR1, HIGH);
  digitalWrite(MOTOR_RIGHT_DIR2, LOW);
  analogWrite(MOTOR_RIGHT_PWM, speed);
}


/**
 * moveBackward(speed)
 * 
 * Drives both motors backward at specified speed.
 * 
 * Parameters:
 *   speed - Motor speed (0-255)
 */
void moveBackward(int speed) {
  digitalWrite(MOTOR_LEFT_DIR1, LOW);
  digitalWrite(MOTOR_LEFT_DIR2, HIGH);
  analogWrite(MOTOR_LEFT_PWM, speed);
  
  digitalWrite(MOTOR_RIGHT_DIR1, LOW);
  digitalWrite(MOTOR_RIGHT_DIR2, HIGH);
  analogWrite(MOTOR_RIGHT_PWM, speed);
}


/**
 * turnLeft(speed)
 * 
 * Spins robot counter-clockwise by running motors in opposite directions.
 * 
 * Parameters:
 *   speed - Turn speed (0-255)
 */
void turnLeft(int speed) {
  digitalWrite(MOTOR_LEFT_DIR1, LOW);
  digitalWrite(MOTOR_LEFT_DIR2, HIGH);
  analogWrite(MOTOR_LEFT_PWM, speed);
  
  digitalWrite(MOTOR_RIGHT_DIR1, HIGH);
  digitalWrite(MOTOR_RIGHT_DIR2, LOW);
  analogWrite(MOTOR_RIGHT_PWM, speed);
}


/**
 * turnRight(speed)
 * 
 * Spins robot clockwise by running motors in opposite directions.
 * 
 * Parameters:
 *   speed - Turn speed (0-255)
 */
void turnRight(int speed) {
  digitalWrite(MOTOR_LEFT_DIR1, HIGH);
  digitalWrite(MOTOR_LEFT_DIR2, LOW);
  analogWrite(MOTOR_LEFT_PWM, speed);
  
  digitalWrite(MOTOR_RIGHT_DIR1, LOW);
  digitalWrite(MOTOR_RIGHT_DIR2, HIGH);
  analogWrite(MOTOR_RIGHT_PWM, speed);
}


/**
 * stopAllMotors()
 * 
 * Immediately stops both motors by setting all control pins LOW
 * and PWM speeds to zero.
 */
void stopAllMotors() {
  digitalWrite(MOTOR_LEFT_DIR1, LOW);
  digitalWrite(MOTOR_LEFT_DIR2, LOW);
  digitalWrite(MOTOR_RIGHT_DIR1, LOW);
  digitalWrite(MOTOR_RIGHT_DIR2, LOW);
  analogWrite(MOTOR_LEFT_PWM, 0);
  analogWrite(MOTOR_RIGHT_PWM, 0);
}


// ============================================================================
// COLOR SENSOR FUNCTIONS
// ============================================================================

/**
 * readColorSensor()
 * 
 * Reads RGB color values from TCS3200/TCS230 color sensor.
 * The sensor outputs frequency - lower frequency = stronger color response.
 * 
 * This function updates global variables:
 *   redFrequency, greenFrequency, blueFrequency
 * 
 * Color filters are selected using S2 and S3 pins:
 *   S2=LOW,  S3=LOW  -> Red filter
 *   S2=HIGH, S3=HIGH -> Green filter
 *   S2=LOW,  S3=HIGH -> Blue filter
 */
void readColorSensor() {
  
  // ========== READ RED COMPONENT ==========
  digitalWrite(COLOR_S2, LOW);
  digitalWrite(COLOR_S3, LOW);
  
  // Measure frequency output (lower = more red detected)
  // Timeout of 50ms prevents hanging if sensor fails
  redFrequency = pulseIn(COLOR_OUT, LOW, 50000);
  
  // If timeout occurs (returns 0), set to max value
  if (redFrequency == 0) {
    redFrequency = 255;
  }
  
  delayMicroseconds(100);  // Small delay for sensor stability
  
  
  // ========== READ GREEN COMPONENT ==========
  digitalWrite(COLOR_S2, HIGH);
  digitalWrite(COLOR_S3, HIGH);
  
  greenFrequency = pulseIn(COLOR_OUT, LOW, 50000);
  if (greenFrequency == 0) {
    greenFrequency = 255;
  }
  
  delayMicroseconds(100);
  
  
  // ========== READ BLUE COMPONENT ==========
  digitalWrite(COLOR_S2, LOW);
  digitalWrite(COLOR_S3, HIGH);
  
  blueFrequency = pulseIn(COLOR_OUT, LOW, 50000);
  if (blueFrequency == 0) {
    blueFrequency = 255;
  }
}


/**
 * detectGreenTape()
 * 
 * Detects if color sensor is viewing green starting tape.
 * 
 * Detection logic:
 *   - Green frequency must be LOWEST (strongest green response)
 *   - Green frequency must be below GREEN_THRESHOLD_G
 *   - Red and Blue frequencies must be HIGHER (weaker response)
 *   - Red frequency must be above GREEN_THRESHOLD_R
 *   - Blue frequency must be above GREEN_THRESHOLD_B
 * 
 * Returns:
 *   true if green tape detected, false otherwise
 */
bool detectGreenTape() {
  
  // Read current color values
  readColorSensor();
  
  // Check if green is the dominant color (lowest frequency)
  bool greenIsDominant = (greenFrequency < redFrequency) && 
                        (greenFrequency < blueFrequency) && 
                        (greenFrequency < GREEN_THRESHOLD_G);
  
  // Verify that red and blue are weak (not interfering)
  bool otherColorsWeak = (redFrequency > GREEN_THRESHOLD_R) && 
                        (blueFrequency > GREEN_THRESHOLD_B);
  
  // Both conditions must be true for positive detection
  return greenIsDominant && otherColorsWeak;
}


/**
 * detectBlueTape()
 * 
 * Detects if color sensor is viewing blue finish tape.
 * 
 * Detection logic:
 *   - Blue frequency must be LOWEST (strongest blue response)
 *   - Blue frequency must be below BLUE_THRESHOLD_B
 *   - Red and Green frequencies must be HIGHER (weaker response)
 *   - Red frequency must be above BLUE_THRESHOLD_R
 *   - Green frequency must be above BLUE_THRESHOLD_G
 * 
 * Returns:
 *   true if blue tape detected, false otherwise
 */
bool detectBlueTape() {
  
  // Read current color values
  readColorSensor();
  
  // Check if blue is the dominant color (lowest frequency)
  bool blueIsDominant = (blueFrequency < redFrequency) && 
                       (blueFrequency < greenFrequency) && 
                       (blueFrequency < BLUE_THRESHOLD_B);
  
  // Verify that red and green are weak (not interfering)
  bool otherColorsWeak = (redFrequency > BLUE_THRESHOLD_R) && 
                        (greenFrequency > BLUE_THRESHOLD_G);
  
  // Both conditions must be true for positive detection
  return blueIsDominant && otherColorsWeak;
}


// ============================================================================
// RACE MANAGEMENT FUNCTIONS
// ============================================================================

/**
 * handleRaceFinish()
 * 
 * Called when finish line is confirmed.
 * Stops robot, calculates race time, and displays results.
 */
void handleRaceFinish() {
  
  // Stop all motors immediately
  stopAllMotors();
  
  // Calculate race duration
  unsigned long raceEndTime = millis();
  unsigned long raceDuration = (raceEndTime - raceStartTime) / 1000;  // Convert to seconds
  
  // Display race completion banner
  Serial.println("");
  Serial.println("================================================");
  Serial.println("           RACE COMPLETE!");
  Serial.println("================================================");
  Serial.print  ("           Finish Time: ");
  Serial.print(raceDuration);
  Serial.println(" seconds");
  Serial.println("================================================");
  Serial.println("");
  
  // Victory celebration - pulse motors briefly
  celebrateVictory();
}


/**
 * celebrateVictory()
 * 
 * Fun motor animation to celebrate race completion.
 * Pulses motors on and off several times.
 */
void celebrateVictory() {
  
  for (int i = 0; i < 5; i++) {
    moveForward(150);
    delay(100);
    stopAllMotors();
    delay(100);
  }
}


// ============================================================================
// CALIBRATION AND DIAGNOSTIC FUNCTIONS
// ============================================================================

/**
 * printStartupBanner()
 * 
 * Displays welcome message and system information at startup.
 */
void printStartupBanner() {
  
  Serial.println("");
  Serial.println("================================================");
  Serial.println("   OBSTACLE COURSE NAVIGATION SYSTEM v2.0");
  Serial.println("================================================");
  Serial.println("   Hardware: Arduino R4 UNO Minima");
  Serial.println("   Algorithm: PID Line Following");
  Serial.println("   Sensors: IR + RGB Color Detection");
  Serial.println("================================================");
  Serial.println("");
}


/**
 * runCalibrationMode()
 * 
 * Runs for 5 seconds at startup to help users calibrate color thresholds.
 * Continuously reads and displays RGB values from color sensor.
 * Users should point sensor at different colored tapes and note the values.
 */
void runCalibrationMode() {
  
  Serial.println("========== CALIBRATION MODE ==========");
  Serial.println("Running for 5 seconds...");
  Serial.println("Point sensor at different colors:");
  Serial.println("  - Black line");
  Serial.println("  - Green start tape");
  Serial.println("  - Blue finish tape");
  Serial.println("");
  Serial.println("Format: R:xxx G:xxx B:xxx -> Detected Color");
  Serial.println("--------------------------------------");
  
  unsigned long calibrationStart = millis();
  
  // Run calibration for 5 seconds
  while (millis() - calibrationStart < 5000) {
    
    // Read color sensor
    readColorSensor();
    
    // Display RGB values
    Serial.print("R:");
    Serial.print(redFrequency);
    Serial.print("  G:");
    Serial.print(greenFrequency);
    Serial.print("  B:");
    Serial.print(blueFrequency);
    Serial.print("  ->  ");
    
    // Show which color is detected
    if (detectGreenTape()) {
      Serial.println("GREEN");
    } else if (detectBlueTape()) {
      Serial.println("BLUE");
    } else {
      Serial.println("Black/Other");
    }
    
    delay(300);  // Update every 300ms for readability
  }
  
  Serial.println("--------------------------------------");
  Serial.println("Calibration complete!");
  Serial.println("");
}


/**
 * printColorValues()
 * 
 * Utility function to display current RGB sensor readings.
 * Useful for debugging color detection issues.
 */
void printColorValues() {
  
  readColorSensor();
  
  Serial.print("RGB Values - R:");
  Serial.print(redFrequency);
  Serial.print(" G:");
  Serial.print(greenFrequency);
  Serial.print(" B:");
  Serial.print(blueFrequency);
  Serial.print(" | Detected: ");
  
  if (detectGreenTape()) {
    Serial.println("GREEN");
  } else if (detectBlueTape()) {
    Serial.println("BLUE");
  } else {
    Serial.println("Black/Other");
  }
}


// ============================================================================
// END OF PROGRAM
// ============================================================================
/*
 * CALIBRATION INSTRUCTIONS:
 * -------------------------
 * 1. Upload this code to your Arduino
 * 2. Open Serial Monitor (9600 baud)
 * 3. During the 5-second calibration mode:
 *    - Point sensor at GREEN tape and note RGB values
 *    - Point sensor at BLUE tape and note RGB values
 * 4. Update threshold constants at top of code:
 *    - For GREEN: Set GREEN_THRESHOLD_G to your green reading + 5-10
 *    - For BLUE: Set BLUE_THRESHOLD_B to your blue reading + 5-10
 * 5. Re-upload code with your calibrated values
 * 6. Place robot on green starting tape
 * 7. Race begins automatically!
 * 
 * SPEED TUNING:
 * -------------
 * If robot is too fast and loses the line:
 *   - Decrease SPEED_BASE and SPEED_FAST
 * If robot is too slow:
 *   - Increase SPEED_BASE and SPEED_FAST
 * 
 * PID TUNING:
 * -----------
 * If robot wobbles/oscillates:
 *   - Decrease Kp (try 0.7-0.8)
 *   - Increase Kd (try 0.6-0.8)
 * If robot responds too slowly:
 *   - Increase Kp (try 1.2-1.5)
 * 
 * Good luck with your race!
 */
