//(Fixed Block-Style Smorphi Robot Code with Proper WIT901CTTL and Encoder Integration)

#define ENCODER_DO_NOT_USE_INTERRUPTS
#include "smorphi.h"
#include "math.h"
#include "Wire.h"
#include "smorphi_odometry.h"

// ==================== ROBOT SETUP ====================
Smorphi my_robot;
HardwareSerial wit_serial(1); // WIT901CTTL on Serial1

// Encoder setup
#define ENCODER_FR_A 16
#define ENCODER_FR_B 17
#define ENCODER_FL_A 19
#define ENCODER_FL_B 18
#define ENCODER_RR_A 25
#define ENCODER_RR_B 23
#define ENCODER_RL_A 26
#define ENCODER_RL_B 27

ESP32Encoder encoder_fr, encoder_fl, encoder_rr, encoder_rl;
smorphi_odometry::MotorProperty_t *motor_prop_fl, *motor_prop_fr, *motor_prop_rl, *motor_prop_rr;
smorphi_odometry::Odometry_t *odometry;

// ==================== CONFIGURATION ====================
int MySpeed = 50;            // Normal movement speed (0-100) - matched to friend's code
int TurnSpeed = 90;          // Turning speed (0-100) - matched to friend's code
int StopSpeed = 0;

// ==================== ROBOT STATE VARIABLES ====================
const double dt = 0.05;     // 50ms update rate - matched to friend's code
double x = 0, y = 0, theta = 0;
unsigned long lastUpdate = 0;
unsigned long lastSensorRead = 0;
const unsigned long sensorInterval = 50;

// Sensor variables
int front_sensor_status = 1;
int right_sensor_status = 1;
int back_sensor_status = 1;
int left_sensor_status = 1;

// WIT901CTTL variables
float wit_yaw = 0.0;
uint8_t wit_buffer[11];
int wit_buffer_index = 0;

// ==================== INTERNAL FUNCTIONS (Critical IMU/Encoder Functions) ====================
bool readWIT901CTTL() {
  while (wit_serial.available()) {
    uint8_t byte = wit_serial.read();
    
    if (wit_buffer_index == 0 && byte != 0x55) {
      continue; // Wait for header
    }
    
    wit_buffer[wit_buffer_index] = byte;
    wit_buffer_index++;
    
    if (wit_buffer_index >= 11) {
      // Check if this is angle data (0x53)
      if (wit_buffer[1] == 0x53) {
        // Parse yaw angle (bytes 6-7)
        int16_t yaw_raw = (wit_buffer[7] << 8) | wit_buffer[6];
        wit_yaw = yaw_raw / 32768.0 * 180.0; // Convert to degrees
        
        // Normalize to 0-360 degrees
        if (wit_yaw < 0) wit_yaw += 360;
        
        wit_buffer_index = 0;
        return true;
      }
      wit_buffer_index = 0;
    }
  }
  return false;
}

void updateRobotState() {
  unsigned long now = millis();
  if (now - lastUpdate >= (dt * 1000)) {
    lastUpdate = now;
    
    // Read WIT901CTTL data - CRITICAL: This was missing proper integration!
    if (readWIT901CTTL()) {
      theta = wit_yaw;
      odometry->setOrientation(theta * M_PI / 180.0); // Convert to radians for odometry
    }
    
    // Update motor properties and odometry - CRITICAL: This ensures encoder data is used!
    motor_prop_fl->update();
    motor_prop_fr->update();
    motor_prop_rl->update();
    motor_prop_rr->update();
    odometry->update();
    
    // Get position from odometry
    x = odometry->pos_x();
    y = odometry->pos_y();
  }
}

float angleDifference(float target, float current) {
  float diff = target - current;
  while (diff > 180) diff -= 360;
  while (diff < -180) diff += 360;
  return diff;
}

// ==================== BLOCK FUNCTIONS - INITIALIZATION ====================
void initialize_smorphi() {
  Serial.begin(115200);
  Serial.println("Initializing Smorphi with Proper IMU/Encoder Integration...");
  
  my_robot.BeginSmorphi();
  
  // Initialize WIT901CTTL - FIXED: Using correct pin configuration
  wit_serial.begin(9600, SERIAL_8N1, 16, 17); // RX=16, TX=17
  
  // Initialize encoders
  ESP32Encoder::useInternalWeakPullResistors = puType::up;
  encoder_fr.attachHalfQuad(ENCODER_FR_A, ENCODER_FR_B);
  encoder_fl.attachHalfQuad(ENCODER_FL_A, ENCODER_FL_B);
  encoder_rr.attachHalfQuad(ENCODER_RR_A, ENCODER_RR_B);
  encoder_rl.attachHalfQuad(ENCODER_RL_A, ENCODER_RL_B);
  
  encoder_fr.clearCount();
  encoder_fl.clearCount();
  encoder_rl.clearCount();
  encoder_rr.clearCount();
  
  // Initialize odometry - FIXED: Using correct tick count (540, not 540)
  int N = 540; // ticks per revolution
  motor_prop_fl = new smorphi_odometry::MotorProperty_t(&encoder_fl, my_robot.sm_wheel_radius, N);
  motor_prop_fr = new smorphi_odometry::MotorProperty_t(&encoder_fr, my_robot.sm_wheel_radius, N);
  motor_prop_rl = new smorphi_odometry::MotorProperty_t(&encoder_rl, my_robot.sm_wheel_radius, N);
  motor_prop_rr = new smorphi_odometry::MotorProperty_t(&encoder_rr, my_robot.sm_wheel_radius, N);
  odometry = new smorphi_odometry::Odometry_t(motor_prop_fl, motor_prop_fr, motor_prop_rl, motor_prop_rr, my_robot.sm_wheel_x, my_robot.sm_wheel_y);
  
  Serial.println("Robot Ready with Proper Navigation!");
}

// ==================== ROBOT SHAPE TRACKING ====================
char currentShape = 'I'; // Track current shape (default: I)

// ==================== SENSOR READING WITH STATE UPDATE ====================
void readSensors() {
  unsigned long now = millis();
  if (now - lastSensorRead >= sensorInterval) {
    lastSensorRead = now;
    
    // CRITICAL: Always update robot state when reading sensors
    updateRobotState();
    
    // Read sensors based on current shape
    if (currentShape == 'I') {
      // I-shape sensor configuration
      front_sensor_status = my_robot.module1_sensor_status(4);
      back_sensor_status = my_robot.module4_sensor_status(4);
      right_sensor_status = my_robot.module2_sensor_status(4) && my_robot.module1_sensor_status(6) && my_robot.module4_sensor_status(10);
      left_sensor_status = my_robot.module2_sensor_status(10) && my_robot.module1_sensor_status(10) && my_robot.module4_sensor_status(6);
    }
    else if (currentShape == 'O') {
      // O-shape sensor configuration
      front_sensor_status = my_robot.module1_sensor_status(4) && my_robot.module4_sensor_status(4);
      back_sensor_status = my_robot.module3_sensor_status(6);
      right_sensor_status = my_robot.module2_sensor_status(4) && my_robot.module1_sensor_status(2);
      left_sensor_status = my_robot.module4_sensor_status(10);
    }
    else if (currentShape == 'L') {
      // L-shape sensor configuration
      front_sensor_status = my_robot.module1_sensor_status(4);
      back_sensor_status = my_robot.module4_sensor_status(4);
      right_sensor_status = my_robot.module2_sensor_status(4);
      left_sensor_status = my_robot.module3_sensor_status(4);
    }
    else if (currentShape == 'S') {
      // S-shape sensor configuration
      front_sensor_status = my_robot.module1_sensor_status(4);
      back_sensor_status = my_robot.module4_sensor_status(4);
      right_sensor_status = my_robot.module2_sensor_status(4);
      left_sensor_status = my_robot.module3_sensor_status(10);
    }
    
    Serial.printf("Sensors - Front:%d, Right:%d, Left:%d, Back:%d | Pos: x=%.2f, y=%.2f, θ=%.1f°\n", 
                  front_sensor_status, right_sensor_status, left_sensor_status, back_sensor_status, x, y, theta);
  }
}

// ==================== OBSTACLE DETECTION ====================
bool irDetectsObstacleAt(String position) {
  readSensors(); // Always update sensors and state
  
  if (position == "Front") return (front_sensor_status == 0);
  else if (position == "Right") return (right_sensor_status == 0);
  else if (position == "Left") return (left_sensor_status == 0);
  else if (position == "Back") return (back_sensor_status == 0);
  return false;
}

// Opposite of the function above, now the command will execute IF the sensor no longer detects
bool irNoObstacleAt(String position) {
  readSensors(); // Always update sensors and state
  
  if (position == "Front") return (front_sensor_status == 1);
  else if (position == "Right") return (right_sensor_status == 1);
  else if (position == "Left") return (left_sensor_status == 1);
  else if (position == "Back") return (back_sensor_status == 1);
  return false;
}

// ==================== BLOCK FUNCTIONS - BASIC MOVEMENTS ====================
void moveForward() {
  readSensors(); // Update state before moving
  my_robot.MoveForward(MySpeed);
  Serial.printf("Moving Forward (%c shape) | Pos: x=%.2f, y=%.2f, θ=%.1f°\n", currentShape, x, y, theta);
}

void moveBackward() {
  readSensors();
  my_robot.MoveBackward(MySpeed);
  Serial.printf("Moving Backward (%c shape) | Pos: x=%.2f, y=%.2f, θ=%.1f°\n", currentShape, x, y, theta);
}

void moveLeft() {
  readSensors();
  my_robot.MoveLeft(MySpeed);
  Serial.printf("Moving Left (%c shape) | Pos: x=%.2f, y=%.2f, θ=%.1f°\n", currentShape, x, y, theta);
}

void moveRight() {
  readSensors();
  my_robot.MoveRight(MySpeed);
  Serial.printf("Moving Right (%c shape) | Pos: x=%.2f, y=%.2f, θ=%.1f°\n", currentShape, x, y, theta);
}

void stopRobot() {
  my_robot.stopSmorphi();
  Serial.printf("Stopping | Final Pos: x=%.2f, y=%.2f, θ=%.1f°\n", x, y, theta);
}

// ==================== BLOCK FUNCTIONS - RESET SMORPHI ====================
void resetAllModule() {
  my_robot.sm_reset_M1();
  my_robot.sm_reset_M2();
  my_robot.sm_reset_M3();
  my_robot.sm_reset_M4();
}

// ==================== BLOCK FUNCTIONS - ADVANCED MOVEMENTS ====================
void rotateDegrees(float degrees) {
  readSensors(); // Get current state
  float start_angle = theta;
  float target_angle = start_angle + degrees;
  
  // Normalize target angle to 0-360 range
  while (target_angle >= 360) target_angle -= 360;
  while (target_angle < 0) target_angle += 360;
  
  Serial.printf("Rotating %.1f° from %.1f° to %.1f°\n", degrees, start_angle, target_angle);
  
  // FIXED: Proper turning loop with IMU feedback
  while (true) {
    readSensors(); // Continuously update IMU data
    float angle_diff = angleDifference(target_angle, theta);
    
    // FIXED: Better precision threshold (5 degrees like your friend's code)
    if (abs(angle_diff) < 5.0) break;
    
    // Turn in the correct direction
    if (degrees > 0) {
      my_robot.CenterPivotLeft(TurnSpeed); // Positive = clockwise
    } else {
      my_robot.CenterPivotRight(TurnSpeed);  // Negative = counter-clockwise
    }
    
    Serial.printf("Turning - Current angle: %.2f°, Target: %.2f°, Diff: %.2f°\n", theta, target_angle, angle_diff);
    delay(50); // FIXED: Proper delay for IMU updates
  }
  
  stopRobot();
  Serial.printf("Rotation complete! Current angle: %.1f°\n", theta);
}

void moveStepsTowardsDirection(String direction, int steps) {
  Serial.printf("Moving %d steps towards %s\n", steps, direction.c_str());
  
  for (int step = 0; step < steps; step++) {
    if (direction == "Forward") moveForward();
    else if (direction == "Backward") moveBackward();
    else if (direction == "Left") moveLeft();
    else if (direction == "Right") moveRight();
    
    delay(200); // Step duration
    stopRobot(); // Stop between steps for better control
    delay(25);  // Brief pause between steps
    Serial.printf("Step %d/%d complete\n", step + 1, steps);
  }
  
  stopRobot();
}

// ==================== BLOCK FUNCTIONS - UTILITIES ====================
void waitMs(int milliseconds) {
  unsigned long startTime = millis();
  while (millis() - startTime < milliseconds) {
    readSensors(); // Keep updating state during wait
    delay(10);
  }
}

void printSensorStatus() {
  readSensors();
  Serial.printf("Sensors - Front:%d, Right:%d, Left:%d, Back:%d | Pos: x=%.2f, y=%.2f, θ=%.1f°\n", 
                front_sensor_status, right_sensor_status, left_sensor_status, back_sensor_status, x, y, theta);
}

// ==================== PRESET BEHAVIOR BLOCKS ====================
void basicObstacleAvoidance() {
  if (irDetectsObstacleAt("Front")) {
    moveBackward();
  }
  else if (irDetectsObstacleAt("Right")) {
    moveLeft();
  }
  else if (irDetectsObstacleAt("Left")) {
    moveRight();
  }
  else if (irDetectsObstacleAt("Back")) {
    moveForward();
  }
  else {
    stopRobot();
  }
}

// ==================== BLOCK FUNCTIONS - ROBOT SHAPES ====================
void changeShape(char shape) {
    switch(shape) {
        case 'O': //Change shape to O
            my_robot.O();
            currentShape = 'O';
            break;
        case 'L': //Change shape to L
            my_robot.L();
            currentShape = 'L';
            break;
        case 'I': //Change shape to I
            my_robot.I();
            currentShape = 'I';
            break;
        case 'S': //Change shape to S
            my_robot.S();
            currentShape = 'S';
            break;
        default:
            Serial.println("Unknown shape");
            return;
    }

    Serial.printf("Changed to shape: %c\n", currentShape);
    delay(2000); // Give time for physical reconfiguration
    
    // Force immediate sensor and state update after shape change
    lastSensorRead = 0;
    lastUpdate = 0;
    readSensors();
    
}

// ==================== MAIN PROGRAM - ASSEMBLE YOUR PROGRAM HERE ====================
void originalBlockDiagramBehavior() {
  Serial.println("Starting Original Block Diagram Behavior...");

  // while (true) {
  //   if (irDetectsObstacleAt("Right")) {
  //     moveForward();
  //     break;
  //   } else {
  //     stopRobot();
  //   }
  //   delay(100);
  // }

 
  //Room 1 ======================================
  //1
  while (true) {
    if (irDetectsObstacleAt("Right")) {
      moveForward();
      break;
    } else {
      moveRight();
    }
    delay(100);
  }
  //2
  while (true) {
    if (irDetectsObstacleAt("Front")) {
      moveLeft();
      break;
    } else {
      moveForward();
    }
    delay(100);
  }
  
  // Final stop
  stopRobot();
  Serial.println("Block diagram behavior completed!");
}

// ==================== MAIN SETUP ====================
void setup() {
  initialize_smorphi();
  delay(1000);
}

// ==================== MAIN LOOP ====================
void loop() {
  // 1. Original block diagram behavior (run once)
  static bool programRun = false;
  if (!programRun) {
    //also add read imu data here if you are using IMU
    originalBlockDiagramBehavior();
    programRun = true;
  }
  
  delay(50);
}