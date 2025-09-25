#include <Bluepad32.h>
#include <smorphi.h>
#include <EEPROM.h>
#include <Encoder.h>
// #include <Wire.h>
// #include <MPU6050_light.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "smorphi_odometry.h"
#include <uni.h>
#define LED_PIN 2
#define AXIS_DEAD_ZONE 4
#define AXIS_XY_THRESHOLD 30


struct Pose {
  float x;
  float y;
  float theta;
  uint8_t shape;
};

#define MAX_POSITIONS 100                         // how many positions to store
#define EEPROM_SIZE MAX_POSITIONS * sizeof(Pose)  // allocate 512 bytes in Flash

#define ENCODER_FR_A 19
#define ENCODER_FR_B 18
#define ENCODER_FL_A 16
#define ENCODER_FL_B 17
#define ENCODER_RR_A 25
#define ENCODER_RR_B 23
#define ENCODER_RL_A 26
#define ENCODER_RL_B 27

#define USE_MPU
// #define USE_WIT
// #define ENCODER_FL_A 4
// #define ENCODER_FL_B 5
// #define ENCODER_FR_A 19
// #define ENCODER_FR_B 18
// #define ENCODER_RR_A 25
// #define ENCODER_RR_B 23
// #define ENCODER_RL_A 26
// #define ENCODER_RL_B 27



// ControllerPtr myControllers[BP32_MAX_GAMEPADS];
ControllerPtr myController = nullptr;
Smorphi my_robot;
bool isFirst = true;
#ifdef USE_MPU
// MPU6050 mpu(Wire);

MPU6050 mpu(0x68);
float ypr[3];                  // [yaw, pitch, roll]   Yaw/Pitch/Roll container and gravity vector
int const INTERRUPT_PIN = 27;  // Define the interruption #0 pin
VectorFloat gravity;           // [x, y, z]            Gravity vector
Quaternion q;                  // [w, x, y, z]         Quaternion container

/*---MPU6050 Control/Status Variables---*/
bool DMPReady = false;   // Set true if DMP init was successful
uint8_t MPUIntStatus;    // Holds actual interrupt status byte from MPU
uint8_t devStatus;       // Return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;     // Expected DMP packet size (default is 42 bytes)
uint8_t FIFOBuffer[64];  // FIFO storage buffer
#endif
volatile bool MPUInterrupt = false;
// D6:36:14:7F:6B:1B
// 41:42:C7:16:80:33
// 79:6E:2A:AC:BC:B7
// 63:40:2D:E7:98:D0
static const char* controller_addr_string = "63:40:2D:E7:98:D0";

ESP32Encoder encoder_fr, encoder_fl, encoder_rr, encoder_rl;
smorphi_odometry::MotorProperty_t *motor_prop_fl, *motor_prop_fr, *motor_prop_rl, *motor_prop_rr;
smorphi_odometry::Odometry_t* odometry;
static int stop = 0, stopRotate = 0, stopDpad = 0;
int currentSpeed = 70;        //Linear
int currentRotSpeed = 180;    // Rot
const int minSpeed = 50;      // Minimum speed
const int maxSpeed = 100;     // Maximum speed
const int minSpeedRot = 130;  // Minimum speed
const int maxSpeedRot = 270;  // Maximum speed


const int speedIncrement = 10;  // Speed change per button press
bool emergencyBrake = false;    // L2 emergency brake


unsigned long lastJoyUpdate;
Pose currentPose = { 0, 0, 0, 0 };
int savedCount = 0;
double errorYaw = 0;



// Map joystick axis to linear speed
int getSpeedFromAxis(int axisValue) {
  int absVal = abs(axisValue);
  if (absVal < 50) return 0;                    // Dead zone
  if (absVal < 270) return currentSpeed * 0.4;  // Slow
  if (absVal < 370) return currentSpeed * 0.7;  // Medium
  return currentSpeed;                          // Max
}

// Map joystick axis to rotational speed
int getSpeedRotFromAxis(int axisValue) {
  int absVal = abs(axisValue);
  if (absVal < 50) return 0;                 // Dead zone
  if (absVal < 300) return currentRotSpeed;  // Slow
  if (absVal < 400) return currentRotSpeed;  // Medium
  return currentRotSpeed;                    // Max
}

void resetMotors() {
  my_robot.sm_reset_M1();
  my_robot.sm_reset_M2();
  my_robot.sm_reset_M3();
  my_robot.sm_reset_M4();
}

void IRAM_ATTR DMPDataReady() {
  MPUInterrupt = true;
}


// Handle special buttons (L1, L2, R1, R2)
void handleSpecialButtons(ControllerPtr ctl) {
  static bool lastR1 = false, lastR2 = false;
  bool currentR1 = ctl->r1();
  bool currentR2 = ctl->r2();



  // R1 - Increase speed
  if (currentR1 && !lastR1) {
    currentSpeed = min(currentSpeed + speedIncrement, maxSpeed);
    currentRotSpeed = min(currentRotSpeed + speedIncrement, maxSpeedRot);
    // Serial.printf("Speed increased to: %d\n", currentSpeed);
  }

  // R2 - Decrease speed
  if (currentR2 && !lastR2) {
    currentSpeed = max(currentSpeed - speedIncrement, minSpeed);
    currentRotSpeed = max(currentRotSpeed - speedIncrement, minSpeedRot);
    // Serial.printf("Speed decreased to: %d\n", currentSpeed);
  }

  // Update last states
  lastR1 = currentR1;
  lastR2 = currentR2;
}



// Handle joystick-based movement
void handleJoystickMovement(ControllerPtr ctl, int& stop) {
  int leftX = ctl->axisX();
  int leftY = ctl->axisY();
  int rightX = ctl->axisRX();
  int speedX = getSpeedFromAxis(leftX);
  int speedY = getSpeedFromAxis(leftY);
  int rotSpeed = getSpeedRotFromAxis(rightX);

  // Serial.printf("leftX: %d | leftY: %d\n", leftX, leftY);
  //Make the Kinematic only works with Linear Movement not Rotate it effect when  we change shape
  if (rightX < -AXIS_DEAD_ZONE) {
    my_robot.CenterPivotLeft(rotSpeed);
    // currentPose.x = 0;
    // currentPose.y = 0;
    // odometry->setResetPose();
    // Serial.printf("CenterPivotLeft at speed %d\n", rotSpeed);
    stopRotate = 1;
  } else if (rightX > AXIS_DEAD_ZONE) {
    my_robot.CenterPivotRight(rotSpeed);
    // currentPose.x = 0;
    // currentPose.y = 0;
    // odometry->setResetPose();
    // Serial.printf("CenterPivotRight at speed %d\n", rotSpeed);
    stopRotate = 1;
  } else if (abs(rightX) < AXIS_DEAD_ZONE || stopRotate == 1) {
    stopRotate = 0;
    my_robot.stopSmorphi();
    resetMotors();
  }

  if (leftY < -AXIS_DEAD_ZONE && abs(leftX) < AXIS_XY_THRESHOLD) {
    my_robot.MoveForward(speedY);
    // Serial.printf("MoveForward at speed %d\n", speedY);
    stop = 1;
  } else if (leftY > AXIS_DEAD_ZONE && abs(leftX) < AXIS_XY_THRESHOLD) {
    my_robot.MoveBackward(speedY);
    // Serial.printf("MoveBackward at speed %d\n", speedY);
    stop = 1;
  } else if (leftX > AXIS_DEAD_ZONE && abs(leftY) < AXIS_XY_THRESHOLD) {
    my_robot.MoveRight(speedX);
    // Serial.printf("MoveRight at speed %d\n", speedX);
    stop = 1;
  } else if (leftX < -AXIS_DEAD_ZONE && abs(leftY) < AXIS_XY_THRESHOLD) {
    my_robot.MoveLeft(speedX);
    // Serial.printf("MoveLeft at speed %d\n", speedX);
    stop = 1;
  } else if (leftX < -AXIS_DEAD_ZONE && leftY < -AXIS_DEAD_ZONE) {
    int spd = min(speedX, speedY);
    my_robot.MoveDiagUpLeft(spd);
    // Serial.printf("MoveDiagUpLeft at speed %d\n", spd);
    stop = 1;
  } else if (leftX > AXIS_DEAD_ZONE && leftY < -AXIS_DEAD_ZONE) {
    int spd = min(speedX, speedY);
    my_robot.MoveDiagUpRight(spd);
    // Serial.printf("MoveDiagUpRight at speed %d\n", spd);
    stop = 1;
  } else if (leftX < -AXIS_DEAD_ZONE && leftY > AXIS_DEAD_ZONE) {
    int spd = min(speedX, speedY);
    my_robot.MoveDiagDownLeft(spd);
    // Serial.printf("MoveDiagDownLeft at speed %d\n", spd);
    stop = 1;
  } else if (leftX > AXIS_DEAD_ZONE && leftY > AXIS_DEAD_ZONE) {
    int spd = min(speedX, speedY);
    my_robot.MoveDiagDownRight(spd);
    // Serial.printf("MoveDiagDownRight at speed %d\n", spd);
    stop = 1;
  } else if (abs(leftX) < AXIS_DEAD_ZONE && abs(leftY) < AXIS_DEAD_ZONE || stop == 1) {
    resetMotors();
    my_robot.stopSmorphi();
    // Serial.println("STOP ROBOT");
    stop = 0;
  }
}
// Save pose to EEPROM
void savePoseToEEPROM(Pose p, int index) {
  if (index < 0 || index >= MAX_POSITIONS) return;
  int addr = index * sizeof(Pose);
  EEPROM.put(addr, p);
  EEPROM.commit();
}

// Load pose from EEPROM
Pose loadPoseFromEEPROM(int index) {
  Pose p;
  if (index < 0 || index >= MAX_POSITIONS) return { 0, 0, 0, 0 };
  int addr = index * sizeof(Pose);
  EEPROM.get(addr, p);
  return p;
}

// Debug gamepad state
void dumpGamepad(ControllerPtr ctl) {
  Serial.printf(
    "idx=%d, dpad: 0x%02x, buttons: 0x%04x, axis L: %4d, %4d, axis R: %4d, %4d, brake: %4d, throttle: %4d\n",
    ctl->index(), ctl->dpad(), ctl->buttons(),
    ctl->axisX(), ctl->axisY(), ctl->axisRX(), ctl->axisRY(),
    ctl->brake(), ctl->throttle());
}

// Callback for controller connection
void onConnectedController(ControllerPtr ctl) {
  if (myController == nullptr) {
    Serial.printf("CALLBACK: Controller is connected\n");
    ControllerProperties properties = ctl->getProperties();
    Serial.printf("Controller model: %s, VID=0x%04x, PID=0x%04x\n",
                  ctl->getModelName().c_str(), properties.vendor_id, properties.product_id);
    myController = ctl;
  } else {
    Serial.println("CALLBACK: Controller connected, but slot is already occupied");
  }
}

// Callback for controller disconnection
void onDisconnectedController(ControllerPtr ctl) {
  if (myController == ctl) {
    Serial.printf("CALLBACK: Controller disconnected\n");
    myController = nullptr;
  } else {
    Serial.println("CALLBACK: Controller disconnected, but not found");
  }
}

void handleShapeButtons(ControllerPtr ctl) {
  switch (ctl->buttons()) {
    case 0x0004:
      {

        my_robot.O();
        // Serial.println("SHAPE O");
        // delay(500);
        currentPose.shape = 1;
        // currentPose.x = 0;
        // currentPose.y = 0;
        // odometry->setResetPose();
        break;
      }  // Square

    case 0x0001:  // X

      my_robot.I();
      // delay(500);
      currentPose.shape = 0;
      // currentPose.x = 0;
      // currentPose.y = 0;
      // odometry->setResetPose();
      // Serial.println("SHAPE I");
      break;
    case 0x0002:  // Circle
      my_robot.L();
      // delay(500);
      currentPose.shape = 2;
      // currentPose.x = 0;
      // currentPose.y = 0;
      // odometry->setResetPose();
      // Serial.println("SHAPE L");
      break;
    case 0x0008:  // Triangle
      my_robot.T();
      // delay(500);
      // currentPose.x = 0;
      // currentPose.y = 0;
      // odometry->setResetPose();
      currentPose.shape = 3;
      // Serial.println("SHAPE T");
      break;
    default:
      break;
  }
}

void handleDpadMovement(ControllerPtr ctl) {
  uint8_t curr_dpad = ctl->dpad();
  if (curr_dpad == 0x01) {
    my_robot.MoveForward(70);
    // Serial.println("Forward Speed 100");
    stopDpad = 1;
  } else if (curr_dpad == 0x02) {
    my_robot.MoveBackward(70);
    // Serial.println("Forward Speed 100");
    stopDpad = 1;
  } else if (curr_dpad == 0x08) {
    my_robot.MoveLeft(70);
    // Serial.println("Left Speed 100");
    stopDpad = 1;
  } else if (curr_dpad == 0x04) {
    my_robot.MoveRight(70);
    stopDpad = 1;
    // Serial.println("Right Speed 100");
  } else if (curr_dpad > 0 || stopDpad == 1) {
    my_robot.stopSmorphi();
    resetMotors();
    stopDpad = 0;
  }
}
// Process gamepad input
void processGamePad(ControllerPtr ctl) {
  static uint8_t prev_buttons = 0;
  handleSpecialButtons(ctl);
  handleJoystickMovement(ctl, stop);
  // Handle D-pad movement
  handleDpadMovement(ctl);
  handleShapeButtons(ctl);

  // if (ctl->l1()) {
  //   savePoseToEEPROM(currentPose, savedCount);
  //   Serial.printf("Saved Pose[%d]: x=%.2f, y=%.2f, th=%.2f\n",
  //                 savedCount, currentPose.x, currentPose.y, currentPose.theta);
  //   savedCount++;
  //   // currentPose = Pose();
  //   currentPose.x = 0;
  //   currentPose.y = 0;
  //   odometry->setResetPose();
  // } else if (ctl->l2()) {
  //   // currentPose = Pose();
  //   currentPose.x = 0;
  //   currentPose.y = 0;

  //   odometry->setResetPose();
  // }

  // Handle button actions
  if (ctl->buttons() > 0 && prev_buttons != ctl->buttons()) {
    switch (ctl->buttons()) {
      case 0x0010:  // Save pose button
        savePoseToEEPROM(currentPose, savedCount);
        Serial.printf("Saved Pose[%d]: x=%.2f, y=%.2f, th=%.2f\n",
                      savedCount, currentPose.x, currentPose.y, currentPose.theta);
        savedCount++;
        // currentPose = Pose();
        // delay(500);
        currentPose.x = 0;
        currentPose.y = 0;
        odometry->setResetPose();
        break;
      case 0x0040:  // Rese pose button
        // currentPose = Pose();
        currentPose.x = 0;
        currentPose.y = 0;
        odometry->setResetPose();
        // delay(500);
        break;
      default:
        break;
    }
  }
  prev_buttons = ctl->buttons();

  // // Reset motors
  // my_robot.sm_reset_M1();
  // my_robot.sm_reset_M2();
  // my_robot.sm_reset_M3();
  // my_robot.sm_reset_M4();
}

void setup() {
  Serial.begin(115200);
  EEPROM.begin(EEPROM_SIZE);
  Serial.printf("Firmware: %s\n", BP32.firmwareVersion());
  const uint8_t* addr = BP32.localBdAddress();
  Serial.printf("BD Addr: %2X:%2X:%2X:%2X:%2X:%2X\n",
                addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);

  bd_addr_t controller_addr;
  // Parse human-readable Bluetooth address.
  sscanf_bd_addr(controller_addr_string, controller_addr);
  // Notice that this address will be added in the Non-volatile-storage (NVS).
  // If the device reboots, the address will still be stored.
  // Adding a duplicate value will do nothing.
  // You can add up to four entries in the allowlist.
  uni_bt_allowlist_add_addr(controller_addr);
  // Finally, enable the allowlist.
  // Similar to the "add_addr", its value gets stored in the NVS.
  uni_bt_allowlist_set_enabled(true);

  BP32.setup(&onConnectedController, &onDisconnectedController);
  BP32.forgetBluetoothKeys();

  my_robot.BeginSmorphi();
  Wire.begin();


  ESP32Encoder::useInternalWeakPullResistors = puType::up;
  encoder_fr.attachHalfQuad(ENCODER_FR_A, ENCODER_FR_B);
  encoder_fl.attachHalfQuad(ENCODER_FL_A, ENCODER_FL_B);
  encoder_rr.attachHalfQuad(ENCODER_RR_A, ENCODER_RR_B);
  encoder_rl.attachHalfQuad(ENCODER_RL_A, ENCODER_RL_B);
  encoder_fr.clearCount();
  encoder_fl.clearCount();
  encoder_rl.clearCount();
  encoder_rr.clearCount();

  motor_prop_fl = new smorphi_odometry::MotorProperty_t(&encoder_fl, my_robot.sm_wheel_radius, 540);
  motor_prop_fr = new smorphi_odometry::MotorProperty_t(&encoder_fr, my_robot.sm_wheel_radius, 540);
  motor_prop_rl = new smorphi_odometry::MotorProperty_t(&encoder_rl, my_robot.sm_wheel_radius, 540);
  motor_prop_rr = new smorphi_odometry::MotorProperty_t(&encoder_rr, my_robot.sm_wheel_radius, 540);
  odometry = new smorphi_odometry::Odometry_t(motor_prop_fl, motor_prop_fr, motor_prop_rl, motor_prop_rr,
                                              my_robot.sm_wheel_x, my_robot.sm_wheel_y);

#ifdef USE_MPU

  // byte status = mpu.begin();
  // Serial.print(F("MPU6050 status: "));
  // Serial.println(status);
  // while (status != 0) {}
  // Serial.println(F("Calculating offsets, do not move MPU6050"));
  // delay(1000);
  // mpu.calcOffsets();
  // Serial.println("Done!\n");
  // #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  //   Wire.begin(0x68);
  //   Wire.setClock(400000);  // 400kHz I2C clock. Comment this line if having compilation difficulties
  // #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  //   Fastwire::setup(400, true);
  // #endif
  mpu.initialize();
  Serial.printf("IMU %d \n", mpu.getDeviceID());
  pinMode(INTERRUPT_PIN, INPUT);
  if (mpu.testConnection() == true) {
    Serial.println("MPU6050 connection failed");
    while (true)
      ;
  } else {
    Serial.println("MPU6050 connection successful");
  }

  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  /* Supply your gyro offsets here, scaled for min sensitivity */
  mpu.setXGyroOffset(0);
  mpu.setYGyroOffset(0);
  mpu.setZGyroOffset(0);
  mpu.setXAccelOffset(0);
  mpu.setYAccelOffset(0);
  mpu.setZAccelOffset(0);

  if (devStatus == 0) {
    mpu.CalibrateAccel(6);  // Calibration Time: generate offsets and calibrate our MPU6050
    mpu.CalibrateGyro(6);
    Serial.println("These are the Active offsets: ");
    mpu.PrintActiveOffsets();
    Serial.println(F("Enabling DMP..."));  //Turning ON DMP
    mpu.setDMPEnabled(true);

    /*Enable Arduino interrupt detection*/
    Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
    Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
    Serial.println(F(")..."));
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), DMPDataReady, RISING);
    MPUIntStatus = mpu.getIntStatus();

    /* Set the DMP Ready flag so the main loop() function knows it is okay to use it */
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    DMPReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();  //Get expected DMP packet size for later comparison
  } else {
    Serial.print(F("DMP Initialization failed (code "));  //Print the error code
    Serial.print(devStatus);
    Serial.println(F(")"));
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
  }

#endif

#ifdef USE_WIT
  // Initialize Serial2 for IMU communication
  Serial2.begin(9600, SERIAL_8N1, 16, 17);  // RX=16, TX=17

  Serial.println("WT901CTTL IMU Test Started");
  Serial.println("Waiting for IMU data...");

  delay(1000);
#endif
}

void loop() {
  unsigned long now = millis();

  // 1. Gamepad (fast)
  static unsigned long lastJoyUpdate = 0;
  if (now - lastJoyUpdate >= 30) {
    lastJoyUpdate = now;
    if (BP32.update()) {
      if (myController && myController->isConnected() && myController->isGamepad()) {
        processGamePad(myController);
      }
    }
  }

#ifdef USE_MPU
  // if (DMPReady && MPUInterrupt) {
  //     MPUInterrupt = false;  // reset flag

  //     // get current FIFO count
  //     uint16_t fifoCount = mpu.getFIFOCount();

  //     // check for overflow
  //     if (fifoCount == 1024) {
  //         mpu.resetFIFO();
  //         Serial.println("FIFO overflow!");
  //         return;
  //     }

  //     // check for available data
  //     while (fifoCount < packetSize) {
  //         fifoCount = mpu.getFIFOCount();
  //     }

  //     // read a packet
  //     mpu.getFIFOBytes(FIFOBuffer, packetSize);

  //     // process quaternion and yaw/pitch/roll
  //     mpu.dmpGetQuaternion(&q, FIFOBuffer);
  //     mpu.dmpGetGravity(&gravity, &q);
  //     mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

  //     currentPose.theta = fmod(-ypr[0] * 180 / M_PI, 360);
  //     if (currentPose.theta < 0) currentPose.theta += 360;

  //     odometry->setOrientation(currentPose.theta * M_PI / 180.0);
  // }

  if (mpu.dmpGetCurrentFIFOPacket(FIFOBuffer)) {
    mpu.dmpGetQuaternion(&q, FIFOBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    currentPose.theta = fmod(-ypr[0] * 180 / M_PI, 360);
    if (currentPose.theta < 0) currentPose.theta += 360;

    odometry->setOrientation(currentPose.theta * M_PI / 180.0);
  }
#endif



  // #ifdef USE_WIT
  //   if (Serial2.available() >= 11) {
  //     uint8_t buffer[11];

  //     // Read 11 bytes (standard WT901 packet size)
  //     for (int i = 0; i < 11; i++) {
  //       buffer[i] = Serial2.read();
  //     }

  //     // Check for valid packet header (0x55)
  //     if (buffer[0] == 0x55 && buffer[1] == 0x53) {
  //       // parseIMUData(buffer);
  //       int16_t yaw = (buffer[7] << 8) | buffer[6];
  //       float angle_yaw = yaw / 32768.0 * 180;
  //       if (isFirst == true) {
  //         errorYaw = angle_yaw;
  //         isFirst = false;
  //       }

  //       angle_yaw -= errorYaw;
  //       if (angle_yaw < 0)
  //         angle_yaw += 360;
  //       currentPose.theta = fmod(angle_yaw, 360);
  //       if (currentPose.theta < 0) currentPose.theta += 360;
  //       odometry->setOrientation(currentPose.theta * M_PI / 180.0);
  //     }
  //   }
  // #endif


  static unsigned long lastOdometryUpdate = 0;
  if (now - lastOdometryUpdate >= 50) {
    lastOdometryUpdate = now;
    Serial.printf("Encoder fl(1): %d | fr(2) : %d | rr(3) : %d | rl(4) : %d \n", motor_prop_fl->get_pulse(), motor_prop_fr->get_pulse(), motor_prop_rr->get_pulse(), motor_prop_rl->get_pulse());
    // motor_prop_fl->update();
    // motor_prop_fr->update();
    // motor_prop_rl->update();
    // motor_prop_rr->update();
    odometry->update();
    // currentPose.theta = radian;
    // currentPose.theta = ypr[0];

    currentPose.x = odometry->pos_x();
    currentPose.y = odometry->pos_y();

    // Serial.printf("x: %.3f | y: %.3f\n", odometry->vel_x(), odometry->vel_y());
    // Serial.printf("x=%.3f y=%.3f th=%.3f shape: %d\n", currentPose.x, currentPose.y, currentPose.theta, currentPose.shape);
  }
}
