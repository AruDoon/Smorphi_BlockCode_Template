#define ENCODER_DO_NOT_USE_INTERRUPTS
// #include <Encoder.h>
// #include "smorphi_single.h"
#include "smorphi.h"
#include "math.h"
// #include "Wire.h"
// #include <MPU6050_light.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <EEPROM.h>
#include "smorphi_odometry.h"


#define MAX_POSITIONS 100                         // how many positions to store
#define EEPROM_SIZE MAX_POSITIONS * sizeof(Pose)  // allocate 512 bytes in Flash

// #define USE_WIFI

// const char* ssid = "DIKA";
// const char* password = "30101973";
#ifdef USE_WIFI
#include <WiFi.h>
#include <WiFiUdp.h>
const char* ssid = "Barelang63";
const char* password = "barelang63";

WiFiUDP udp;
const unsigned int localUdpPort = 9999;
char incomingPacket[255];  // buffer
IPAddress remoteIP;
unsigned int remotePort;
String action;
#endif


struct Pose {
  float x;
  float y;
  float theta;
  uint8_t shape;
};

Pose targetPose = { 0, 0, 0, 0 };
Pose loadPoseFromEEPROM(int index) {
  Pose p;
  if (index < 0 || index >= MAX_POSITIONS) return { 0, 0, 0, 0 };
  int addr = index * sizeof(Pose);
  EEPROM.get(addr, p);
  return p;
}


MPU6050 mpu(0x68);
float ypr[3] = { 0, 0, 0 };    // [yaw, pitch, roll]   Yaw/Pitch/Roll container and gravity vector
int const INTERRUPT_PIN = 27;  // Define the interruption #0 pin
VectorFloat gravity;           // [x, y, z]            Gravity vector
Quaternion q;                  // [w, x, y, z]         Quaternion container

/*---MPU6050 Control/Status Variables---*/
bool DMPReady = false;   // Set true if DMP init was successful
uint8_t MPUIntStatus;    // Holds actual interrupt status byte from MPU
uint8_t devStatus;       // Return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;     // Expected DMP packet size (default is 42 bytes)
uint8_t FIFOBuffer[64];  // FIFO storage buffer
volatile bool MPUInterrupt = false;

// MPU6050 mpu(Wire);
#define ENCODER_FR_A 19
#define ENCODER_FR_B 18
#define ENCODER_FL_A 16
#define ENCODER_FL_B 17
#define ENCODER_RR_A 25
#define ENCODER_RR_B 23
#define ENCODER_RL_A 26
#define ENCODER_RL_B 27

// #define ENCODER_FL_A 4
// #define ENCODER_FL_B 5
// #define ENCODER_FR_A 19
// #define ENCODER_FR_B 18
// #define ENCODER_RR_A 25
// #define ENCODER_RR_B 23
// #define ENCODER_RL_A 26
// #define ENCODER_RL_B 27



unsigned long lastUpdate = 0;

// PID parameters
double kp_rot = 120;  // Reduced to prevent overshooting
double ki_rot = 20;   // Reduced to avoid windup
double kd_rot = 1.0;  // Slightly increased for damping

double kp_lin = 120;  // Reduced for smoother linear motion
double ki_lin = 20;   // Reduced to avoid windup
double kd_lin = 1.0;  // Slightly increased for damping

double integral = 0.0, integral_lin = 0.0;
double lastError = 0.0, lastError_lin = 0.0;

ESP32Encoder encoder_fr, encoder_fl, encoder_rr, encoder_rl;
smorphi_odometry::MotorProperty_t *motor_prop_fl, *motor_prop_fr, *motor_prop_rl, *motor_prop_rr;
smorphi_odometry::Odometry_t* odometry;
// Encoder motor1(18, 19);
// Encoder motor2(25, 23);
// Encoder motor3(27, 26);
// Encoder motor4(4, 5);

Smorphi my_robot;
// constants
const double R = 0.03;      // sm_wheel_radius
const double lx = 0.04925;  // sm_wheel_x
const double ly = 0.035;    // sm_wheel_y
const double L = lx + ly;
const int N = 540;     // ticks per rev (set yours)
const double dt = 20;  // 20 ms

// state
long p1 = 0, p2 = 0, p3 = 0, p4 = 0;
// double x = 0, y = 0, theta = 0;
Pose currentPose = { 0, 0, 0, 0 };
unsigned long t0 = 0;
int targetIndex = 0;
uint8_t prev_shape = 0;
bool running = true;  // Flag to control continuous navigation

void DMPDataReady() {
  MPUInterrupt = true;
}

void odomFromEnc(long e1, long e2, long e3, long e4) {
  long d1 = e1 - p1, d2 = e2 - p2, d3 = e3 - p3, d4 = e4 - p4;
  p1 = e1;
  p2 = e2;
  p3 = e3;
  p4 = e4;

  // ticks -> wheel linear speed (m/s)
  const double m_per_tick = (2.0 * M_PI * R) / N;
  double vFL = (d1 * m_per_tick) / dt;
  double vFR = (d2 * m_per_tick) / dt;
  double vRL = (d3 * m_per_tick) / dt;
  double vRR = (d4 * m_per_tick) / dt;

  double vx = 0.25 * (vFL + vFR + vRL + vRR);
  double vy = 0.25 * (-vFL + vFR - vRL + vRR);
  double w = 0.25 * (-vFL + vFR - vRL + vRR) / L;  //Ini perlu dicek


  // double radian = fmod(mpu.getAngleZ(), 360);
  // if (radian < 0)
  //   radian += 360;

  // theta = radian;
  // // integrate to world frame
  // currentPose.y += (vx * sin(radian * M_PI / 180.0) + vy * cos(radian * M_PI / 180.0)) * dt;
  // currentPose.x += (vx * cos(radian * M_PI / 180.0) - vy * sin(radian * M_PI / 180.0)) * dt;
  // theta += w * dt;

  // y += vy * dt;
  // x += vx * dt;
  // theta += w * dt;
  // Serial.printf("vx=%.3f vy=%.3f w=%.3f | x=%.3f y=%.3f th=%.3f\n", vx, vy, w, x, y, theta);
}

void setup() {
  Serial.begin(115200);
  Serial.println("Basic NoInterrupts Test:");
  my_robot.BeginSmorphi();

  EEPROM.begin(EEPROM_SIZE);

  ESP32Encoder::useInternalWeakPullResistors = puType::up;
  encoder_fr.attachHalfQuad(ENCODER_FR_A, ENCODER_FR_B);
  encoder_fl.attachHalfQuad(ENCODER_FL_A, ENCODER_FL_B);
  encoder_rr.attachHalfQuad(ENCODER_RR_A, ENCODER_RR_B);
  encoder_rl.attachHalfQuad(ENCODER_RL_A, ENCODER_RL_B);
  encoder_fr.clearCount();
  encoder_fl.clearCount();
  encoder_rl.clearCount();
  encoder_rr.clearCount();

  motor_prop_fl = new smorphi_odometry::MotorProperty_t(&encoder_fl, my_robot.sm_wheel_radius, N);
  motor_prop_fr = new smorphi_odometry::MotorProperty_t(&encoder_fr, my_robot.sm_wheel_radius, N);
  motor_prop_rl = new smorphi_odometry::MotorProperty_t(&encoder_rl, my_robot.sm_wheel_radius, N);
  motor_prop_rr = new smorphi_odometry::MotorProperty_t(&encoder_rr, my_robot.sm_wheel_radius, N);
  odometry = new smorphi_odometry::Odometry_t(motor_prop_fl, motor_prop_fr, motor_prop_rl, motor_prop_rr, my_robot.sm_wheel_x, my_robot.sm_wheel_y);


  // Start UDP
  #ifdef USE_WIFI
  running = false;
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.print("Ip: ");
  Serial.println(WiFi.localIP());

  udp.begin(localUdpPort);
  Serial.printf("Listening UDP on port %d\n", localUdpPort);
  #endif


  mpu.initialize();
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
}


#ifdef USE_WIFI
void handleCommand(String cmd) {
  // Split command at '|'
  int sepIndex = cmd.indexOf('|');
  action = cmd;
  String param = "";

  if (sepIndex != -1) {
    action = cmd.substring(0, sepIndex);
    param = cmd.substring(sepIndex + 1);
  }
  if (action == "CP1") {
    targetIndex = param.toInt();  // use param as index
    currentPose.x = 0;
    currentPose.y = 0;
    odometry->setResetPose();
    Serial.printf("Goto Checkpoint 1, index=%d\n", targetIndex);
  } else if (action == "CP2") {
    targetIndex = param.toInt();
    currentPose.x = 0;
    currentPose.y = 0;
    odometry->setResetPose();
    Serial.printf("Goto Checkpoint 2, index=%d\n", targetIndex);
  } else if (action == "CP3") {
    targetIndex = param.toInt();
    currentPose.x = 0;
    currentPose.y = 0;
    odometry->setResetPose();
    Serial.printf("Goto Checkpoint 3, index=%d\n", targetIndex);
  } else if (action == "START") {
    running = true;
    Serial.println("Robot Start!");
  } else if (action == "STOP") {
    running = false;
    Serial.println("Robot Stop!");
    my_robot.stopSmorphi();
    my_robot.sm_reset_M1();
    my_robot.sm_reset_M2();
    my_robot.sm_reset_M3();
    my_robot.sm_reset_M4();
    integral = 0;
    lastError = 0;
    lastError_lin = 0;
    integral_lin = 0;
  }
}

void sendPoseData() {
  if (remotePort == 0) return;  // Don't send if no command received yet

  char buffer[128];
  snprintf(buffer, sizeof(buffer),
           "CUR:%.2f,%.2f,%.2f,%d|TAR:%.2f,%.2f,%.2f,%d|IDX:%d",
           currentPose.x, currentPose.y, currentPose.theta, currentPose.shape,
           targetPose.x, targetPose.y, targetPose.theta, targetPose.shape, targetIndex);
  udp.beginPacket(remoteIP, remotePort);
  udp.write((uint8_t*)buffer, strlen(buffer));
  udp.endPacket();

  // Serial.printf("Sent Pose: %s\n", buffer);
}
#endif


void loop() {
  if (mpu.dmpGetCurrentFIFOPacket(FIFOBuffer)) {  // Get the Latest packet
    /* Display Euler angles in degrees */
    mpu.dmpGetQuaternion(&q, FIFOBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    currentPose.theta = fmod(-ypr[0] * 180 / M_PI, 360);
    if (currentPose.theta < 0) currentPose.theta += 360;
    odometry->setOrientation(currentPose.theta * M_PI / 180.0);
  }
  
  static unsigned long lastOdomTime = 0;
  if (millis() - lastOdomTime >= 50) {
    lastOdomTime = millis();
    // Update motors and odometry
    motor_prop_fl->update();
    motor_prop_fr->update();
    motor_prop_rl->update();
    motor_prop_rr->update();


    odometry->update();
    // odometry->setOrientation(ypr[0] * M_PI / 180.0);
    currentPose.x = odometry->pos_x();
    currentPose.y = odometry->pos_y();
    // currentPose.theta = ypr[0];
  }

  if (millis() - lastUpdate >= 30) {
    lastUpdate = millis();
    // Navigation logic if running
    if (running) {
      targetPose = loadPoseFromEEPROM(targetIndex);

      // --- error computation ---

      double errorTheta = targetPose.theta - currentPose.theta;
      double errorX = targetPose.x - currentPose.x;
      double errorY = targetPose.y - currentPose.y;

      //  double errorTheta = 200 - currentPose.theta;
      // double errorX = 0 - currentPose.x;
      // double errorY = 0 - currentPose.y;


      Serial.printf("Error Global: %.2f | %.2f | %.2f \n", errorX, errorY, errorTheta);

      double errorLocalX = errorX * cos(currentPose.theta * M_PI / 180.0) + errorY * sin(currentPose.theta * M_PI / 180.0);
      double errorLocalY = -errorX * sin(currentPose.theta * M_PI / 180.0) + errorY * cos(currentPose.theta * M_PI / 180.0);

      double errorLinear = fabs(errorLocalX) > fabs(errorLocalY) ? errorLocalX : errorLocalY;
      // double errorLinear = sqrt(errorLocalX * errorLocalX + errorLocalY * errorLocalY);
      while (errorTheta > 180) errorTheta -= 360;
      while (errorTheta < -180) errorTheta += 360;


      // integral += errorTheta * dt;
      // integral = constrain(integral, -270, 270);
      // double derivative = (errorTheta - lastError) / dt;
      // lastError = errorTheta;
      // double controlRot = (kp_rot * errorTheta) + (ki_rot * integral) + (kd_rot * derivative);
      // double speedRot = constrain(controlRot, -270, 270);

      // // PID for linear movement
      // integral_lin += errorLinear * dt;
      // integral_lin = constrain(integral_lin, -500, 500);
      // double derivative_lin = (errorLinear - lastError_lin) / dt;
      // lastError_lin = errorLinear;
      // double speedLinear = (kp_lin * errorLinear) + (ki_lin * integral_lin) + (kd_lin * derivative_lin);
      // speedLinear = constrain(speedLinear, -100, 100);

      // Serial.printf("index = %d |TargetPose: %.2f | %.2f | %.2f | %d  \n", targetIndex, targetPose.x, targetPose.y, targetPose.theta, targetPose.shape);
      // Serial.printf("currnetPose: %.2f | %.2f | %.2f | %d \n", currentPose.x, currentPose.y, currentPose.theta, currentPose.shape);
      // Serial.printf("Error Local: %.2f | %.2f | %.2f \n", errorLocalX, errorLocalY, errorTheta);
      // Serial.printf("Error Linear: %.2f | SpeedLinear: %.2f | integral_lin: %.2f \n", errorLinear, speedLinear, integral_lin);

      if (targetIndex >= MAX_POSITIONS) {
        running = false;
        my_robot.stopSmorphi();
        my_robot.sm_reset_M1();
        my_robot.sm_reset_M2();
        my_robot.sm_reset_M3();
        my_robot.sm_reset_M4();
      }

      // else if (fabs(errorLinear) > 0.03 || fabs(errorTheta) > 3) {
      //   // double vx = (speedLinear / 100) * (errorLocalX / errorLinear);
      //   // double vy = (speedLinear / 100) * (errorLocalY / errorLinear);
      //   // double w = speedRot * M_PI / 180.0;
      //   double vx = 0.0, vy = 0.0, w = 0.0;
      //   int flag = 1;  // Default to curve mapping for combined motion


      //   // Determine dominant error for flag selection
      //   if (fabs(errorTheta) > 3 && errorLinear < 0.03) {
      //     // Pure rotation (theta error dominates)
      //     w = speedRot * M_PI / 180.0 * my_robot.sm_max_angular_speed;
      //     flag = 2;  // Angular mapping
      //   } else if (fabs(errorLinear) > 0.03 && fabs(errorTheta) <= 3) {
      //     // Linear motion (X/Y error dominates)
      //     if (fabs(errorLinear) > 0) {  // Avoid divide-by-zero
      //       vx = (speedLinear / 100.0)  * my_robot.sm_max_linear_speed;
      //       vy = (speedLinear / 100.0)  * my_robot.sm_max_linear_speed;
      //     }
      //     if (fabs(errorLocalX) > fabs(errorLocalY) * 1.5) {
      //       vy = 0;    // Pure X motion
      //       flag = 0;  // Linear mapping
      //     } else if (fabs(errorLocalY) > fabs(errorLocalX) * 1.5) {
      //       vx = 0;    // Pure Y motion
      //       flag = 0;  // Linear mapping
      //     } else {
      //       flag = 3;  // Diagonal mapping for X and Y
      //     }
      //   } else {
      //     // Combined motion (X, Y, theta errors)
      //     if (fabs(errorLinear) > 0) {
      //       vx = (speedLinear / 100.0)  * my_robot.sm_max_curve_speed;
      //       vy = (speedLinear / 100.0)  * my_robot.sm_max_curve_speed;
      //     }
      //     w = speedRot * M_PI / 180.0 * my_robot.sm_max_angular_speed;
      //     flag = 1;  // Curve mapping
      //   }

      //   Serial.printf("Velocities: vx=%.2f, vy=%.2f, w=%.2f, flag=%d\n", vx, vy, w, flag);
      //   my_robot.sm_velocity_handler(vx, -vy, w * 10);
      //   my_robot.sm_pwm_handler(flag);
      //   my_robot.MotorDirection();


      //   // Serial.printf("vx: %.2f | vy: %.2f | w: %.2f\n", vx, vy, w);
      //   // my_robot.sm_velocity_handler(vx, -vy, w);
      //   // my_robot.sm_pwm_handler(1);
      //   // my_robot.MotorDirection();
      // }



      else if (fabs(errorTheta) > 3) {
        // --- PID terms ---
        integral += errorTheta * dt;
        integral = constrain(integral, -50, 50);
        double derivative = (errorTheta - lastError) / dt;
        lastError = errorTheta;

        double control = (kp_rot * errorTheta) + (ki_rot * integral) + (kd_rot * derivative);

        double speed = (control * M_PI / 180.0);
        speed = constrain(control, -180, 180);
        Serial.printf("Speed Angular: %.2f", speed);
        if (speed > 0)
          my_robot.CenterPivotLeft(abs(speed));
        else
          my_robot.CenterPivotRight(abs(speed));
      } else if (fabs(errorLinear) > 0.01) {
        integral_lin += errorLinear * dt;
        integral_lin = constrain(integral_lin, -50, 50);
        double derivative = (errorLinear - lastError) / dt;
        lastError = errorLinear;

        double speedLinear = (kp_lin * errorLinear) + (ki_lin * integral_lin) + (kd_lin * derivative);
        speedLinear = constrain(speedLinear, -50, 50);
        Serial.printf("Speed Linear: %.2f", speedLinear);

        if (fabs(errorLocalX) > fabs(errorLocalY) && errorLocalX > 0) {
          my_robot.MoveForward(fabs(speedLinear));
        } else if (fabs(errorLocalX) > fabs(errorLocalY) && errorLocalX < 0) {
          my_robot.MoveBackward(fabs(speedLinear));
        } else if (fabs(errorLocalX) < fabs(errorLocalY) && errorLocalY > 0) {
          my_robot.MoveRight(fabs(speedLinear));
        } else if (fabs(errorLocalX) < fabs(errorLocalY) && errorLocalY < 0) {
          my_robot.MoveLeft(fabs(speedLinear));
        }
      } else {
        my_robot.stopSmorphi();
        my_robot.sm_reset_M1();
        my_robot.sm_reset_M2();
        my_robot.sm_reset_M3();
        my_robot.sm_reset_M4();
        integral = 0;
        lastError = 0;
        lastError_lin = 0;
        integral_lin = 0;
        // Removed delay(1000) to avoid blocking UDP
        if (fabs(errorTheta) < 3 && fabs(errorLinear) < 0.01) {
          if (prev_shape != targetPose.shape) {
            switch (targetPose.shape) {
              case 0:
                my_robot.I();
                break;
              case 1:
                my_robot.O();
                break;
              case 2:
                my_robot.L();
                break;
              default:
                break;
            }
          }
          prev_shape = targetPose.shape;
          targetIndex++;
          currentPose.x = 0;
          currentPose.y = 0;
          odometry->setResetPose();
        }
      }
    }
  }


   #ifdef USE_WIFI 
  // Always check for UDP packets for fast response
  int packetSize = udp.parsePacket();
  if (packetSize) {
    int len = udp.read(incomingPacket, 255);
    if (len > 0) incomingPacket[len] = 0;

    remoteIP = udp.remoteIP();
    remotePort = udp.remotePort();

    // Serial.printf("Received: %s\n", incomingPacket);
    handleCommand(String(incomingPacket));
  }
  // Send pose data periodically
  static unsigned long sendDataTime = 0;
  if (millis() - sendDataTime > 200) {  // Increased to 100ms to reduce load
    sendPoseData();
    sendDataTime = millis();
  }

  #endif

}