#include <Arduino.h>
#include <MPU6050_light.h>
#include <Wire.h>
#include <TaskScheduler.h>
#include <AS5600.h>

// --- Configuration ---
#define TCA_ADDR 0x70
#define I2C_TIMEOUT_MS 20

// --- Data Structure for micro-ROS ---
struct RobotState {
  float body_euler[3];    // [x, y, z]
  float platform_euler[3]; // [x, y, z]
  float arm_angle[2];     // [Left, Right]
  float wheel_velocity[2]; // [Left, Right] (RPM)
};

RobotState robotData;

// --- Global Objects ---
MPU6050 body_gyro(Wire), platform_gyro(Wire);
AS5600 arm_encoderL, arm_encoderR, wheel_encoderL, wheel_encoderR;

const float alpha = 0.2; 
Scheduler runner;

// --- Helper Functions ---

bool tcaSelect(uint8_t i) {
  if (i > 7) return false;
  Wire.beginTransmission(TCA_ADDR);
  Wire.write(1 << i);
  // endTransmission return 0 Success
  byte status = Wire.endTransmission(); 
  return (status == 0); 
}

void statePrint(){  
  Serial.print("BODY[");
  Serial.printf("%.1f, %.1f, %.1f", robotData.body_euler[0], robotData.body_euler[1], robotData.body_euler[2]);
  Serial.print("]  ");

  Serial.print("PLAT[");
  Serial.printf("%.1f, %.1f, %.1f", robotData.platform_euler[0], robotData.platform_euler[1], robotData.platform_euler[2]);
  Serial.print("]  ");

  Serial.print("ARM[");
  Serial.printf("L:%.1f, R:%.1f", robotData.arm_angle[0], robotData.arm_angle[1]);
  Serial.print("]  ");

  Serial.print("WHEEL[");
  Serial.printf("L:%.1f, R:%.1f", robotData.wheel_velocity[0], robotData.wheel_velocity[1]);
  Serial.println("]");
}

// --- Task Callbacks ---

void bodyGyroReadCallback() { 
  if(tcaSelect(0)) { 
    body_gyro.update(); 
    robotData.body_euler[0] = body_gyro.getAngleX(); 
    robotData.body_euler[1] = body_gyro.getAngleY(); 
    robotData.body_euler[2] = body_gyro.getAngleZ();     
  }
}

void platformGyroReadCallback() { 
  if(tcaSelect(1)) { 
    platform_gyro.update(); 
    robotData.platform_euler[0] = platform_gyro.getAngleX(); 
    robotData.platform_euler[1] = platform_gyro.getAngleY(); 
    robotData.platform_euler[2] = platform_gyro.getAngleZ(); 
  }
}

void ArmEncoderReadCallback() { 
  // Left Arm
  if(tcaSelect(2)) {
    robotData.arm_angle[0] = arm_encoderL.readAngle() * AS5600_RAW_TO_DEGREES; 
  }

  // Right Arm
  if(tcaSelect(3)) {
    robotData.arm_angle[1] = arm_encoderR.readAngle() * AS5600_RAW_TO_DEGREES; 
  }
}

void WheelVelocityReadCallback() { 
  // Left Wheel
  if(tcaSelect(4)) {
    float raw_vl = wheel_encoderL.getAngularSpeed(AS5600_MODE_RPM);
    robotData.wheel_velocity[0] = (alpha * raw_vl) + ((1.0 - alpha) * robotData.wheel_velocity[0]);
  }

  // Right Wheel
  if(tcaSelect(5)) {
    float raw_vr = wheel_encoderR.getAngularSpeed(AS5600_MODE_RPM);
    robotData.wheel_velocity[1] = (alpha * raw_vr) + ((1.0 - alpha) * robotData.wheel_velocity[1]);
  }
}

// --- Tasks Definition ---
Task tBodyGyroRead(20, TASK_FOREVER, &bodyGyroReadCallback);
Task tPlatformGyroRead(20, TASK_FOREVER, &platformGyroReadCallback);
Task tArmEncoderRead(50, TASK_FOREVER, &ArmEncoderReadCallback);
Task tWheelVelocityRead(20, TASK_FOREVER, &WheelVelocityReadCallback);
Task tStatePrint(100, TASK_FOREVER, &statePrint);

// --- Setup ---
void setup() {
  Serial.begin(115200);
  
  Wire.begin();

  Wire.setClock(100000); 
  Wire.setTimeOut(I2C_TIMEOUT_MS); 

  Serial.println("Initializing Sensors...");

  // Init Sensors 
  if(tcaSelect(0)) {
    delay(10);
    body_gyro.begin(); 
    body_gyro.calcOffsets(true, true);
  }

  if(tcaSelect(1)) {
    delay(10);
    platform_gyro.begin(); 
    platform_gyro.calcOffsets(true, true);
  }

  // Init Encoders
  if(tcaSelect(2)) arm_encoderL.begin();
  if(tcaSelect(3)) arm_encoderR.begin();
  if(tcaSelect(4)) wheel_encoderL.begin();
  if(tcaSelect(5)) wheel_encoderR.begin();

  runner.init();
  runner.addTask(tBodyGyroRead); 
  runner.addTask(tPlatformGyroRead); 
  runner.addTask(tArmEncoderRead); 
  runner.addTask(tWheelVelocityRead);
  runner.addTask(tStatePrint);
  
  tBodyGyroRead.enable(); 
  tPlatformGyroRead.enable(); 
  tArmEncoderRead.enable(); 
  tWheelVelocityRead.enable();
  tStatePrint.enable();
  
  Serial.println("System Ready.");
}

void loop() {
  runner.execute();
}
