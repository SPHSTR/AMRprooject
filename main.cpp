#include <Arduino.h>
#include <MPU6050_light.h>
#include <Wire.h>
#include <TCA9548.h>
#include <TaskScheduler.h>
#include <AS5600.h>

// --- Configuration ---

// --- Global Variables ---
PCA9548 MP(0x70);
MPU6050 body_gyro(Wire), platform_gyro(Wire);
AS5600 arm_encoderL, arm_encoderR, wheel_encoderL, wheel_encoderR;

volatile float vL_filtered, vR_filtered;
volatile float bodyX, bodyY, bodyZ, platX, platY, platZ;
volatile float armL_deg, armR_deg;
const float alpha = 0.2;

Scheduler myTimeKeeper1;


// --- Function Prototypes ---
void bodyGyroReadCallback(); 
void platformGyroReadCallback(); 
void ArmEncoderReadCallback(); 
void WheelVelocityReadCallback();

// --- Tasks Definition ---
Task tBodyGyroRead(10, TASK_FOREVER, &bodyGyroReadCallback);
Task tPlatformGyroRead(10, TASK_FOREVER, &platformGyroReadCallback);
Task tArmEncoderRead(20, TASK_FOREVER, &ArmEncoderReadCallback);
Task tWheelVelocityRead(10, TASK_FOREVER, &WheelVelocityReadCallback);

// --- taks callback ---
void bodyGyroReadCallback(){ 
  MP.selectChannel(0); 
  body_gyro.update(); 
  bodyX = body_gyro.getAngleX(); 
  bodyY = body_gyro.getAngleY(); 
  bodyZ = body_gyro.getAngleZ(); 

  Serial.print("Body Gyro X: "); Serial.print(bodyX);
  Serial.print(" | Y: "); Serial.print(bodyY);
  Serial.print(" | Z: "); Serial.println(bodyZ);
}

void platformGyroReadCallback(){ 
  MP.selectChannel(1); 
  platform_gyro.update(); 
  platX = platform_gyro.getAngleX(); 
  platY = platform_gyro.getAngleY(); 
  platZ = platform_gyro.getAngleZ(); 
}

void ArmEncoderReadCallback(){ 
  MP.selectChannel(2); 
  armL_deg = arm_encoderL.readAngle() * AS5600_RAW_TO_DEGREES; 

  MP.selectChannel(3); 
  armR_deg = arm_encoderR.readAngle() * AS5600_RAW_TO_DEGREES; 
}

void WheelVelocityReadCallback(){ 
  MP.selectChannel(4); 
  vL_filtered = (alpha * wheel_encoderL.getAngularSpeed(AS5600_MODE_RPM)) + ((1.0 - alpha) * vL_filtered);

  MP.selectChannel(5); 
  vR_filtered = (alpha * wheel_encoderR.getAngularSpeed(AS5600_MODE_RPM)) + ((1.0 - alpha) * vR_filtered);
}

// --- Standard Arduino Functions ---
void setup() {
  Wire.begin();
  Wire.setClock(100000);
  
  Serial.begin(115200);

  MP.begin();

  MP.selectChannel(0); 
  body_gyro.begin(); 
  body_gyro.calcOffsets(true, true);

  MP.selectChannel(1); 
  platform_gyro.begin(); 
  platform_gyro.calcOffsets(true, true);

  MP.selectChannel(2); 
  arm_encoderL.begin();

  MP.selectChannel(3); 
  arm_encoderR.begin();

  MP.selectChannel(4); 
  wheel_encoderL.begin();

  MP.selectChannel(5); 
  wheel_encoderR.begin();


  // Scheduler init
  myTimeKeeper1.init();
  myTimeKeeper1.addTask(tBodyGyroRead); 
  myTimeKeeper1.addTask(tPlatformGyroRead); 
  myTimeKeeper1.addTask(tArmEncoderRead); 
  myTimeKeeper1.addTask(tWheelVelocityRead);
  tBodyGyroRead.enable(); 
  // tPlatformGyroRead.enable(); 
  // tArmEncoderRead.enable(); 
  // tWheelVelocityRead.enable();

  // tBodyGyroRead.enableDelayed(5000);
  // tPlatformGyroRead.enableDelayed(5000);
}

void loop() {
  myTimeKeeper1.execute();
}

