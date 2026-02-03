#include <Arduino.h>
#include <MPU6050_light.h>
#include <Wire.h>
#include <TaskScheduler.h>
#include <AS5600.h>

// --- Micro-ROS Includes ---
#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

// Message Type 
#include <std_msgs/msg/float32_multi_array.h>
#include <geometry_msgs/msg/vector3.h>

// --- Configuration ---
#define TCA_ADDR 0x70
#define I2C_TIMEOUT_MS 20
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

// --- Data Structure ---
struct RobotState {
  float body_euler[3];    
  float platform_euler[3];
  float arm_angle[2];     
  float wheel_velocity[2];
};

RobotState robotData;

// --- Global Objects (Sensors) ---
MPU6050 body_gyro(Wire), platform_gyro(Wire);
AS5600 arm_encoderL, arm_encoderR, wheel_encoderL, wheel_encoderR;
const float alpha = 0.2; 
Scheduler runner;

// --- Global Objects (micro-ROS) ---
// Publisher
rcl_publisher_t body_pub;
rcl_publisher_t plat_pub;
rcl_publisher_t arm_pub;
rcl_publisher_t wheel_pub;

// Message 
geometry_msgs__msg__Vector3 body_msg;
geometry_msgs__msg__Vector3 plat_msg;
std_msgs__msg__Float32MultiArray arm_msg;
std_msgs__msg__Float32MultiArray wheel_msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

TaskHandle_t rosTaskHandle = NULL;

// --- Helper Functions ---
bool tcaSelect(uint8_t i) {
  if (i > 7) return false;
  Wire.beginTransmission(TCA_ADDR);
  Wire.write(1 << i);
  byte status = Wire.endTransmission(); 
  return (status == 0); 
}

void error_loop(){
  while(1){ delay(100); }
}

// --- Sensor Callbacks (Core 1) ---
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
  if(tcaSelect(2)) robotData.arm_angle[0] = arm_encoderL.readAngle() * AS5600_RAW_TO_DEGREES; 
  if(tcaSelect(3)) robotData.arm_angle[1] = arm_encoderR.readAngle() * AS5600_RAW_TO_DEGREES; 
}

void WheelVelocityReadCallback() { 
  if(tcaSelect(4)) {
    float raw_vl = wheel_encoderL.getAngularSpeed(AS5600_MODE_RPM);
    robotData.wheel_velocity[0] = (alpha * raw_vl) + ((1.0 - alpha) * robotData.wheel_velocity[0]);
  }
  if(tcaSelect(5)) {
    float raw_vr = wheel_encoderR.getAngularSpeed(AS5600_MODE_RPM);
    robotData.wheel_velocity[1] = (alpha * raw_vr) + ((1.0 - alpha) * robotData.wheel_velocity[1]);
  }
}

// --- Micro-ROS Functions (Core 0) ---

void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    
    // 1. Publish Body Gyro (Vector3)
    body_msg.x = robotData.body_euler[0];
    body_msg.y = robotData.body_euler[1];
    body_msg.z = robotData.body_euler[2];
    RCSOFTCHECK(rcl_publish(&body_pub, &body_msg, NULL));

    // 2. Publish Platform Gyro (Vector3)
    plat_msg.x = robotData.platform_euler[0];
    plat_msg.y = robotData.platform_euler[1];
    plat_msg.z = robotData.platform_euler[2];
    RCSOFTCHECK(rcl_publish(&plat_pub, &plat_msg, NULL));

    // 3. Publish Arm Angles (Array)
    arm_msg.data.data[0] = robotData.arm_angle[0];
    arm_msg.data.data[1] = robotData.arm_angle[1];
    arm_msg.data.size = 2;
    RCSOFTCHECK(rcl_publish(&arm_pub, &arm_msg, NULL));

    // 4. Publish Wheel Velocity (Array)
    wheel_msg.data.data[0] = robotData.wheel_velocity[0];
    wheel_msg.data.data[1] = robotData.wheel_velocity[1];
    wheel_msg.data.size = 2;
    RCSOFTCHECK(rcl_publish(&wheel_pub, &wheel_msg, NULL));
  }
}

void microRosTask(void * parameter) {
  set_microros_transports();
  
  Serial.println("Waiting for agent...");
  while (RMW_RET_OK != rmw_uros_ping_agent(100, 1)) {
      delay(500); 
  }

  Serial.println("Agent connected!");

  allocator = rcl_get_default_allocator();

  // Init Node
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "esp32_sensor_node", "", &support));

  // --- Init 4 Publishers ---
  
  // Pub 1: Body Gyro
  RCCHECK(rclc_publisher_init_default(
    &body_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3),
    "sensors/body_imu"));

  // Pub 2: Platform Gyro
  RCCHECK(rclc_publisher_init_default(
    &plat_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3),
    "sensors/platform_imu"));

  // Pub 3: Arm
  RCCHECK(rclc_publisher_init_default(
    &arm_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
    "sensors/arm_joints"));

  // Pub 4: Wheels
  RCCHECK(rclc_publisher_init_default(
    &wheel_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
    "sensors/wheel_vel"));

  // --- Allocate Memory for Arrays ---
  arm_msg.data.capacity = 2;
  arm_msg.data.data = (float*) malloc(arm_msg.data.capacity * sizeof(float));
  arm_msg.data.size = 0;
  arm_msg.layout.dim.capacity = 0;
  arm_msg.layout.dim.size = 0;
  arm_msg.layout.dim.data = NULL;
  arm_msg.layout.data_offset = 0;

  wheel_msg.data.capacity = 2;
  wheel_msg.data.data = (float*) malloc(wheel_msg.data.capacity * sizeof(float));
  wheel_msg.data.size = 0;
  wheel_msg.layout.dim.capacity = 0;
  wheel_msg.layout.dim.size = 0;
  wheel_msg.layout.dim.data = NULL;
  wheel_msg.layout.data_offset = 0;
  
  // Init Timer
  RCCHECK(rclc_timer_init_default2(&timer, &support, RCL_MS_TO_NS(100), timer_callback, true));

  // Init Executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));

  while(1) {
    RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
    delay(10);
  }
}

// --- Scheduler Tasks ---
Task tBodyGyroRead(20, TASK_FOREVER, &bodyGyroReadCallback);
Task tPlatformGyroRead(20, TASK_FOREVER, &platformGyroReadCallback);
Task tArmEncoderRead(50, TASK_FOREVER, &ArmEncoderReadCallback);
Task tWheelVelocityRead(20, TASK_FOREVER, &WheelVelocityReadCallback);

void setup() {
  Wire.begin();
  Wire.setClock(100000); 
  Wire.setTimeOut(I2C_TIMEOUT_MS); 

  if(tcaSelect(0)) { delay(10); body_gyro.begin(); body_gyro.calcOffsets(true, true); }

  // if(tcaSelect(1)) { delay(10); platform_gyro.begin(); platform_gyro.calcOffsets(true, true);}

  // if(tcaSelect(2)) arm_encoderL.begin();
  // if(tcaSelect(3)) arm_encoderR.begin();

  // if(tcaSelect(4)) wheel_encoderL.begin();
  // if(tcaSelect(5)) wheel_encoderR.begin();
  
  runner.init();
  runner.addTask(tBodyGyroRead); 
  runner.addTask(tPlatformGyroRead); 
  runner.addTask(tArmEncoderRead); 
  runner.addTask(tWheelVelocityRead);
  
  tBodyGyroRead.enable(); 
  // tPlatformGyroRead.enable(); 
  // tArmEncoderRead.enable(); 
  // tWheelVelocityRead.enable();

  xTaskCreatePinnedToCore(microRosTask, "microRosTask", 10000, NULL, 1, &rosTaskHandle, 0);         
}

void loop() {
  runner.execute();
}
