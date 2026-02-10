#include <Arduino.h>
#include <math.h>
#include <XboxSeriesXControllerESP32_asukiaaa.hpp>
#include <TaskScheduler.h>
#include <PID_v1.h>

// --- Micro-ROS Includes ---

// Message Type 

// --- Configuration ---
#define control_select_pin 15

#define WheelmotorRight_R 5
#define WheelmotorRight_L 17
#define WheelmotorLeft_R 18
#define WheelmotorLeft_L 19

#define ArmmotorLeft_R 25
#define ArmmotorLeft_L 33
#define ArmmotorRight_R 4
#define ArmmotorRight_L 16 

#define PlatformmotorLeft_R 12
#define PlatformmotorLeft_L 14  
#define PlatformmotorRight_R 26
#define PlatformmotorRight_L 27

// --- Data Structure ---

// --- Global Objects  ---

XboxSeriesXControllerESP32_asukiaaa::Core xboxController;
// XboxSeriesXControllerESP32_asukiaaa::Core xboxController("28:ea:0b:d0:38:a4");

Scheduler runner;

int controllerLV = 0;
int controllerLH = 0;
int controllerBtnA = 0;
int controllerBtnB = 0;
int controllerBtnX = 0;
int controllerBtnY = 0;
int controllertheshold = 12;
bool startVibrationDone = false;

float rampedLV_manual = 0.0;
float rampedLH_manual = 0.0; 

float robot_linear_velocity = 0.0;
float robot_angular_velocity = 0.0;

bool controlMode = false;



// --- micro-ROS object ---


// --- Helper Functions ---
void Motor_drive(int motor_F, int motor_R, float motorspeed) {
  motorspeed = constrain(motorspeed, -255, 255);
  if (motorspeed > 0) {
    analogWrite(motor_F, motorspeed);
    analogWrite(motor_R, 0);
  } else if (motorspeed < 0) {
    analogWrite(motor_F, 0);
    analogWrite(motor_R, -motorspeed);
  } else {
    analogWrite(motor_F, 0);
    analogWrite(motor_R, 0);
  }
}

void Robot_move(float linear_velocity, float angular_velocity) {
  float wheel_base = 0.7;

  linear_velocity *= -1;
  angular_velocity *= -1;

  float left_velocity = linear_velocity - (angular_velocity * wheel_base / 2.0);
  float right_velocity = linear_velocity + (angular_velocity * wheel_base / 2.0);

  Motor_drive(WheelmotorLeft_R, WheelmotorLeft_L, left_velocity);
  Motor_drive(WheelmotorRight_R, WheelmotorRight_L, right_velocity);
}

float scale_controllervalue(float value) {
  return (value - 0) * (255.0 - (-255.0)) / (65535.0 - 0) + (-255.0);
}

void Vibration(int power, int duration_ms) {
  XboxSeriesXHIDReportBuilder_asukiaaa::ReportBase repo;
  repo.v.select.left = repo.v.select.right = repo.v.select.shake = repo.v.select.center = 1;
  repo.v.power.left = repo.v.power.right = repo.v.power.shake = repo.v.power.center = power;
  repo.v.timeActive = duration_ms / 10;
  xboxController.writeHIDReport(repo);
}

float wheel_manual_ApplyRamp(float current, float target) {
  // 1 m/s = 53.55 units
  const float max_accel_threshold = 53.55; 
  const float accel_step_fast = 2.0;
  const float accel_step_slow = 0.8;
  const float decel_step = 10.5;

  // Case 1: Target is Zero
  if (target == 0) {
    if (current > 0) {
      current -= decel_step;
      if (current < 0) current = 0;
    } else if (current < 0) {
      current += decel_step;
      if (current > 0) current = 0;
    }
  } 
  // Case 2: Target is Positive
  else if (target > 0) {
    if (current < target) {
      if (current < max_accel_threshold) current += accel_step_fast;
      else current += accel_step_slow;
      if (current > target) current = target;
    } else {
      // current > target
      current -= decel_step;
      if (current < target) current = target;
    }
  } 
  
  // Case 3: Target is Negative
  else {
    if (current > target) {
      if (current > -max_accel_threshold) current -= accel_step_fast;
      else current -= accel_step_slow;
      if (current < target) current = target;
    } else {
      //current < target
      current += decel_step;
      if (current > target) current = target;
    }
  }

  return current;
}

// --- Sensor Callbacks (Core 1) ---
void tControlModeCheckCallback(){
  if(digitalRead(control_select_pin)==HIGH){ // true = ROS , faase = Controller
    controlMode = true;
  }else{
    controlMode = false;
  }
}

void tUpdateControllerCallback() {
  if(controlMode == false){
    xboxController.onLoop();
    if (xboxController.isConnected() && !xboxController.isWaitingForFirstNotification()) {
      if (!startVibrationDone) {
        Vibration(70, 250);
        startVibrationDone = true;
      }

      // อ่านและ Scale ค่า
      float rawLV = (-1) *scale_controllervalue(xboxController.xboxNotif.joyLVert);
      float rawLH = (-1) * scale_controllervalue(xboxController.xboxNotif.joyLHori);

      controllerLV = (abs(rawLV) > controllertheshold) ? rawLV : 0;  //+-255
      controllerLH = (abs(rawLH) > controllertheshold) ? rawLH : 0;

      // Serial.print(controllerLV); Serial.print(" "); Serial.println(controllerLH);

      controllerBtnA = xboxController.xboxNotif.btnA;
      controllerBtnB = xboxController.xboxNotif.btnB;
      controllerBtnX = xboxController.xboxNotif.btnX;
      controllerBtnY = xboxController.xboxNotif.btnY;

    }else if(!xboxController.isConnected()){
      controllerLH = 0;
      controllerLV = 0;
      controllerBtnA = 0;
      controllerBtnB = 0;
      controllerBtnX = 0;
      controllerBtnY = 0;
  }
}
}

void tDriveRobotCallback() {
    if(controlMode == false){

      rampedLV_manual = wheel_manual_ApplyRamp(rampedLV_manual, (float)controllerLV);
      rampedLH_manual = wheel_manual_ApplyRamp(rampedLH_manual, (float)controllerLH);
  
      Robot_move(rampedLV_manual, rampedLH_manual);
    }else if (controlMode == true){
      
      //uros control here  PID


    }
    
}

void tRobotArmCallback() {
  int motorspeed = 80;
  if(controlMode == false){
    if(controllerBtnA){Motor_drive(ArmmotorLeft_R,ArmmotorLeft_L,motorspeed);Motor_drive(ArmmotorRight_R,ArmmotorRight_L,motorspeed);}else
    if(controllerBtnB){Motor_drive(ArmmotorLeft_R,ArmmotorLeft_L,-motorspeed);Motor_drive(ArmmotorRight_R,ArmmotorRight_L,-motorspeed);}else
    {
      Motor_drive(ArmmotorLeft_R,ArmmotorLeft_L,0);
      Motor_drive(ArmmotorRight_R,ArmmotorRight_L,0);
    }
    Serial.println(controllerBtnA);
  }else if (controlMode == true){
    //uros mode




  }
}

void tplatformCallback() {
  int motorspeed = 60;
  if(controlMode == false){
    if(controllerBtnX){Motor_drive(PlatformmotorLeft_R,PlatformmotorLeft_L,motorspeed);Motor_drive(PlatformmotorRight_R,PlatformmotorRight_L,motorspeed);}
    if(controllerBtnY){Motor_drive(PlatformmotorLeft_R,PlatformmotorLeft_L,-motorspeed);Motor_drive(PlatformmotorRight_R,PlatformmotorRight_L,-motorspeed);}

  }else if (controlMode == true){
    //uros mode



  
  }
}

void tCheckConnectionCallback() {
  if(controlMode == false){
    if (!xboxController.isConnected()) {
      Serial.println("Controller not found. Restarting ESP...");
      ESP.restart();
    }
}
}


// --- Micro-ROS Functions (Core 0) ---




// --- Tasks Definition ---
// Controller read 
Task tUpdateController(10, TASK_FOREVER, &tUpdateControllerCallback);
// robot driving 
Task tDriveRobot(10, TASK_FOREVER, &tDriveRobotCallback);
//robot arm
Task tDriveArm(10, TASK_FOREVER, &tRobotArmCallback);


// controller connection check
Task tCheckConnection(10000, TASK_FOREVER, &tCheckConnectionCallback);
// mode check
Task tControlModeCheck(500, TASK_FOREVER, &tControlModeCheckCallback);



// --- Standard Arduino Functions ---

void setup() {
  Serial.begin(9600);

  pinMode(control_select_pin, INPUT_PULLUP);

  pinMode(WheelmotorLeft_R, OUTPUT);
  pinMode(WheelmotorLeft_L, OUTPUT);
  pinMode(WheelmotorRight_R, OUTPUT);
  pinMode(WheelmotorRight_L, OUTPUT);

  pinMode(ArmmotorLeft_R, OUTPUT);
  pinMode(ArmmotorLeft_L, OUTPUT);
  pinMode(ArmmotorRight_R, OUTPUT);
  pinMode(ArmmotorRight_L, OUTPUT);

  pinMode(PlatformmotorLeft_R, OUTPUT);
  pinMode(PlatformmotorLeft_L, OUTPUT);
  pinMode(PlatformmotorRight_R, OUTPUT);
  pinMode(PlatformmotorRight_L, OUTPUT);

  xboxController.begin();
  
  // Scheduler init
  runner.init();
  runner.addTask(tUpdateController);
  runner.addTask(tDriveRobot);
  runner.addTask(tDriveArm);

  runner.addTask(tCheckConnection);
  runner.addTask(tControlModeCheck);
  

  tUpdateController.enable();
  tDriveRobot.enable();
  tDriveArm.enable();

  tCheckConnection.enable();
  tControlModeCheck.enable();

  tCheckConnection.enableDelayed(10000);
}

void loop() {
  runner.execute();
}
