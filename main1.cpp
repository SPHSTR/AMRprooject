#include <Arduino.h>
#include <math.h>
#include <XboxSeriesXControllerESP32_asukiaaa.hpp>
#include <TaskScheduler.h>
#include <PID_v1.h>

// --- Configuration ---
#define control_select_pin 15

#define WheelmotorLeft_F 19
#define WheelmotorLeft_R 18
#define WheelmotorRight_F 5
#define WheelmotorRight_R 17


XboxSeriesXControllerESP32_asukiaaa::Core xboxController;
// XboxSeriesXControllerESP32_asukiaaa::Core xboxController("28:ea:0b:d0:38:a4");

Scheduler myTimeKeeper1;

// --- Global Variables ---
int controllerLV = 0;
int controllerLH = 0;
int controllertheshold = 10;
bool startVibrationDone = false;

float rampedLV_manual = 0.0;
float rampedLH_manual = 0.0; 
const float ramp_manual_step = 11.0;

float robot_linear_velocity = 0.0;
float robot_angular_velocity = 0.0;

bool controlMode = false;

float r_wheel_velocity;
float l_wheel_velocity;
float r_wheel_velocity_target;
float l_wheel_velocity_target;


// --- Function Prototypes ---
void tUpdateControllerCallback();
void tDriveRobotCallback();
void tCheckConnectionCallback();
void tControlModeCheckCallback();

// --- Tasks Definition ---
// Controller read 
Task tUpdateController(10, TASK_FOREVER, &tUpdateControllerCallback);
// robot driving 
Task tDriveRobot(10, TASK_FOREVER, &tDriveRobotCallback);
// controller connection check
Task tCheckConnection(10000, TASK_FOREVER, &tCheckConnectionCallback);
// mode check
Task tControlModeCheck(500, TASK_FOREVER, &tControlModeCheckCallback);

// --- Motor Logic ---
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

  float left_velocity = linear_velocity - (angular_velocity * wheel_base / 2.0);
  float right_velocity = linear_velocity + (angular_velocity * wheel_base / 2.0);

  Motor_drive(WheelmotorLeft_F, WheelmotorLeft_R, left_velocity);
  Motor_drive(WheelmotorRight_F, WheelmotorRight_R, right_velocity);
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

float wheel_manual_ApplyRamp(float current, float target, float step) {
  if (current < target) {
    current += step;
    if (current > target) current = target; // ป้องกันค่าเลยเป้าหมาย
  } else if (current > target) {
    current -= step;
    if (current < target) current = target; // ป้องกันค่าเลยเป้าหมาย
  }
  return current;
}

// --- Task Callbacks ---

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
      float rawLV = (-1) * scale_controllervalue(xboxController.xboxNotif.joyLVert);
      float rawLH = scale_controllervalue(xboxController.xboxNotif.joyLHori);

      controllerLV = (abs(rawLV) > controllertheshold) ? rawLV : 0;  //+-255
      controllerLH = (abs(rawLH) > controllertheshold) ? rawLH : 0;

    }else if(!xboxController.isConnected()){
      controllerLH = 0;
      controllerLV = 0;
  }
}
}

void tDriveRobotCallback() {
    if(controlMode == false){

      rampedLV_manual = wheel_manual_ApplyRamp(rampedLV_manual, (float)controllerLV, ramp_manual_step);
      rampedLH_manual = wheel_manual_ApplyRamp(rampedLH_manual, (float)controllerLH, ramp_manual_step);

      Robot_move(rampedLV_manual, rampedLH_manual);
    }else if (controlMode == true){
      
      //uros control here  PID   rampfunction
    }
    
}

void tRobotArmCallback() {
  if(controlMode == false){


  }else if (controlMode == true){
  
  }
}

void tplatformCallback() {
    if(controlMode == false){


  }else if (controlMode == true){
  
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

// --- Standard Arduino Functions ---

void setup() {
  Serial.begin(9600);

  pinMode(control_select_pin, INPUT_PULLUP);

  pinMode(WheelmotorLeft_F, OUTPUT);
  pinMode(WheelmotorLeft_R, OUTPUT);
  pinMode(WheelmotorRight_F, OUTPUT);
  pinMode(WheelmotorRight_R, OUTPUT);

  xboxController.begin();
  
  // Scheduler init
  myTimeKeeper1.init();
  myTimeKeeper1.addTask(tUpdateController);
  myTimeKeeper1.addTask(tDriveRobot);
  myTimeKeeper1.addTask(tCheckConnection);
  myTimeKeeper1.addTask(tControlModeCheck);
  
  tUpdateController.enable();
  tDriveRobot.enable();
  tCheckConnection.enable();
  tControlModeCheck.enable();

  tCheckConnection.enableDelayed(10000);
}

void loop() {
  myTimeKeeper1.execute();

}
