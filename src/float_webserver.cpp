// =================================================================
// LIBRARIES
// =================================================================
#include <WiFi.h>
#include <Wire.h>
#include <ESPAsyncWebServer.h> //处理不同的网络请求(对应按下网页按钮发起请求, 产生不同反应) webserver lib
#include <PID_v1.h> 
#include <vector> 
#include <MS5837.h> // depth sensor
#include <ArduinoJson.h> // 把记录数据转成json格式的lib 
#include <Arduino.h> // 因为有一个额外的cpp文件(本文件), 需要这个lib让arduino可以链接所有文件
#include <AccelStepper.h> // 控制电机更smooth运动的lib stepper lib
#include "float_webserver.h" // function header file
#include "web_content.h" // 定义了网页内容(网页长啥样, 有什么按键, 按下会发起什么请求)web content
#include <string.h>`
//white d4 (SDA) green d5(SCL)
//ultra white d6 yellow d7


// =================================================================
// SENSORS SET UP
// =================================================================
// I2C(depth sensor) pin and UART(ultra sonic sensor) pins
#define SDA_PIN D4
#define SCL_PIN D5
#define ULTRA_RX D6
#define ULTRA_TX D7

// Pressure and depth Sensor object
MS5837 sensor;
float temp = 0; //temperature
float pressure = 0; //pressure
float depth_filtered = 0;
const float alpha = 0.1;  // 越小越稳但越慢
float raw = 0; //raw_depth 
const float DEPTH_DEADBAND = 0.1; // 10 cm
float depth_offset = 0;

// ULTRASONIC SENSOR

unsigned char us_data[4] = {0};

float ultrasonic_distance_mm = -1;
float ultrasonic_distance_cm = -1;

unsigned long last_ultrasonic_read_ms = 0;
const int ULTRASONIC_INTERVAL_MS = 50;


// =================================================================
// HARDWARE & NETWORK CONFIGURATION
// =================================================================
// Stepper Motor Pins
#define DIR_PIN D2
#define STEP_PIN D3
AccelStepper stepper(1, STEP_PIN, DIR_PIN); //(1表示一号电机, 1 for the first motor(the only motor here))
// Stepper Motor Configuration
const int MIN_FLOAT_STEPS = 0;                     // lowest (min buoyancy)
const int MAX_FLOAT_STEPS = 17000;     // highest (max buoyance)
volatile int max_speed = 800; 
volatile int accel = 500;
volatile float target_step;
volatile int current_step;
volatile float motor_speed = 0;
bool manual_override = false; //control mannually or automatically by PID

// =================================================================
// PID & MOTOR PARAMETERS
// =================================================================
// PID Variables
double setpoint_depth, current_depth, pid_output;

// PID Tuning Parameters 
double Kp = 50, Ki = 5, Kd = 100;
PID myPID(&current_depth, &pid_output, &setpoint_depth, Kp, Ki, Kd, REVERSE); 
float fluid_density = 1015; // Default to freshwater (kg/m^3)



// esp32 的cpu有两个核心, 可以让一个核心专门产生控制电机的高频pwm信号, 电机运动就更加丝滑, 但是要创建一个任务句柄, 来让第二个cpu核心可以通过这个句柄执行任务
// 任务句柄是一个指向任务控制块(相当于一个class的对象吧)的指针
// 定义任务句柄
TaskHandle_t StepperTaskHandle = NULL; 


// Network Credentials
const char* ssid = "WUROVFloat";
const char* password = "wentworth";

// Web Server port service initiate 在80号端口开启网页服务. 因为访问网址时, 浏览器默认访问服务器的80号端口, 所以这里要把服务器的网页服务绑定到80号
AsyncWebServer server(80);


// =================================================================
// CONTROL LOGIC & STATE MACHINE
// =================================================================

// 工作状态列表 working state list
enum MainState { 
  STATE_IDLE,
  STATE_MISSION_RUNNING,
  STATE_PID_TEST,
  STATE_MANUAL_TOP,
  STATE_MANUAL_BOTTOM,
  STATE_MANUAL_DESCENDING,
  STATE_RESET
};

// 初始化工作状态 initial
MainState main_state = STATE_IDLE;

// 二级工作状态列表(自动工作时的流程) substate list (automation working state list)
enum MissionSubState {
  SUBSTATE_DESCENDING,      //first descending (2.5m in depth)
  SUBSTATE_HOLDING,         //first holding    (45sec)
  SUBSTATE_ASCENDING,       //first ascending  (0.4m in depth)
  SUBSTATE_DESCENDING2,     //second descending(2.5m in depth)
  SUBSTATE_HOLDING2,        //second holding   (45sec)
  SUBSTATE_ASCENDING2,       //final ascending   (0m)
  SUBSTATE_DESCENDING_TEST, // PID test descending (to target_depth)
  SUBSTATE_HOLDING_TEST,    // PID test holding (10 sec)
};

// 初始化二级工作状态
MissionSubState mission_substate = SUBSTATE_DESCENDING;


// =================================================================
// MISSION PARAMETERS
// =================================================================
const float MISSION_DEPTH_GOAL_M = 2.5;
const float MISSION_DEPTH_TOLERANCE_M = 0.1;
const float MISSION_DEPTH_ASCENDING = 0.4;
const unsigned long MISSION_HOLD_DURATION_MS = 45 * 1000; // 45 seconds
unsigned long mission_hold_start_ms = 0;

// =================================================================
// DATA LOGGING & TIMING
// =================================================================
const int SENSOR_READ_INTERVAL_MS = 100;
const int DATA_LOG_INTERVAL_MS = 1000;
unsigned long last_sensor_read_ms = 0;
unsigned long last_data_log_ms = 0;
unsigned long last_motor_step_ms = 0;

std::vector<unsigned long> history_time_ms;
std::vector<float> history_depth;
std::vector<float> history_temp;
std::vector<float> history_pressure;


// =================================================================
// SETUP FUNCTION
// =================================================================
void setup_float() {
//  Serial.begin(115200);  // uncomment if needed for debugging on PC

  // Setup Wi-Fi as an Access Point
  WiFi.softAP(ssid, password);
  Serial.print("Access Point IP: ");
  Serial.println(WiFi.softAPIP());

  // Initialize Pressure Sensor
  Wire.begin(SDA_PIN, SCL_PIN); // depth sensor pins
  Serial1.begin(9600, SERIAL_8N1, ULTRA_RX, ULTRA_TX); // ultrasonic sensor pins
  sensor.init(); //depth sensor initial
  sensor.setModel(MS5837::MS5837_30BA);
  sensor.setFluidDensity(fluid_density); // Use the variable here; // Freshwater
  sensor.read();
  depth_offset = sensor.depth();

  // Initialize PID
  myPID.SetOutputLimits(-800, 800);
  myPID.SetMode(MANUAL);
  myPID.SetSampleTime(100);

  // Initialize Motor Position
  stepper.setMaxSpeed(max_speed);      // 最大速度 set max speed
  stepper.setAcceleration(accel);   // 加速度 set acceleration
  stepper.setCurrentPosition(MAX_FLOAT_STEPS);    // set current position as the highest position (syringe at the lowest point: all water pumps out)
  current_step = stepper.currentPosition();
  target_step = current_step;

  //绑定发送电机pwm信号任务到第一cpu核心
  xTaskCreatePinnedToCore(
        stepperTask,        /* 任务函数名, 函数名称实际是函数指针, 指向stepperTask 函数 */
        "StepperTask",      /* 任务名称, 自由定义 */
        4096,               /* 栈大小 (Stack size) */
        NULL,               /* 任务参数 */
        6,                 /* 优先级 */
        &StepperTaskHandle, /* 任务句柄 */
        0                   /* 固定在核心 0 核心0 是第一个核心, 其他loop里的任务都跑在第二核心 core 1 */ 
    );
  


  // ========== WEB SERVER ENDPOINTS ==========
  // Serve the main web page 主网页, 发送web_content里定义的网页内容
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/html", index_html,processor);
  });

  // API: Start the mission 收到start请求后反应
  server.on("/start", HTTP_GET, [](AsyncWebServerRequest *request){
    if (main_state == STATE_IDLE) {
      Serial.println("Command received: START MISSION");
      history_time_ms.clear();
      history_depth.clear();
      history_temp.clear();
      history_pressure.clear();
      main_state = STATE_MISSION_RUNNING;
      mission_substate = SUBSTATE_DESCENDING;
      request->send(200, "text/plain", "Mission started.");
    } else {
      request->send(400, "text/plain", "Already running.");
    }
  });

  // API: Stop all operations and go to idle 收到stop请求
  server.on("/stop", HTTP_GET, [](AsyncWebServerRequest *request){
    Serial.println("Command received: STOP");
    main_state = STATE_IDLE;
    target_step = current_step; // Hold current position
    request->send(200, "text/plain", "All operations stopped.");
  });

  // API: Go to top (max buoyancy)
  server.on("/top", HTTP_GET, [](AsyncWebServerRequest *request){
    Serial.println("Command received: GO TO TOP");
    main_state = STATE_MANUAL_TOP;
    request->send(200, "text/plain", "Going to top.");
  });

  // API: Go to bottom (min buoyancy)
  server.on("/bottom", HTTP_GET, [](AsyncWebServerRequest *request){
    Serial.println("Command received: GO TO BOTTOM");
    main_state = STATE_MANUAL_BOTTOM;
    request->send(200, "text/plain", "Going to bottom.");
  });

  // API: descend for adjusting motor position
  server.on("/descend", HTTP_GET, [](AsyncWebServerRequest *request){
    Serial.println("Command received: descending");
    main_state = STATE_MANUAL_DESCENDING;
    request->send(200, "text/plain", "DESCENDING.");
  });

  // API: Reset motor 0 position
  server.on("/reset", HTTP_GET, [](AsyncWebServerRequest *request){
    Serial.println("Command received: reset");
    stepper.setCurrentPosition(0);
    target_step = 0;
    main_state = STATE_RESET;
    request->send(200, "text/plain", "reset.");
  });

  //PID test
  server.on("/set_depth", HTTP_GET, [](AsyncWebServerRequest *request){
    if(request->hasParam("depth"))
    {
        setpoint_depth = request->getParam("depth")->value().toFloat();
        main_state = STATE_PID_TEST;
        request->send(200, "text/plain", "Target depth updated.");
    }
    else
    {
        request->send(400, "text/plain", "Missing depth parameter.");
    }

});
  
  // API: Get real-time data
  server.on("/data", HTTP_GET, [](AsyncWebServerRequest *request){
    String stateStr;
    switch(main_state){
      case STATE_IDLE: stateStr = "Idle"; break;
      case STATE_MISSION_RUNNING: 
        switch(mission_substate){
          case SUBSTATE_DESCENDING: stateStr = "Mission: Descending"; break;
          case SUBSTATE_HOLDING: stateStr = "Mission: Holding"; break;
          case SUBSTATE_ASCENDING: stateStr = "Mission: Ascending"; break;
          case SUBSTATE_DESCENDING2: stateStr = "Mission: Descending"; break;
          case SUBSTATE_HOLDING2: stateStr = "Mission: Holding"; break;
          case SUBSTATE_ASCENDING2: stateStr = "Mission: Ascending"; break;
          case SUBSTATE_DESCENDING_TEST: stateStr = "Mission:Descending Test"; break;
          case SUBSTATE_HOLDING_TEST: stateStr = "Mission: Holding Test"; break;
        }
        break;
      case STATE_MANUAL_TOP: stateStr = "Manual: To Top"; break;
      case STATE_MANUAL_BOTTOM: stateStr = "Manual: To Bottom"; break;
      case STATE_MANUAL_DESCENDING: stateStr = "Manual: Descending 2000"; break;
      case STATE_PID_TEST: stateStr = "PID Descending Task Testing"; break;
    }
    
    String json = "{";
    json += "\"state\":\"" + stateStr + "\",";
    json += "\"depth\":" + String(current_depth) + ",";
    json += "\"temp\":" + String(temp) + ",";
    json += "\"pressure\":" + String(pressure) + ",";
    json += "\"ultrasonic\":" +String(ultrasonic_distance_cm) +",";
    json += "\"setpoint\":" + String(setpoint_depth) + ",";
    json += "\"motor_pos\":" + String(current_step) + ",";
    json += "\"motor_max\":" + String(MAX_FLOAT_STEPS) + ",";
    json += "\"kp\":" + String(Kp) + ",";
    json += "\"ki\":" + String(Ki) + ",";
    json += "\"kd\":" + String(Kd) + ",";
    json += "\"target_step\":" + String(target_step) + ",";
    json += "\"density\":" + String(fluid_density)+ ",";
    json += "\"max_speed\":" + String(max_speed)+ ",";
    json += "\"accel\":" + String(accel);
    json += "}";
    request->send(200, "application/json", json);
  });

  // Endpoint to receive and apply new tuning parameters
  server.on("/set_tuning", HTTP_GET, [](AsyncWebServerRequest *request){
    bool success = true;
    if (request->hasParam("kp"))        Kp = request->getParam("kp")->value().toFloat();
    if (request->hasParam("ki"))        Ki = request->getParam("ki")->value().toFloat();
    if (request->hasParam("kd"))        Kd = request->getParam("kd")->value().toFloat();
    if (request->hasParam("density"))   fluid_density = request->getParam("density")->value().toFloat();
    if (request->hasParam("max_speed")) max_speed = request->getParam("max_speed")->value().toFloat();
    if (request->hasParam("accel"))     accel = request->getParam("accel")->value().toFloat();


    // Apply the new settings immediately
    myPID.SetTunings(Kp, Ki, Kd);
    stepper.setMaxSpeed(max_speed);      // 最大速度
    stepper.setAcceleration(accel);   // 加速度


    Serial.println("Tuning parameters updated:");
    Serial.printf("  Kp: %.2f, Ki: %.2f, Kd: %.2f\n", Kp, Ki, Kd);
    Serial.printf("  Density: %.1f\n", fluid_density);

    request->send(200, "text/plain", "Tuning updated successfully.");
  });
  
  // API: Get historical mission data
  server.on("/history_csv", HTTP_GET, [](AsyncWebServerRequest *request){
    // 1. 创建一个 String 来构建 CSV 内容
    String csv = "Time (ms),Depth (m), Pressure (Bar), temp (C)"; // CSV header
    
    // 2. 遍历历史数据并添加到 String 中
    for(size_t i=0; i < history_depth.size(); i++){
      csv += String(history_time_ms[i]);
      csv += ",";
      csv += String(history_depth[i]);
      csv += ",";
      csv += String(history_temp[i]);
      csv += ",";
      csv += String(history_pressure[i]);
      csv += "\n";
    }
    
    // 3. 发送 CSV 数据。
    // "text/csv" 告诉浏览器这是一个CSV文件。
    // `download="mission_data.csv"` 在HTML的<a>标签中已经处理了文件名。
    request->send(200, "text/csv", csv);
  });

  server.begin();
  Serial.println("Web server started.");
}

String processor(const String& var){
  if(var == "KP") return String(Kp);
  if(var == "KI") return String(Ki);
  if(var == "KD") return String(Kd);
  if(var == "DS") return String(fluid_density);
  if(var == "MS") return String(max_speed);
  if(var == "AC") return String(accel);
  return String();
}


// =================================================================
// MAIN LOOP - NON-BLOCKING
// =================================================================
void loop_float() {
  handleSensor();
  handleUltrasonic();
  handleStateMachine();
  handlePID();  
  handleDataLogging();
}

// =================================================================
// HANDLER FUNCTIONS 
// =================================================================

// depth sensor reading
void handleSensor() {
  if (millis() - last_sensor_read_ms > SENSOR_READ_INTERVAL_MS) {
    last_sensor_read_ms = millis();
    sensor.read();
    raw = sensor.depth()-depth_offset;
    depth_filtered = alpha * raw + (1 - alpha) * depth_filtered;
    current_depth = depth_filtered;
    temp = sensor.temperature();
    pressure = sensor.pressure();
  }
}

void handleUltrasonic()
{
    while (Serial1.available() >= 4)
    {
        if (Serial1.peek() == 0xFF)
        {
            for (int i = 0; i < 4; i++)
            {
                us_data[i] = Serial1.read();
            }

            uint8_t checksum =
                (us_data[0] +
                 us_data[1] +
                 us_data[2]) & 0xFF;

            if (checksum == us_data[3])
            {
                ultrasonic_distance_mm =
                    (us_data[1] << 8) |
                    us_data[2];

                if (ultrasonic_distance_mm >= 280)
                {
                    ultrasonic_distance_cm =
                        ultrasonic_distance_mm / 10.0;
                }
                else
                {
                    ultrasonic_distance_cm = -1;
                }
            }
        }
        else
        {
            Serial1.read();   // 丢弃错误字节
        }
    }
}

// 主任务状态控制
void handleStateMachine() {
  switch (main_state) {
    case STATE_IDLE:
      // Do nothing, PID is off, motor target is current position
      manual_override = true;
      myPID.SetMode(MANUAL);
      break;

    case STATE_MISSION_RUNNING:
      manual_override = false;
      myPID.SetMode(AUTOMATIC);
      handleMissionSubstates();
      break;

    case STATE_PID_TEST:
      manual_override = false;
      myPID.SetMode(AUTOMATIC);
      mission_substate = SUBSTATE_DESCENDING_TEST;
      handleMissionSubstates();
      break;

    case STATE_MANUAL_TOP:
      manual_override = true;
      myPID.SetMode(MANUAL);
      target_step = MAX_FLOAT_STEPS;
      main_state = STATE_IDLE; // Reached target, go idle
      break;

    case STATE_MANUAL_DESCENDING:
      manual_override = true;
      myPID.SetMode(MANUAL);
      target_step = current_step-2000;
      main_state = STATE_IDLE; // Reached target, go idle
      break;
    
    case STATE_RESET:
      manual_override = true;
      myPID.SetMode(MANUAL);
      current_step = 0;
      target_step = 0;
      main_state = STATE_IDLE; // Reached target, go idle
      break;

    case STATE_MANUAL_BOTTOM:
      manual_override = true;
      myPID.SetMode(MANUAL);
      target_step = MIN_FLOAT_STEPS;
      if (current_step == target_step) {
        main_state = STATE_IDLE; // Reached target, go idle
      }
      break;
  }
}

// 自动任务控制
void handleMissionSubstates() {
  switch (mission_substate) {
    case SUBSTATE_DESCENDING:
      setpoint_depth = MISSION_DEPTH_GOAL_M;
      if (abs(setpoint_depth - current_depth) < MISSION_DEPTH_TOLERANCE_M) {
        Serial.println("Reached descent goal. Starting hold.");
        mission_substate = SUBSTATE_HOLDING;
        mission_hold_start_ms = millis();
      }
      break;

    case SUBSTATE_HOLDING:
      setpoint_depth = MISSION_DEPTH_GOAL_M;
      if (millis() - mission_hold_start_ms >= MISSION_HOLD_DURATION_MS) {
        Serial.println("Hold time finished. Starting ascent.");
        mission_substate = SUBSTATE_ASCENDING;
      }
      break;

    case SUBSTATE_ASCENDING:
      setpoint_depth = MISSION_DEPTH_ASCENDING;
      if (current_depth <= MISSION_DEPTH_ASCENDING+0.2) {
        Serial.println("Reached 0.4 m depth.");
        mission_substate = SUBSTATE_DESCENDING2;
      }
      break;

    case SUBSTATE_DESCENDING2:
      setpoint_depth = MISSION_DEPTH_GOAL_M;
      if (abs(setpoint_depth - current_depth) < MISSION_DEPTH_TOLERANCE_M) {
        mission_substate = SUBSTATE_HOLDING2;
        mission_hold_start_ms = millis();
      }
      break;

    case SUBSTATE_DESCENDING_TEST:
      if (abs(setpoint_depth - current_depth) < MISSION_DEPTH_TOLERANCE_M) {
        mission_substate = SUBSTATE_HOLDING_TEST;
        mission_hold_start_ms = millis();
      }
      break;
    
    case SUBSTATE_HOLDING_TEST:
      if (millis() - mission_hold_start_ms >= 10000) {
        mission_substate = SUBSTATE_ASCENDING2;
      }
      break;

    case SUBSTATE_HOLDING2:
      setpoint_depth = MISSION_DEPTH_GOAL_M;
      if (millis() - mission_hold_start_ms >= MISSION_HOLD_DURATION_MS) {
        mission_substate = SUBSTATE_ASCENDING2;
      }
      break;

    case SUBSTATE_ASCENDING2:
      setpoint_depth = 0;
      if (current_depth > -0.1) {
        main_state = STATE_IDLE; // Reached target, go idle
      }
      break;
  }
}

// 每次loop compute PID output
void handlePID() {
  if (main_state == STATE_MISSION_RUNNING) {
    if (abs(setpoint_depth - current_depth) < DEPTH_DEADBAND) {
      motor_speed = 0;
      return;
    } 
    else {
      if (myPID.Compute()) {
        motor_speed = pid_output;
      }
    }
  }
  else 
  {
    motor_speed = 0;
  }
}

// stepper function control
void stepperTask(void *pvParameters)
{
    static unsigned long last_time = millis();

    while (true)
    {
        // dt, delta time
        unsigned long now = millis();
        float dt = (now - last_time) / 1000.0 ;// milli second to second
        last_time = now;

        // 1. 用速度积分更新位置 update position with integration of speed to time
        if (!manual_override)
        {
          target_step += motor_speed * dt;
        }

        // 2. 限制范围（防止跑飞） limit range step
        target_step = constrain(target_step,
                             MIN_FLOAT_STEPS,
                             MAX_FLOAT_STEPS);

        // 3. 写入目标位置 move to target step
        stepper.moveTo((int)target_step);

        // 4. 执行电机 execute stepper
        stepper.run();

        // 5. 当前反馈 get current step
        current_step = stepper.currentPosition();

        // 6. 让出 CPU delay time for cpu core
        vTaskDelay(1);
    }
}

void handleDataLogging(){
  if(main_state != STATE_IDLE && millis() - last_data_log_ms > DATA_LOG_INTERVAL_MS){
    last_data_log_ms = millis();
    history_time_ms.push_back(millis());
    history_depth.push_back(current_depth);
    history_temp.push_back(temp);
    history_pressure.push_back(pressure);
  }
}