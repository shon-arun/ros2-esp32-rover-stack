#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/int32_multi_array.h> 
#include <std_msgs/msg/int32.h> 
#include <sensor_msgs/msg/imu.h> 
#include <sensor_msgs/msg/temperature.h>
#include <rmw_microros/rmw_microros.h>
#include <FastLED.h> 
#include <rcl/error_handling.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// --- Custom I2C Pins for MPU6500 ---
#define SDA_PIN 2
#define SCL_PIN 1
Adafruit_MPU6050 mpu;

// --- IMU Calibration Offsets ---
float gyro_x_offset = 0.0;
float gyro_y_offset = 0.0;
float gyro_z_offset = 0.0;
float accel_x_offset = 0.0;
float accel_y_offset = 0.0;

// --- RGB LED Settings ---
#define LED_PIN 48
#define NUM_LEDS 1
CRGB leds[NUM_LEDS];
uint8_t current_hue = 0;
unsigned long last_led_time = 0;
const unsigned long LED_INTERVAL_MS = 10; 

// --- Speed Encoder Pins ---
#define ENC_FL_PIN 13
#define ENC_RL_PIN 47
#define ENC_FR_PIN 21
#define ENC_RR_PIN 38

// Volatile variables for hardware interrupts (Tick Counts)
volatile int32_t ticks_FL = 0;
volatile int32_t ticks_RL = 0;
volatile int32_t ticks_FR = 0;
volatile int32_t ticks_RR = 0;

// Volatile variables for hardware interrupts (Debounce Timestamps)
volatile unsigned long last_tick_FL = 0;
volatile unsigned long last_tick_RL = 0;
volatile unsigned long last_tick_FR = 0;
volatile unsigned long last_tick_RR = 0;

const unsigned long DEBOUNCE_DELAY_US = 2500; 

void IRAM_ATTR isr_FL() { 
  unsigned long current_time = micros(); 
  if (current_time - last_tick_FL > DEBOUNCE_DELAY_US) {
    ticks_FL++; last_tick_FL = current_time;
  }
}
void IRAM_ATTR isr_RL() { 
  unsigned long current_time = micros(); 
  if (current_time - last_tick_RL > DEBOUNCE_DELAY_US) {
    ticks_RL++; last_tick_RL = current_time;
  }
}
void IRAM_ATTR isr_FR() { 
  unsigned long current_time = micros(); 
  if (current_time - last_tick_FR > DEBOUNCE_DELAY_US) {
    ticks_FR++; last_tick_FR = current_time;
  }
}
void IRAM_ATTR isr_RR() { 
  unsigned long current_time = micros(); 
  if (current_time - last_tick_RR > DEBOUNCE_DELAY_US) {
    ticks_RR++; last_tick_RR = current_time;
  }
}

// --- Motor Pins ---
#define IN1_FL 4
#define IN2_FL 5
#define IN1_FR 6
#define IN2_FR 7
#define IN3_RL 15
#define IN4_RL 16
#define IN3_RR 17
#define IN4_RR 18
#define ENA_FL 9
#define ENB_FR 10
#define ENA_RL 11
#define ENB_RR 12

// --- Rover Tuning Parameters ---
const int MIN_PWM = 100;
const int MAX_PWM = 255;
const float MAX_SPEED_CMD = 1.0;
const float DEADBAND = 0.0;

// --- ROS 2 Nodes and Entities ---
rcl_subscription_t subscriber;
geometry_msgs__msg__Twist msg;

rcl_publisher_t encoder_pub;
std_msgs__msg__Int32MultiArray encoder_msg;

rcl_publisher_t status_pub;
std_msgs__msg__Int32 status_msg;

rcl_publisher_t imu_pub;
sensor_msgs__msg__Imu imu_msg;

rcl_publisher_t temp_pub;
sensor_msgs__msg__Temperature temp_msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

unsigned long last_cmd_time = 0;
const unsigned long TIMEOUT_MS = 500;

// --- Stall Detection Variables ---
int32_t current_rover_status = 0; 
bool stall_lockout = false;
float current_cmd_linear = 0.0;
float current_cmd_angular = 0.0;

int32_t prev_ticks_FL = 0;
int32_t prev_ticks_RL = 0;
int32_t prev_ticks_FR = 0;
int32_t prev_ticks_RR = 0;
unsigned long last_stall_check_time = 0;
const unsigned long STALL_CHECK_INTERVAL_MS = 500;

void stop_motors() {
  digitalWrite(IN1_FL, LOW); digitalWrite(IN2_FL, LOW);
  digitalWrite(IN3_RL, LOW); digitalWrite(IN4_RL, LOW);
  digitalWrite(IN1_FR, LOW); digitalWrite(IN2_FR, LOW);
  digitalWrite(IN3_RR, LOW); digitalWrite(IN4_RR, LOW);
  
  analogWrite(ENA_FL, 0); analogWrite(ENA_RL, 0);
  analogWrite(ENB_FR, 0); analogWrite(ENB_RR, 0);
}

void cmd_vel_callback(const void * msgin) {
  last_cmd_time = millis();
  const geometry_msgs__msg__Twist * twist_msg = (const geometry_msgs__msg__Twist *)msgin;
  
  current_cmd_linear = twist_msg->linear.x;
  current_cmd_angular = twist_msg->angular.z; 

  if (abs(current_cmd_linear) <= 0.05 && abs(current_cmd_angular) <= 0.05) {
      stall_lockout = false;
      current_rover_status = 0;
  }

  if (stall_lockout) return; 

  float left_speed, right_speed;

  if (abs(current_cmd_linear) > 0.05) {
      left_speed = current_cmd_linear - current_cmd_angular;
      right_speed = current_cmd_linear + current_cmd_angular;
  } else if (abs(current_cmd_angular) > 2.0) {
      left_speed = -current_cmd_angular;
      right_speed = current_cmd_angular;
  } else {
      left_speed = 0.0;
      right_speed = 0.0;
  }

  left_speed = constrain(left_speed, -MAX_SPEED_CMD, MAX_SPEED_CMD);
  right_speed = constrain(right_speed, -MAX_SPEED_CMD, MAX_SPEED_CMD);

  int left_pwm = 0;
  int right_pwm = 0;

  if (abs(left_speed) > DEADBAND) {
    if (left_speed > 0) { 
      digitalWrite(IN1_FL, HIGH); digitalWrite(IN2_FL, LOW);
      digitalWrite(IN3_RL, HIGH); digitalWrite(IN4_RL, LOW);
    } else { 
      digitalWrite(IN1_FL, LOW); digitalWrite(IN2_FL, HIGH);
      digitalWrite(IN3_RL, LOW); digitalWrite(IN4_RL, HIGH);
    }
    float normalized_speed = abs(left_speed) / MAX_SPEED_CMD;
    left_pwm = constrain(MIN_PWM + (normalized_speed * (MAX_PWM - MIN_PWM)), MIN_PWM, MAX_PWM);
  } else {
    digitalWrite(IN1_FL, LOW); digitalWrite(IN2_FL, LOW);
    digitalWrite(IN3_RL, LOW); digitalWrite(IN4_RL, LOW);
  }
  analogWrite(ENA_FL, left_pwm);
  analogWrite(ENA_RL, left_pwm);

  if (abs(right_speed) > DEADBAND) {
    if (right_speed > 0) { 
      digitalWrite(IN1_FR, HIGH); digitalWrite(IN2_FR, LOW);
      digitalWrite(IN3_RR, HIGH); digitalWrite(IN4_RR, LOW);
    } else { 
      digitalWrite(IN1_FR, LOW); digitalWrite(IN2_FR, HIGH);
      digitalWrite(IN3_RR, LOW); digitalWrite(IN4_RR, HIGH);
    }
    float normalized_speed = abs(right_speed) / MAX_SPEED_CMD;
    right_pwm = constrain(MIN_PWM + (normalized_speed * (MAX_PWM - MIN_PWM)), MIN_PWM, MAX_PWM);
  } else {
    digitalWrite(IN1_FR, LOW); digitalWrite(IN2_FR, LOW);
    digitalWrite(IN3_RR, LOW); digitalWrite(IN4_RR, LOW);
  }
  analogWrite(ENB_FR, right_pwm);
  analogWrite(ENB_RR, right_pwm);
}

bool agent_connected = false;
unsigned long last_ping_time = 0;
const unsigned long PING_INTERVAL_MS = 5000;

void micro_ros_task(void * arg) {
  int missed_pings = 0; 

  while(1) {
    if (agent_connected) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
        
        // Trick the compiler by storing returns in a dummy variable
        rcl_ret_t dummy;

        // Publish Encoders
        encoder_msg.data.data[0] = ticks_FL;
        encoder_msg.data.data[1] = ticks_RL;
        encoder_msg.data.data[2] = ticks_FR;
        encoder_msg.data.data[3] = ticks_RR;
        dummy = rcl_publish(&encoder_pub, &encoder_msg, NULL);

        // Publish Status
        status_msg.data = current_rover_status;
        dummy = rcl_publish(&status_pub, &status_msg, NULL);

        // Grab Shared Timestamp
        int64_t time_ns = rmw_uros_epoch_nanos();
        int32_t sec_stamp = (int32_t)(time_ns / 1000000000);
        uint32_t nano_stamp = (uint32_t)(time_ns % 1000000000);

        // Read Sensor
        sensors_event_t a, g, temp_event;
        mpu.getEvent(&a, &g, &temp_event);

        // --- Publish IMU Data ---
        imu_msg.header.frame_id.data = (char*)"imu_link";
        imu_msg.header.frame_id.size = strlen(imu_msg.header.frame_id.data);
        imu_msg.header.frame_id.capacity = imu_msg.header.frame_id.size + 1;
        imu_msg.header.stamp.sec = sec_stamp;
        imu_msg.header.stamp.nanosec = nano_stamp;

        imu_msg.orientation_covariance[0] = 0.001; // Roll
        imu_msg.orientation_covariance[4] = 0.001; // Pitch
        imu_msg.orientation_covariance[8] = 0.001; // Yaw

        // Angular Velocity Covariance (Gyroscope noise floor)
        imu_msg.angular_velocity_covariance[0] = 0.001; 
        imu_msg.angular_velocity_covariance[4] = 0.001; 
        imu_msg.angular_velocity_covariance[8] = 0.001; 

        // Linear Acceleration Covariance (Accelerometer noise floor)
        imu_msg.linear_acceleration_covariance[0] = 0.05; 
        imu_msg.linear_acceleration_covariance[4] = 0.05; 
        imu_msg.linear_acceleration_covariance[8] = 0.05; 

        // Apply calculated offsets
        imu_msg.linear_acceleration.x = a.acceleration.x - accel_x_offset;
        imu_msg.linear_acceleration.y = a.acceleration.y - accel_y_offset;
        imu_msg.linear_acceleration.z = a.acceleration.z; // Gravity left untouched

        imu_msg.angular_velocity.x = g.gyro.x - gyro_x_offset;
        imu_msg.angular_velocity.y = g.gyro.y - gyro_y_offset;
        imu_msg.angular_velocity.z = g.gyro.z - gyro_z_offset;

        dummy = rcl_publish(&imu_pub, &imu_msg, NULL);

        // --- Publish Temperature Data ---
        temp_msg.header.frame_id.data = (char*)"imu_link";
        temp_msg.header.frame_id.size = strlen(temp_msg.header.frame_id.data);
        temp_msg.header.frame_id.capacity = temp_msg.header.frame_id.size + 1;
        temp_msg.header.stamp.sec = sec_stamp;
        temp_msg.header.stamp.nanosec = nano_stamp;
        
        temp_msg.temperature = temp_event.temperature;
        temp_msg.variance = 0.0; // Variance unknown

        dummy = rcl_publish(&temp_pub, &temp_msg, NULL);

        // Explicitly cast dummy to void to silence GCC
        (void)dummy;
    }
    
    if (millis() - last_ping_time > PING_INTERVAL_MS) {
      last_ping_time = millis();
      if (rmw_uros_ping_agent(50, 1) == RMW_RET_OK) {
          missed_pings = 0; agent_connected = true;
      } else {
          missed_pings++; 
          if (agent_connected && missed_pings >= 3) {
              agent_connected = false; stop_motors(); ESP.restart(); 
          }
      }
    }
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

void led_watchdog_task(void * arg) {
  while(1) {
    bool safety_stop = false;

    if (millis() - last_cmd_time > TIMEOUT_MS) {
      stop_motors(); safety_stop = true;
      current_cmd_linear = 0.0; current_cmd_angular = 0.0;
    }

    if (millis() - last_stall_check_time > STALL_CHECK_INTERVAL_MS) {
        last_stall_check_time = millis();
        if (!safety_stop && !stall_lockout && (abs(current_cmd_linear) > 0.05 || abs(current_cmd_angular) > 0.05)) {
            int32_t delta_FL = ticks_FL - prev_ticks_FL;
            int32_t delta_RL = ticks_RL - prev_ticks_RL;
            int32_t delta_FR = ticks_FR - prev_ticks_FR;
            int32_t delta_RR = ticks_RR - prev_ticks_RR;

            if (delta_FL == 0 && delta_RL == 0 && delta_FR == 0 && delta_RR == 0) {
                stall_lockout = true; current_rover_status = 1; stop_motors();
            }
        }
        prev_ticks_FL = ticks_FL; prev_ticks_RL = ticks_RL;
        prev_ticks_FR = ticks_FR; prev_ticks_RR = ticks_RR;
    }

    if (millis() - last_led_time > LED_INTERVAL_MS) {
      last_led_time = millis();
      if (!agent_connected) {
          leds[0] = CRGB::Red; 
      } else if (stall_lockout) {
          leds[0] = CRGB::Red; 
      } else if (safety_stop) {
          leds[0] = CRGB::Lime; 
      } else {
          leds[0] = CHSV(current_hue, 255, 255); current_hue++;
      }
      FastLED.show();
    }
    vTaskDelay(10 / portTICK_PERIOD_MS); 
  }
}

void setup() {
  // Initialize the serial port exactly at the baud rate the Pi Agent expects
  Serial.begin(115200);
  
  // Set up the Serial Transport BEFORE initializing the ROS nodes
  set_microros_serial_transports(Serial);

  FastLED.addLeds<WS2812, LED_PIN, GRB>(leds, NUM_LEDS);
  FastLED.setBrightness(255);
  
  leds[0] = CRGB::White;
  FastLED.show();
  delay(1000); 

  Wire.begin(SDA_PIN, SCL_PIN);
  if (!mpu.begin(0x68, &Wire)) {
    
  } else {
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ); 
  }

  leds[0] = CRGB::Cyan; // Visual cue that calibration is running
  FastLED.show();
  
  int num_readings = 2000; 
  for (int i = 0; i < num_readings; i++) {
    sensors_event_t a, g, temp_event;
    mpu.getEvent(&a, &g, &temp_event);
    
    accel_x_offset += a.acceleration.x;
    accel_y_offset += a.acceleration.y;
    gyro_x_offset += g.gyro.x;
    gyro_y_offset += g.gyro.y;
    gyro_z_offset += g.gyro.z;
    delay(5);
  }
  accel_x_offset /= num_readings;
  accel_y_offset /= num_readings;
  gyro_x_offset /= num_readings;
  gyro_y_offset /= num_readings;
  gyro_z_offset /= num_readings;

  pinMode(ENA_FL, OUTPUT); pinMode(IN1_FL, OUTPUT); pinMode(IN2_FL, OUTPUT);
  pinMode(ENB_FR, OUTPUT); pinMode(IN1_FR, OUTPUT); pinMode(IN2_FR, OUTPUT);
  pinMode(ENA_RL, OUTPUT); pinMode(IN3_RL, OUTPUT); pinMode(IN4_RL, OUTPUT);
  pinMode(ENB_RR, OUTPUT); pinMode(IN3_RR, OUTPUT); pinMode(IN4_RR, OUTPUT);
  stop_motors();

  pinMode(ENC_FL_PIN, INPUT_PULLUP); pinMode(ENC_RL_PIN, INPUT_PULLUP);
  pinMode(ENC_FR_PIN, INPUT_PULLUP); pinMode(ENC_RR_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_FL_PIN), isr_FL, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_RL_PIN), isr_RL, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_FR_PIN), isr_FR, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_RR_PIN), isr_RR, RISING);

  bool toggle_blue = false;
  while (rmw_uros_ping_agent(1000, 1) != RMW_RET_OK) {
      leds[0] = toggle_blue ? CRGB::Blue : CRGB::Black; FastLED.show(); toggle_blue = !toggle_blue; delay(500); 
  }
  
  leds[0] = CRGB::Blue; FastLED.show();
  agent_connected = true;
  delay(1000); 

  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "esp32_rover_node", "", &support);

  rclc_subscription_init_default(&subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "cmd_vel_throttled");
  rclc_publisher_init_default(&encoder_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray), "wheel_ticks");
  
  encoder_msg.data.capacity = 4; encoder_msg.data.size = 4;
  encoder_msg.data.data = (int32_t*) malloc(4 * sizeof(int32_t));

  rclc_publisher_init_default(&status_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "rover_status");
  rclc_publisher_init_default(&imu_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), "imu/data_raw");
  rclc_publisher_init_default(&temp_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Temperature), "imu/temperature");

  rclc_executor_init(&executor, &support.context, 1, &allocator);
  rclc_executor_add_subscription(&executor, &subscriber, &msg, &cmd_vel_callback, ON_NEW_DATA);

  // --- SYNCHRONIZE TIME ---
  leds[0] = CRGB::Orange; 
  FastLED.show();

  int sync_attempts = 0;
  while (!rmw_uros_epoch_synchronized() && sync_attempts < 10) {
      rmw_uros_sync_session(1000);
      sync_attempts++;
      delay(100);
  }
  
  leds[0] = CRGB::Blue;
  FastLED.show();

  xTaskCreatePinnedToCore(micro_ros_task, "micro_ros_task", 4096, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(led_watchdog_task, "led_task", 2048, NULL, 1, NULL, 1);
  vTaskDelete(NULL);
}

/*
// WiFi setup
void setup() {
  Serial.begin(115200);
  
  FastLED.addLeds<WS2812, LED_PIN, GRB>(leds, NUM_LEDS);
  FastLED.setBrightness(255);
  
  leds[0] = CRGB::White;
  FastLED.show();
  delay(1000); 

  Wire.begin(SDA_PIN, SCL_PIN);
  if (!mpu.begin(0x68, &Wire)) {
    Serial.println("Failed to find MPU6500 chip");
  } else {
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ); // The 21Hz internal hardware low-pass filter
  }

  // --- DEEP IMU CALIBRATION ---
  Serial.println("Calibrating IMU... DO NOT MOVE THE ROVER");
  leds[0] = CRGB::Cyan; // Visual cue that calibration is running
  FastLED.show();
  
  int num_readings = 2000; // 10 seconds of calibration data
  for (int i = 0; i < num_readings; i++) {
    sensors_event_t a, g, temp_event;
    mpu.getEvent(&a, &g, &temp_event);
    
    accel_x_offset += a.acceleration.x;
    accel_y_offset += a.acceleration.y;
    gyro_x_offset += g.gyro.x;
    gyro_y_offset += g.gyro.y;
    gyro_z_offset += g.gyro.z;
    delay(5);
  }
  accel_x_offset /= num_readings;
  accel_y_offset /= num_readings;
  gyro_x_offset /= num_readings;
  gyro_y_offset /= num_readings;
  gyro_z_offset /= num_readings;

  pinMode(ENA_FL, OUTPUT); pinMode(IN1_FL, OUTPUT); pinMode(IN2_FL, OUTPUT);
  pinMode(ENB_FR, OUTPUT); pinMode(IN1_FR, OUTPUT); pinMode(IN2_FR, OUTPUT);
  pinMode(ENA_RL, OUTPUT); pinMode(IN3_RL, OUTPUT); pinMode(IN4_RL, OUTPUT);
  pinMode(ENB_RR, OUTPUT); pinMode(IN3_RR, OUTPUT); pinMode(IN4_RR, OUTPUT);
  stop_motors();

  pinMode(ENC_FL_PIN, INPUT_PULLUP); pinMode(ENC_RL_PIN, INPUT_PULLUP);
  pinMode(ENC_FR_PIN, INPUT_PULLUP); pinMode(ENC_RR_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_FL_PIN), isr_FL, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_RL_PIN), isr_RL, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_FR_PIN), isr_FR, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_RR_PIN), isr_RR, RISING);

  leds[0] = CRGB::Yellow;
  FastLED.show();

  WiFi.mode(WIFI_STA);
  unsigned long global_start = millis();
  const unsigned long TOTAL_TIMEOUT_MS = 120000;

  while (WiFi.status() != WL_CONNECTED && (millis() - global_start < TOTAL_TIMEOUT_MS)) {
      WiFi.disconnect(true, true); delay(200);
      WiFi.begin("Pi_Hotspot", "SuperSecretPassword");
      unsigned long phase_start = millis();
      while (WiFi.status() != WL_CONNECTED && (millis() - phase_start < 10000)) {
          leds[0] = (millis() % 1000 < 500) ? CRGB::Yellow : CRGB::Black; FastLED.show(); delay(250);
      }
      if (WiFi.status() == WL_CONNECTED) break;

      WiFi.disconnect(); delay(200);
      uint8_t new_mac[6]; esp_read_mac(new_mac, ESP_MAC_WIFI_STA);
      new_mac[5] = (new_mac[5] + (millis() % 50) + 1) % 255; esp_wifi_set_mac(WIFI_IF_STA, new_mac);
      WiFi.begin("Pi_Hotspot", "SuperSecretPassword");
      phase_start = millis();
      while (WiFi.status() != WL_CONNECTED && (millis() - phase_start < 10000)) {
          leds[0] = (millis() % 200 < 100) ? CRGB::Magenta : CRGB::Black; FastLED.show(); delay(100);
      }
      if (WiFi.status() == WL_CONNECTED) break;

      WiFi.disconnect(true, true); phase_start = millis();
      while (millis() - phase_start < 5000) {
          leds[0] = (millis() % 2000 < 1000) ? CRGB::Yellow : CRGB::Black; FastLED.show(); delay(500);
      }
  }

  if (WiFi.status() != WL_CONNECTED) {
      WiFi.mode(WIFI_OFF); 
      while (true) { leds[0] = CRGB::Purple; FastLED.show(); delay(500); leds[0] = CRGB::Black; FastLED.show(); delay(500); }
  }

  leds[0] = CRGB::Blue; FastLED.show();
  IPAddress agent_ip(192, 168, 4, 1);
  set_microros_wifi_transports((char*)"Pi_Hotspot", (char*)"SuperSecretPassword", agent_ip, 8888);

  bool toggle_blue = false;
  while (rmw_uros_ping_agent(1000, 1) != RMW_RET_OK) {
      leds[0] = toggle_blue ? CRGB::Blue : CRGB::Black; FastLED.show(); toggle_blue = !toggle_blue; delay(500); 
  }
  
  leds[0] = CRGB::Blue; FastLED.show();
  agent_connected = true;
  delay(1000); 

  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "esp32_rover_node", "", &support);

  rclc_subscription_init_default(&subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "cmd_vel_throttled");
  rclc_publisher_init_default(&encoder_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray), "wheel_ticks");
  
  encoder_msg.data.capacity = 4; encoder_msg.data.size = 4;
  encoder_msg.data.data = (int32_t*) malloc(4 * sizeof(int32_t));

  rclc_publisher_init_default(&status_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "rover_status");
  rclc_publisher_init_default(&imu_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), "imu/data_raw");
  
  // Initialize Temperature Publisher
  rclc_publisher_init_default(&temp_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Temperature), "imu/temperature");

  rclc_executor_init(&executor, &support.context, 1, &allocator);
  rclc_executor_add_subscription(&executor, &subscriber, &msg, &cmd_vel_callback, ON_NEW_DATA);

  // --- SYNCHRONIZE TIME (FORCE RETRY W/ SAFEGUARD) ---
  Serial.println("Syncing time with ROS Agent...");
  leds[0] = CRGB::Orange; // Visual indicator that it's waiting for time
  FastLED.show();

  int sync_attempts = 0;
  while (!rmw_uros_epoch_synchronized() && sync_attempts < 10) {
      rmw_uros_sync_session(1000);
      sync_attempts++;
      delay(100);
  }
  
  if (rmw_uros_epoch_synchronized()) {
      Serial.println("Time synced perfectly!");
  } else {
      Serial.println("Warning: Time sync failed, reverting to local uptime.");
  }
  
  leds[0] = CRGB::Blue;
  FastLED.show();

  xTaskCreatePinnedToCore(micro_ros_task, "micro_ros_task", 4096, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(led_watchdog_task, "led_task", 2048, NULL, 1, NULL, 1);
  vTaskDelete(NULL);
}

*/

void loop() { }