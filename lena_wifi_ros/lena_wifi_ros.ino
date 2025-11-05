#define ROSSERIAL_ARDUINO_TCP

#include <AVision_ESP8266.h>
#include <ESP8266WiFi.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Float32.h>
#include <std_msgs/UInt16.h>
#include <string.h>
#include <Servo.h>
#include <SharpIR.h>
#include "Wire.h"
#include <MPU6050_light.h>

MPU6050 mpu(Wire);
unsigned long timer = 0;

// Define modelo e pino do sensor IR:
#define IRPin A0
#define model 1080
SharpIR SharpSensor(SharpIR::GP2Y0A41SK0F, A0);

#define base_port 11411 

#define MAX_POINTS 180  // 180 pontos com servo_step = 2

struct Point {
  float x;
  float y;
};

//----------------------
#define top_cmd_vel "/lena/cmd_vel"
#define top_servo   "/lena/ir_motor"   // Agora será publisher
#define top_sharp   "/lena/ir_sensor"
#define top_imu     "/lena/heading"
#define top_renc    "/lena/raw/right_ticks"
#define top_lenc    "/lena/raw/left_ticks"
#define top_point   "/lena/ir_point"   // Novo tópico para pontos (x,y)
//----------------------

Servo servo; 
int pos = 0;
int dir = 0;
int servo_step = 6;     // Passo de 2° para varredura mais rápida
int servo_delay = 20;   // Delay reduzido para 10ms
bool scanning = true;   // Flag para ativar/desativar varredura

//----------------------
int IN1 = 3;
int IN2 = 1;
int IN3 = 16;
int IN4 = 13;
int LED = 2;
int SRV = 0;
int EC1 = 12;
int EC2 = 14;
//----------------------

int EC1_count = 0;
int EC2_count = 0;
unsigned long last_led_toggle = 0;
bool led_state = false;

struct config_t {
  const char* ssid = "EU ROBO";
  const char* password = "RoverBSI";
  uint16_t serverPort = base_port; 
  int serverIP[4] = {192, 168, 100, 39}; // IP do computador com roscore
  int robot = 0;
  const char* topic_servo = top_servo;
  const char* topic_lenc = top_lenc;
  const char* topic_renc = top_renc;
  const char* topic_cmd_vel = top_cmd_vel;
  const char* topic_sharp = top_sharp;
  const char* topic_imu = top_imu;
  const char* topic_point = top_point;
} configuration;

std_msgs::Float32 sharp_msg, lenc_msg, renc_msg, imu_msg;
std_msgs::UInt16 servo_msg;
geometry_msgs::Point point_msg;

ros::Publisher pub_lenc(configuration.topic_lenc, &lenc_msg);
ros::Publisher pub_renc(configuration.topic_renc, &renc_msg);
ros::Publisher pub_sharp(configuration.topic_sharp, &sharp_msg);
ros::Publisher pub_imu(configuration.topic_imu, &imu_msg);
ros::Publisher pub_servo(configuration.topic_servo, &servo_msg);
ros::Publisher pub_point(configuration.topic_point, &point_msg);

ros::Subscriber<geometry_msgs::Twist> sub_cmd_vel(configuration.topic_cmd_vel, &odometry_cb);

void odometry_cb(const geometry_msgs::Twist& msg) {
  float forward, lateral, wheelL, wheelR, wlength = 0.115, wradius = 0.0685/2, reduction = 48;
  forward = msg.linear.x;
  lateral = msg.angular.z;

  wheelR = ((forward / wradius) + (lateral * wlength) / (2 * wradius)) * reduction;
  wheelL = ((forward / wradius) - (lateral * wlength) / (2 * wradius)) * reduction;

  if (wheelR <= 0) {
    analogWrite(IN1, wheelR);
    analogWrite(IN2, 0);
  } else {
    analogWrite(IN1, 0);
    analogWrite(IN2, abs(wheelR));
  }
  if (wheelL <= 0) {
    analogWrite(IN3, wheelL);
    analogWrite(IN4, 0);
  } else {
    analogWrite(IN3, 0);
    analogWrite(IN4, abs(wheelL));
  }  
}

void setupWiFi() {
  WiFi.begin(configuration.ssid, configuration.password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
  }
}

ros::NodeHandle nh;

void IRAM_ATTR ISR_EC1() {
  EC1_count++;
}

void IRAM_ATTR ISR_EC2() {
  EC2_count++;
}

void setup() {
  configuration.serverPort = configuration.serverPort + configuration.robot;  
  IPAddress server(configuration.serverIP[0], configuration.serverIP[1], configuration.serverIP[2], configuration.serverIP[3]);
  setupWiFi();
  delay(2000);
  nh.getHardware()->setConnection(server, configuration.serverPort); 
  nh.initNode();
  nh.subscribe(sub_cmd_vel);
  nh.advertise(pub_lenc);
  nh.advertise(pub_renc);
  nh.advertise(pub_sharp);
  nh.advertise(pub_imu);
  nh.advertise(pub_servo);
  nh.advertise(pub_point);

  // IMU Initialization
  Wire.begin();
  byte status = mpu.begin();
  while (status != 0) { }
  delay(1000);
  mpu.calcOffsets();

  // Configure GPIO's
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(LED, OUTPUT);
  servo.attach(SRV);
  servo.write(90);
  pos = 90;
  pinMode(EC1, INPUT);
  attachInterrupt(EC1, ISR_EC1, RISING);
  pinMode(EC2, INPUT);
  attachInterrupt(EC2, ISR_EC2, RISING);
}

void loop() {
  // Controle do LED com millis() (não bloqueante)
  if (millis() - last_led_toggle >= 100) {
    led_state = !led_state;
    digitalWrite(LED, led_state ? HIGH : LOW);
    last_led_toggle = millis();
  }

  // Publica encoders
  lenc_msg.data = EC1_count;
  pub_lenc.publish(&lenc_msg);
  
  renc_msg.data = EC2_count;
  pub_renc.publish(&renc_msg);

  // IMU update e publish
  mpu.update();
  float yaw = mpu.getAngleZ();
  imu_msg.data = yaw;
  pub_imu.publish(&imu_msg);

  // Varredura do servo e geração de nuvem de pontos
  if (scanning) {
    servo.write(pos);
    delay(servo_delay);

    // Publica posição atual do servo
    servo_msg.data = pos;
    pub_servo.publish(&servo_msg);

    // Lê distância com filtro (média de 2 leituras)
    float dist_cm = 0;
    for (int i = 0; i < 2; i++) {
     dist_cm += (SharpSensor.getDistance() * 1.8) * 2.54;
     delay(1);
    }
    // dist_cm /= 2.0;
    //dist_cm += (SharpSensor.getDistance() * 1.8) * 2.54;
    sharp_msg.data = dist_cm;
    pub_sharp.publish(&sharp_msg);

    // Calcula ponto global
    float global_angle_deg = pos + yaw;
    float global_angle_rad = global_angle_deg * PI / 180.0;
    float x = cos(global_angle_rad) * dist_cm;
    float y = sin(global_angle_rad) * dist_cm;

    // Publica ponto (x, y) em metros
    point_msg.x = x / 100.0; // convertendo para metros
    point_msg.y = y / 100.0;
    point_msg.z = 0.0;
    pub_point.publish(&point_msg);

    // Avança o servo
    if (dir == 0) {
      pos += servo_step;
      if (pos >= 180) {
        dir = 1;
      }
    } else {
      pos -= servo_step;
      if (pos <= 0) {
        dir = 0;
      }
    }
  }

  nh.spinOnce();
}
