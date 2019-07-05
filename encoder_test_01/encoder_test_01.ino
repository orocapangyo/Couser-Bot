#include <ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <Wire.h>
#include <Adafruit_MotorShield.h>

std_msgs::String encoder_msg;


void cmdCallback(const geometry_msgs::Twist& cmd_vel_msg);
ros::Publisher encoder("encoder", &encoder_msg);
ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("cmd_vel", cmdCallback);
ros::NodeHandle nh;

long publisher_timer;

//Motor driver stack 1
Adafruit_MotorShield AFMS = Adafruit_MotorShield();//0x60

Adafruit_DCMotor *LMotor = AFMS.getMotor(2);
Adafruit_DCMotor *RMotor = AFMS.getMotor(3);


// encoder pin
#define encoderPinRA 2
#define encoderPinRB 3
#define encoderPinLA 18
#define encoderPinLB 19

int encoderPosR = 0;
int encoderPosL = 0;

void doEncoderA(const int encoderPinA){ // 빨녹일 때
  if(encoderPinA==2){
    encoderPosR += (digitalRead(encoderPinRA)==digitalRead(encoderPinRB))?1:-1;
  }
  else if(encoderPinA==18){
    encoderPosL += (digitalRead(encoderPinLA)==digitalRead(encoderPinLB))?1:-1;
  }
}

void doEncoderB(const int encoderPinB){ // 보파일 때
  if(encoderPinB==3){
    encoderPosR += (digitalRead(encoderPinRA)==digitalRead(encoderPinRB))?-1:1;
  }
  else if(encoderPinB==19){
    encoderPosL += (digitalRead(encoderPinLA)==digitalRead(encoderPinLB))?-1:1;
  }
}

void forward(const float cmd){
   LMotor->run(FORWARD);
   RMotor->run(FORWARD);

   LMotor->setSpeed(cmd*10);
   RMotor->setSpeed(cmd*10);
}

void turn(const float cmd){
  if(cmd >= 0){
   LMotor->run(FORWARD);
   RMotor->run(BACKWARD);
   LMotor->setSpeed(cmd*10);
   RMotor->setSpeed(cmd*10);
   
  }
  else{
   LMotor->run(BACKWARD);
   RMotor->run(FORWARD);
   LMotor->setSpeed(cmd*10);
   RMotor->setSpeed(cmd*10);
  }
  
}

void cmdCallback(const geometry_msgs::Twist& cmd_vel_msg){
  forward(cmd_vel_msg.linear.x);
  turn(cmd_vel_msg.angular.z);
}

void setup() {
  nh.initNode();
  nh.advertise(encoder);
  nh.subscribe(cmd_vel_sub);
  
  AFMS.begin();//1.6kHz

  pinMode(encoderPinRA, INPUT_PULLUP);
  pinMode(encoderPinRB, INPUT_PULLUP);
  pinMode(encoderPinLA, INPUT_PULLUP);
  pinMode(encoderPinLB, INPUT_PULLUP);
  
  attachInterrupt(0,doEncoderA(encoderPinRA),CHANGE); //2
  attachInterrupt(1,doEncoderB(encoderPinRB),CHANGE); //3
  attachInterrupt(5,doEncoderA(encoderPinLA),CHANGE); //18
  attachInterrupt(4,doEncoderB(encoderPinLB),CHANGE); //19

  Serial.begin(115200);
}

void loop() {

  String data = "A" + (String)encoderPosL + "B" + (String)encoderPosR + "C";
  int length = data.indexOf("C")+2;
  char data_final[length+1];
  data.toCharArray(data_final, length+1);


  if (millis() > publisher_timer) {
    // step 1: request reading from sensor
    encoder_msg.data = data_final;
    encoder.publish(&encoder_msg);
    publisher_timer = millis() + 100; //publish ten times a second
    nh.spinOnce();
  }
}
