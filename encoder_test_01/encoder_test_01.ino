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
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); //0x60

Adafruit_DCMotor *LMotor = AFMS.getMotor(2);
Adafruit_DCMotor *RMotor = AFMS.getMotor(3);


// encoder pin
#define encoderPinRA 2
#define encoderPinRB 3
#define encoderPinLA 18
#define encoderPinLB 19

int encoderPosR = 0;
int encoderPosL = 0;

void getEncoder_LA(){ // pin 18,19
  encoderPosL += (digitalRead(encoderPinLA)==digitalRead(encoderPinLB))?1:-1;
}
void getEncoder_LB(){ // pin 18,19
  encoderPosL += (digitalRead(encoderPinLA)==digitalRead(encoderPinLB))?-1:1;
}
void getEncoder_RA(){ // pin 2, 3
  encoderPosR += (digitalRead(encoderPinRA)==digitalRead(encoderPinRB))?1:-1;
}
void getEncoder_RB(){ // pin 2, 3
  encoderPosR += (digitalRead(encoderPinRA)==digitalRead(encoderPinRB))?-1:1;
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
  
  attachInterrupt(0,getEncoder_LA,CHANGE); //2
  attachInterrupt(1,getEncoder_LB,CHANGE); //3
  attachInterrupt(5,getEncoder_RA,CHANGE); //18
  attachInterrupt(4,getEncoder_RB,CHANGE); //19

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
