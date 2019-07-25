/*******************************************************************************
* Copyright 2016 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/* Authors: Yoonseok Pyo, Leon Jung, Darby Lim, HanCheol Cho */

#include "../../include/CouserBot_motor_driver.h"

CouserBotMotorDriver::CouserBotMotorDriver()
: baudrate_(BAUDRATE),
  protocol_version_(PROTOCOL_VERSION),
  dcmotor_limit_max_velocity_(DCMOTOR_LIMIT_MAX_VELOCITY)
{
}

CouserBotMotorDriver::~CouserBotMotorDriver()
{
  close();
}

bool CouserBotMotorDriver::init(String couserbot)
{ 
  Serial.println("Success to init Motor Driver");
  return true;
}

void CouserBotMotorDriver::close(void)
{
}

bool CouserBotMotorDriver::controlMotor(const float wheel_radius, const float wheel_separation, float* value)
{
  float wheel_velocity_cmd[2];

  float lin_vel = value[0];
  float ang_vel = value[1];

  wheel_velocity_cmd[LEFT]   = lin_vel - (ang_vel * wheel_separation / 2);
  wheel_velocity_cmd[RIGHT]  = lin_vel + (ang_vel * wheel_separation / 2);

  wheel_velocity_cmd[LEFT]  = constrain(SCALING_FACTOR * MOTOR_VELOCITY_COEFFICIENT * wheel_velocity_cmd[LEFT] / wheel_radius, -dcmotor_limit_max_velocity_, dcmotor_limit_max_velocity_);
  wheel_velocity_cmd[RIGHT] = constrain(SCALING_FACTOR * MOTOR_VELOCITY_COEFFICIENT * wheel_velocity_cmd[RIGHT] / wheel_radius, -dcmotor_limit_max_velocity_, dcmotor_limit_max_velocity_);

  if(wheel_velocity_cmd[LEFT] >= 0){
    LMotor->run(Forward);
  }
  else{
    LMotor->run(Backward);
  }
  if(wheel_velocity_cmd[RIGHT] >= 0){
    RMotor->run(Forward);
  }
  else{
    RMotor->run(Backward);
  }

  LMotor->setSpeed(wheel_velocity_cmd[LEFT]);
  RMotor->setSpeed(wheel_velocity_cmd[RIGHT]);

 return true;
}

bool CouserBotMotorDriver::readEncoder(int32_t &left_value, int32_t &right_value, int32_t &left_encoder, int32_t &right_encoder)
{
  //Encoder 값 받아오기
  //Encoder 값 Calibration
  //sensor_state_msg에 left_encoder, right_encoder 값 넣기

  int32_t scaler = 1; //-> No need

  //Encoder 값 Scaling
  left_value *= scaler;
  right_value *= scaler;

  //sensor_state_msg에 value 넣기
  left_encoder = left_value;
  right_encoder = right_value;
  //*** sensor_state_msg.left_encoder, sensor_state_msg.right_encoder 
  
  //Encoder 값 검증 Code -> FAIL 시 FALSE 반환
  
  return true;
}
