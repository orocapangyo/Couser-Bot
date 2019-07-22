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
  protocol_version_(PROTOCOL_VERSION)
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

  wheel_velocity_cmd[LEFT]  = constrain(SCALING_FACTOR * MOTOR_VELOCITY_COEFFICIENT * wheel_velocity_cmd[LEFT] / wheel_radius, -DCMOTOR_LIMIT_MAX_VELOCITY, DCMOTOR_LIMIT_MAX_VELOCITY);
  wheel_velocity_cmd[RIGHT] = constrain(SCALING_FACTOR * MOTOR_VELOCITY_COEFFICIENT * wheel_velocity_cmd[RIGHT] / wheel_radius, -DCMOTOR_LIMIT_MAX_VELOCITY, DCMOTOR_LIMIT_MAX_VELOCITY);

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

bool CouserBotMotorDriver::readEncoder(int32_t &left_value, int32_t &right_value)
{
  //Encoder 값 받아오기
  //Encoder 값 Calibration
  //sensor_state_msg에 left_encoder, right_encoder 값 넣기

  //*** sensor_state_msg.left_encoder, sensor_state_msg.right_encoder 
  
  
  
  
  /*
  int dxl_comm_result = COMM_TX_FAIL;              // Communication result
  bool dxl_addparam_result = false;                // addParam result
  bool dxl_getdata_result = false;                 // GetParam result

  // Set parameter
  dxl_addparam_result = groupSyncReadEncoder_->addParam(left_wheel_id_);
  if (dxl_addparam_result != true)
    return false;

  dxl_addparam_result = groupSyncReadEncoder_->addParam(right_wheel_id_);
  if (dxl_addparam_result != true)
    return false;

  // Syncread present position
  dxl_comm_result = groupSyncReadEncoder_->txRxPacket();
  if (dxl_comm_result != COMM_SUCCESS)
    Serial.println(packetHandler_->getTxRxResult(dxl_comm_result));

  // Check if groupSyncRead data of Dynamixels are available
  dxl_getdata_result = groupSyncReadEncoder_->isAvailable(left_wheel_id_, ADDR_X_PRESENT_POSITION, LEN_X_PRESENT_POSITION);
  if (dxl_getdata_result != true)
    return false;

  dxl_getdata_result = groupSyncReadEncoder_->isAvailable(right_wheel_id_, ADDR_X_PRESENT_POSITION, LEN_X_PRESENT_POSITION);
  if (dxl_getdata_result != true)
    return false;

  // Get data
  left_value  = groupSyncReadEncoder_->getData(left_wheel_id_,  ADDR_X_PRESENT_POSITION, LEN_X_PRESENT_POSITION);
  right_value = groupSyncReadEncoder_->getData(right_wheel_id_, ADDR_X_PRESENT_POSITION, LEN_X_PRESENT_POSITION);

  groupSyncReadEncoder_->clearParam();
  */
  return true;
}
