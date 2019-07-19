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

#ifndef CouserBot_MOTOR_DRIVER_H_
#define CouserBot_MOTOR_DRIVER_H_

#include "variant.h"
#include <Adafruit_MotorShield.h>

#define BAUDRATE 115200
#define PROTOCOL_VERSION 1.0
#define LEFT_WHEEL 2
#define RIGHT_WHEEL 3
#define DCMOTOR_LIMIT_MAX_VELOCITY 200

Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *LMotor = AFMS.getMotor(LEFT_WHEEL);
Adafruit_DCMotor *RMotor = AFMS.getMotor(RIGHT_WHEEL);

#define LEFT                            0
#define RIGHT                           1

#define MOTOR_VELOCITY_COEFFICIENT      535
#define SCALING_FACTOR                  0.067
// Motor 감속비 56:1, 모터정격rpm 5100 RPM, 정격 토크 60 gfcm
// Wheel rpm : 91 RPM
// w_wheel [rpm] = (V/r) * (60/2*pi)
// w_motor [rpm] = 56 * w_wheel

class CouserBotMotorDriver
{
 public:
  CouserBotMotorDriver();
  ~CouserBotMotorDriver();
  bool init(String couserbot);
  void close(void);
  bool controlMotor(const float wheel_radius, const float wheel_separation, float* value);
  
 private:
  uint32_t baudrate_;
  float  protocol_version_;
  bool torque_;

  uint16_t dcmotor_limit_max_velocity_;
  //uint16_t dynamixel_limit_max_velocity_;

  //dynamixel::PortHandler *portHandler_;
  //dynamixel::PacketHandler *packetHandler_;

  //dynamixel::GroupSyncWrite *groupSyncWriteVelocity_;
  //dynamixel::GroupSyncRead *groupSyncReadEncoder_;
};

#endif // COUSERBOT_MOTOR_DRIVER_H_
