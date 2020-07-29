/*******************************************************************************
* Copyright 2016 VKROBOT CO., LTD.
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

/* Authors: jiapeng.feng */

#ifndef TRANSBOT_MOTOR_DRIVER_H_
#define TRANSBOT_MOTOR_DRIVER_H_

#include "variant.h"
#include <DynamixelSDK.h>

// Control table address (Dynamixel X-series)
/*
#define ADDR_X_TORQUE_ENABLE            64
#define ADDR_X_GOAL_VELOCITY            104
#define ADDR_X_GOAL_POSITION            116
#define ADDR_X_REALTIME_TICK            120
#define ADDR_X_PRESENT_VELOCITY         128
#define ADDR_X_PRESENT_POSITION         132
*/

#define ADDR_X_TORQUE_ENABLE            40
#define ADDR_X_GOAL_VELOCITY            46
#define ADDR_X_GOAL_POSITION            42
#define ADDR_X_REALTIME_TICK            120
#define ADDR_X_PRESENT_VELOCITY         58
#define ADDR_X_PRESENT_POSITION         56

// Limit values (XM430-W210-T and XM430-W350-T)
#define BURGER_DXL_LIMIT_MAX_VELOCITY            265     // MAX RPM is 61 when XL is powered 12.0V
#define WAFFLE_DXL_LIMIT_MAX_VELOCITY            330     // MAX RPM is 77 when XM is powered 12.0V

// Data Byte Length
#define LEN_X_TORQUE_ENABLE             1
#define LEN_X_GOAL_VELOCITY             2
#define LEN_X_GOAL_POSITION             2
#define LEN_X_REALTIME_TICK             2
#define LEN_X_PRESENT_VELOCITY          2
#define LEN_X_PRESENT_POSITION          2

//#define PROTOCOL_VERSION                2.0     // Dynamixel protocol version 2.0
#define PROTOCOL_VERSION                1.0     // Dynamixel protocol version 2.0

#define DXL_LEFT_ID                     1       // ID of left motor
#define DXL_RIGHT_ID                    2       // ID of right motor

#define BAUDRATE                        1000000 // baurd rate of Dynamixel
#define DEVICENAME                      ""      // no need setting on OpenCR

#define TORQUE_ENABLE                   1       // Value for enabling the torque
#define TORQUE_DISABLE                  0       // Value for disabling the torque

#define LEFT                            0
#define RIGHT                           1

//#define VELOCITY_CONSTANT_VALUE         41.69988758  // V = r * w = r     *        (RPM             * 0.10472)
                                                     //           = r     * (0.229 * Goal_Velocity) * 0.10472
                                                     //
                                                     // Goal_Velocity = V / r * 41.69988757710309
#define VELOCITY_CONSTANT_VALUE         13.0404
#define DEBUG_SERIAL  SerialBT2

class Turtlebot3MotorDriver
{
 public:
  Turtlebot3MotorDriver();
  ~Turtlebot3MotorDriver();
  bool init(String turtlebot3);
  void close(void);
  bool setTorque(bool onoff);
  bool getTorque();
  int readEncoder(int32_t &left_value, int32_t &right_value);
  //hepei add 2020/05/15
  int getAbsEncoder(int id,int32_t value); //2?那yid㏒o0(℅車??)㏒?1(車辰??)㏒????邦?米2??∫3?
  //end
  bool writeVelocity(int64_t left_value, int64_t right_value);
  bool controlMotor(const float wheel_radius, const float wheel_separation, float* value);

 private:
  uint32_t baudrate_;
  float  protocol_version_;
  uint8_t left_wheel_id_;
  uint8_t right_wheel_id_;
  bool torque_;

  uint16_t dynamixel_limit_max_velocity_;

  dynamixel::PortHandler *portHandler_;
  dynamixel::PacketHandler *packetHandler_;

  dynamixel::GroupSyncWrite *groupSyncWriteVelocity_;
  dynamixel::GroupSyncRead *groupSyncReadEncoder_;
  
  //hepei add 2020/05/15
  bool abs_encoder_init_[2]={false};//℅車車辰㊣角???¯3?那??‘㊣那??
  uint32_t abs_encoder_q_count_[2] = {0}; //℅車?⊿車辰??㊣角???¯豕|那y那y?Y
  int left_encoder_buff_[2] = {0};//℅車??㊣角???¯那y?Y?o∩?
  int right_encoder_buff_[2] = {0};//車辰??㊣角???¯那y?Y?o∩?
  bool is_clockwise_[2] = {true};//℅車車辰??℅a?‘﹞??辰㊣那???? true㏒o?y℅a  false㏒o﹞∩℅a
  //end
};

#endif // TRANSBOT_MOTOR_DRIVER_H_
