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

#ifndef TRANSBOT_MECANUM_MOTOR_DRIVER_H_
#define TRANSBOT_MECANUM_MOTOR_DRIVER_H_

#include <DynamixelSDK.h>


//change by jiapengfeng for test 20200824
// Control table address (VKESSM A-series)
#define ADDR_X_TORQUE_ENABLE            40
#define ADDR_X_OPERATION_MODE           33
#define ADDR_X_GOAL_VELOCITY            46
#define ADDR_X_GOAL_POSITION            42
#define ADDR_X_REALTIME_TICK            120
#define ADDR_X_PRESENT_VELOCITY         58
#define ADDR_X_PRESENT_POSITION         56

#define VALUE_OM_POSITION_MODE            0
#define VALUE_OM_SPEED_MODE               1


// Limit values (XM430-W210-T)
#define LIMIT_X_MAX_VELOCITY            240

// Data Byte Length
#define LEN_X_TORQUE_ENABLE             1
#define LEN_X_GOAL_VELOCITY             2
#define LEN_X_GOAL_POSITION             2
#define LEN_X_REALTIME_TICK             2
#define LEN_X_PRESENT_VELOCITY          2
#define LEN_X_PRESENT_POSITION          2



//change by jiapengfeng for test 20200824


//change by jiapengfeng for test 20200821
#define PROTOCOL_VERSION                1.0     // Dynamixel protocol version 2.0
//change by jiapengfeng for test 20200821

//change by jiapengfeng for test 20200821
#define DXL_LEFT_REAR_ID                1       // ID of left rear motor
#define DXL_RIGHT_REAR_ID               2       // ID of right rear motor
#define DXL_LEFT_FRONT_ID               3       // ID of left front motor
#define DXL_RIGHT_FRONT_ID              4       // ID of right front motor

#define VELOCITY_DIRECTION_LEFT_FRONT_FLAG              -1 
#define VELOCITY_DIRECTION_RIGHT_FRONT_FLAG             1 
#define VELOCITY_DIRECTION_RIGHT_REAR_FLAG              1 
#define VELOCITY_DIRECTION_LEFT_REAR_FLAG               -1 
//change by jiapengfeng for test 20200821

#define BAUDRATE                        1000000 // baud rate of Dynamixel
#define DEVICENAME                      ""      // no need setting on OpenCR

#define TORQUE_ENABLE                   1       // Value for enabling the torque
#define TORQUE_DISABLE                  0       // Value for disabling the torque

class TransbotMotorDriver
{
 public:
  TransbotMotorDriver();
  ~TransbotMotorDriver();
  bool init(void);
  void closeDynamixel(void);
  bool setTorque(uint8_t id, bool onoff);
  //change by jiapengfeng for test 20200824
  bool setOperationMode(uint8_t id, uint8_t mode);
  //change by jiapengfeng for test 20200824
  bool setProfileAcceleration(uint8_t id, uint32_t value);
  bool setProfileVelocity(uint8_t id, uint32_t value);
  bool controlMotor(uint8_t* left_rear_wheel_value, uint8_t* right_rear_wheel_value, uint8_t* left_front_wheel_value, uint8_t* right_front_wheel_value);

 private:
  uint32_t baudrate_;
  float  protocol_version_;
  uint8_t left_rear_wheel_id_, right_rear_wheel_id_;
  uint8_t left_front_wheel_id_, right_front_wheel_id_;

  dynamixel::PortHandler *portHandler_;
  dynamixel::PacketHandler *packetHandler_;

  dynamixel::GroupSyncWrite *groupSyncWriteVelocity_;
};

#endif // TRANSBOT_MECANUM_MOTOR_DRIVER_H_
