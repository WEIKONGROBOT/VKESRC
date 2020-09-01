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

#include "transbot_omni.h"

/*******************************************************************************
* Declaration for Hardware Timer (Interrupt control)
*******************************************************************************/
HardwareTimer Timer(TIMER_CH1);

/*******************************************************************************
* Declaration for VKPS2 remote conroller
*******************************************************************************/
VKPS2 remote_controller;
double const_cmd_vel    = 0.2;

/*******************************************************************************
* Declaration for motor
*******************************************************************************/
TransbotMotorDriver motor_driver;

double linear_x                = 0.0;
double linear_y                = 0.0;
double angular_z               = 0.0;
double goal_linear_x_velocity  = 0.0;
double goal_linear_y_velocity  = 0.0;
double goal_angular_velocity   = 0.0;

void setup()
{
  Serial.begin(115200);
  // Setting for Dynamixel motors
  motor_driver.init();

  // Setting for VKPS2 remote control and cmd_vel
  remote_controller.begin(1);  //9600bps for VKPS2

  pinMode(13, OUTPUT);
  
//change by jiapengfeng for test 20200825
  pinMode(BDPIN_DIP_SW_1, INPUT);
  pinMode(BDPIN_DIP_SW_2, INPUT);
  pinMode(BDPIN_PUSH_SW_1, INPUT);
  pinMode(BDPIN_PUSH_SW_2, INPUT);
//change by jiapengfeng for test 20200825

  SerialBT2.begin(57600);

  // Start Dynamixel Control Interrupt
  startDynamixelControlInterrupt();
}

void loop()
{
  receiveRemoteControl();
}

void startDynamixelControlInterrupt()
{
  Timer.pause();
  Timer.setPeriod(CONTROL_PERIOD);           // in microseconds
  Timer.attachInterrupt(controlomni);
  Timer.refresh();
  Timer.resume();
}

/*******************************************************************************
* Receive VKPS2 remote controller data
*******************************************************************************/
void receiveRemoteControl(void)
{
  int received_data = 0;
/* 
//change by jiapengfeng for test 20200825
  int dip_state;
  int push_state;
 
  dip_state  = digitalRead(BDPIN_DIP_SW_1)<<0;
  dip_state |= digitalRead(BDPIN_DIP_SW_2)<<1;

  push_state  = digitalRead(BDPIN_PUSH_SW_1)<<0;
  push_state |= digitalRead(BDPIN_PUSH_SW_2)<<1;

 // Serial.print("dip_state = ");
 // Serial.print(dip_state, BIN);

 // Serial.print("\tpush_state = ");
 // Serial.print(push_state, BIN);
//X
    if ((push_state == 2) && (dip_state ==2))
    {
      linear_x  = MAX_LINEAR_VELOCITY;
    }
    else if ((push_state == 1) &&(dip_state ==2))
    {
      linear_x  = (-MAX_LINEAR_VELOCITY);
    }
//Y
    if ((push_state == 1) && (dip_state ==1))
    {
      linear_y = (-MAX_LINEAR_VELOCITY);
    }
    else if ((push_state == 2) && (dip_state ==1))
    {
      linear_y = MAX_LINEAR_VELOCITY;
    }
//Z    
    if ((push_state == 2) && (dip_state ==0))
    {
      angular_z = MAX_ANGULAR_VELOCITY;
    }
    else if ((push_state == 1) && (dip_state ==0))
    {
      angular_z = (-MAX_ANGULAR_VELOCITY);
    }
//  Serial.print("  linear_x : ");  Serial.print(linear_x);
//  Serial.print(" linear_y : "); Serial.print(linear_y);
//  Serial.print(" angular_z : "); Serial.println(angular_z);
//change by jiapengfeng for test 20200825
*/
  if (remote_controller.available())
  {
    received_data = remote_controller.readData();
    Serial.print("received_data = ");
    Serial.print(received_data);
    if (received_data == VKPS2_BTN_U)
    {
      linear_x  += VELOCITY_LINEAR_X * SCALE_VELOCITY_LINEAR_X;
      Serial.println("VKPS2_BTN_U");
    }
    else if (received_data == VKPS2_BTN_D)
    {
      linear_x  -= VELOCITY_LINEAR_X * SCALE_VELOCITY_LINEAR_X;
      Serial.println("VKPS2_BTN_D");
    }
    else if (received_data == VKPS2_BTN_L)
    {
      linear_y -= VELOCITY_LINEAR_Y * SCALE_VELOCITY_LINEAR_Y;
      Serial.println("VKPS2_BTN_L");
    }
    else if (received_data == VKPS2_BTN_R)
    {
      linear_y += VELOCITY_LINEAR_Y * SCALE_VELOCITY_LINEAR_Y;
      Serial.println("VKPS2_BTN_R");
    }
    else if (received_data == VKPS2_BTN_SQUARE)
    {
      angular_z += VELOCITY_ANGULAR_Z * SCALE_VELOCITY_ANGULAR_Z;
      Serial.println("VKPS2_BTN_SQUARE");
    }
    else if (received_data == VKPS2_BTN_CIRCLE)
    {
      angular_z -= VELOCITY_ANGULAR_Z * SCALE_VELOCITY_ANGULAR_Z;
      Serial.println("VKPS2_BTN_CIRCLE");
    }
    else if (received_data == VKPS2_BTN_TRIANGLE)
    {
      linear_x  = const_cmd_vel;
      linear_y  = 0.0;
      angular_z = 0.0;
      Serial.println("VKPS2_BTN_TRIANGLE");
    }
    else if (received_data == VKPS2_BTN_CROSS)
    {
      linear_x  = 0.0;
      linear_y  = 0.0;
      angular_z = 0.0;
      Serial.println("VKPS2_BTN_CROSS");
    }

    if (linear_x > MAX_LINEAR_VELOCITY)
    {
      linear_x = MAX_LINEAR_VELOCITY;
    }
    else if((linear_x < (-MAX_LINEAR_VELOCITY)))
    {
      linear_x = -MAX_LINEAR_VELOCITY;    
    }


    if (linear_y > MAX_LINEAR_VELOCITY)
    {
      linear_y = MAX_LINEAR_VELOCITY;
    }
    else if((linear_y < (-MAX_LINEAR_VELOCITY)))
    {
      linear_y = -MAX_LINEAR_VELOCITY;    
    }
    
    if (angular_z > MAX_ANGULAR_VELOCITY)
    {
      angular_z = MAX_ANGULAR_VELOCITY;
    }
    else if((angular_z < (-MAX_ANGULAR_VELOCITY)))
    {
      angular_z = -MAX_ANGULAR_VELOCITY;    
    }

    goal_linear_x_velocity  = linear_x;
    goal_linear_y_velocity  = linear_y;
    goal_angular_velocity   = angular_z;
  Serial.print("Vx : ");  Serial.print(goal_linear_x_velocity);
  Serial.print(" Vy : "); Serial.print(goal_linear_y_velocity);
  Serial.print(" W : "); Serial.println(goal_angular_velocity);
  }
}

/*******************************************************************************
* Control omni speed
*******************************************************************************/
//change by jiapengfeng for test 20200825
void controlomni()
{
  bool dxl_comm_result = false;

  int16_t wheel_value[OMNIWHEEL_NUM] = {0, 0, 0, 0};
  double wheel_angular_velocity[OMNIWHEEL_NUM] = {0.0, 0.0, 0.0, 0.0};
  uint8_t wheel_value_data_byte[OMNIWHEEL_NUM][2] = {0,0,0,0,0,0,0,0 };
  int8_t velocity_direction_flag[OMNIWHEEL_NUM]= {VELOCITY_DIRECTION_LEFT_FRONT_FLAG,VELOCITY_DIRECTION_RIGHT_FRONT_FLAG,VELOCITY_DIRECTION_LEFT_REAR_FLAG,VELOCITY_DIRECTION_RIGHT_REAR_FLAG};

/*
  wheel_angular_velocity[0] = (1/WHEEL_RADIUS) * (goal_linear_x_velocity - goal_linear_y_velocity - (WHEEL_SEPARATION_X + WHEEL_SEPARATION_Y) * goal_angular_velocity);  //LF
  wheel_angular_velocity[1] = (1/WHEEL_RADIUS) * (goal_linear_x_velocity + goal_linear_y_velocity + (WHEEL_SEPARATION_X + WHEEL_SEPARATION_Y) * goal_angular_velocity);  //RF
  wheel_angular_velocity[2] = (1/WHEEL_RADIUS) * (goal_linear_x_velocity + goal_linear_y_velocity - (WHEEL_SEPARATION_X + WHEEL_SEPARATION_Y) * goal_angular_velocity);  //LR
  wheel_angular_velocity[3] = (1/WHEEL_RADIUS) * (goal_linear_x_velocity - goal_linear_y_velocity + (WHEEL_SEPARATION_X + WHEEL_SEPARATION_Y) * goal_angular_velocity);  //RR
*/
  wheel_angular_velocity[0] = (1/WHEEL_RADIUS) * (-COS45 * goal_linear_x_velocity + SIN45 *  goal_linear_y_velocity  + WHEEL_SEPARATION_RADIUS * goal_angular_velocity);  //LR
  wheel_angular_velocity[1] = (1/WHEEL_RADIUS) * (SIN45 * goal_linear_x_velocity + COS45 *  goal_linear_y_velocity   + WHEEL_SEPARATION_RADIUS * goal_angular_velocity);  //RR
  wheel_angular_velocity[2] = (1/WHEEL_RADIUS) * (-SIN45 * goal_linear_x_velocity - COS45 *  goal_linear_y_velocity  + WHEEL_SEPARATION_RADIUS * goal_angular_velocity);  //LF
  wheel_angular_velocity[3] = (1/WHEEL_RADIUS) * (COS45 * goal_linear_x_velocity - SIN45 *  goal_linear_y_velocity   + WHEEL_SEPARATION_RADIUS * goal_angular_velocity);  //RF

  for (int id = 0; id < OMNIWHEEL_NUM; id++)
  {
    wheel_value[id] = (int16_t)(wheel_angular_velocity[id] * 9.54 / RPM_CONSTANT_VALUE);

    //Match installation direction
    wheel_value[id] *=  velocity_direction_flag[id];
    //Limit value range
    if (wheel_value[id] > LIMIT_X_MAX_VALUE)       wheel_value[id] =  LIMIT_X_MAX_VALUE;
    else if (wheel_value[id] < -LIMIT_X_MAX_VALUE) wheel_value[id] = -LIMIT_X_MAX_VALUE;
    //Match servo data format
    if(wheel_value[id] < 0)
    {
      wheel_value[id] = -wheel_value[id];
      wheel_value[id] |= (1<<15);
    }
    //Match high low bit
    wheel_value_data_byte[id][0] = DXL_LOBYTE(wheel_value[id]);
    wheel_value_data_byte[id][1] = DXL_HIBYTE(wheel_value[id]); 
  }

#ifdef DEBUG
  Serial.print("Vx : ");  Serial.print(goal_linear_x_velocity);
  Serial.print(" Vy : "); Serial.print(goal_linear_y_velocity);
  Serial.print(" W : "); Serial.println(goal_angular_velocity);
#endif
  dxl_comm_result = motor_driver.controlMotor(&wheel_value_data_byte[0][0], &wheel_value_data_byte[1][0], &wheel_value_data_byte[2][0], &wheel_value_data_byte[3][0]);
  if (dxl_comm_result == false)
    return;
}
//change by jiapengfeng for test 20200825
