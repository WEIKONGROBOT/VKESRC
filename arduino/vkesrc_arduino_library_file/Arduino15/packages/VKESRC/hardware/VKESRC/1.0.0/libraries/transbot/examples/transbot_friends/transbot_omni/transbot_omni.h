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

#include <math.h>

#include <VKPS2.h>

#include "transbot_omni_motor_driver.h"

//#define DEBUG

//change by jiapengfeng for test 20200824
#define WHEEL_RADIUS                    0.029      // meter
#define WHEEL_SEPARATION_RADIUS         0.265    // 170mm/2 + 31mm/2
#define DISTANCE_CENTER_TO_WHEEL        0.165     // meter

#define ENCODER_MIN                     -2147483648     // raw
#define ENCODER_MAX                     2147483648      // raw

#define RPM_CONSTANT_VALUE              0.732

//change by jiapengfeng for test 20200824

#define CONTROL_PERIOD                  8000

#define MAX_LINEAR_VELOCITY             0.22   // m/s
#define MAX_ANGULAR_VELOCITY            2.84   // rad/s
#define VELOCITY_LINEAR_X               0.01   // m/s
#define VELOCITY_LINEAR_Y               0.01   // m/s
#define VELOCITY_ANGULAR_Z              0.1    // rad/s
#define SCALE_VELOCITY_LINEAR_X         1
#define SCALE_VELOCITY_LINEAR_Y         1
#define SCALE_VELOCITY_ANGULAR_Z        1

#define OMNIWHEEL_NUM                4
//change by jiapengfeng for test 20200824
#define LIMIT_X_MAX_VALUE               127
#define SIN45                           0.707106
#define COS45                           0.707106
//change by jiapengfeng for test 20200824
#define DEG2RAD(x)                      (x * 0.01745329252)  // *PI/180
#define RAD2DEG(x)                      (x * 57.2957795131)  // *180/PI

//change by jiapengfeng for test 20200824
#define DXL_LOBYTE(w)       ((uint8_t)(((uint16_t)(w)) & 0xff))
#define DXL_HIBYTE(w)       ((uint8_t)((((uint16_t)(w)) >> 8) & 0xff))
//change by jiapengfeng for test 20200824

void receiveRemoteControlData(void);
void controlMotorSpeed(void);
void controlomni();
