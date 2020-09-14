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

#ifndef TRANSBOT_H_
#define TRANSBOT_H_

#define NAME                             "Normal"

#define WHEEL_RADIUS                     0.033           // meter
#define WHEEL_SEPARATION                 0.231           // meter (TRANSBOT : 0.231)   
#define TURNING_RADIUS                   0.1155           // meter (TRANSBOT : 0.1105)
#define ROBOT_RADIUS                     0.231          // meter (TRANSBOT : 0.221)
#define ENCODER_MIN                      -2147483648     // raw
#define ENCODER_MAX                      2147483648      // raw

#define MAX_LINEAR_VELOCITY              (WHEEL_RADIUS * 2 * 3.14159265359 * 61 / 60) // m/s  (TRANSBOT : 61[rpm])
#define MAX_ANGULAR_VELOCITY             (MAX_LINEAR_VELOCITY / TURNING_RADIUS)       // rad/s

#define MIN_LINEAR_VELOCITY              -MAX_LINEAR_VELOCITY  
#define MIN_ANGULAR_VELOCITY             -MAX_ANGULAR_VELOCITY 

#endif  //TRANSBOT_BURGER_H_