/*******************************************************************************
* Copyright (c) 2016, VKROBOT CO., LTD.
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* * Redistributions of source code must retain the above copyright notice, this
*   list of conditions and the following disclaimer.
*
* * Redistributions in binary form must reproduce the above copyright notice,
*   this list of conditions and the following disclaimer in the documentation
*   and/or other materials provided with the distribution.
*
* * Neither the name of VKROBOT nor the names of its
*   contributors may be used to endorse or promote products derived from
*   this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************/

/* Author: jiapeng.feng */

#ifndef VKPS2_H_
#define VKPS2_H_

#include "variant.h"

////////// define RC-100 button key value ////////////////
#define VKPS2_BTN_U				(10)
#define VKPS2_BTN_D				(12)
#define VKPS2_BTN_L				(11)
#define VKPS2_BTN_R				(13)
#define VKPS2_BTN_SQUARE		(0)
#define VKPS2_BTN_CROSS			(1)
#define VKPS2_BTN_CIRCLE		(2)
#define VKPS2_BTN_TRIANGLE		(3)
#define VKPS2_BTN_R1			(4)
#define VKPS2_BTN_R2			(5)

#define PACKET_LENGTH 		6

/**************************************************************************************/
/**********************************遥控器按键定义***************************************/
#define SQUARE      0
#define CROSS       1
#define CIRCLE      2
#define TRIANGLE    3
#define R1          4
#define R2          5
#define R3          6
#define L3          7
#define L2          8
#define L1          9
#define UP          10
#define LEFT        11
#define DOWN        12
#define RIGHT       13
#define SELECT      14
#define START       15
#define PSS_LX      16
#define PSS_LY      17
#define PSS_RX      18
#define PSS_RY      19
#define RCOK    20
/**************************************************************************************/

#define PS2PROTOCOL_HEADER 0XCC
#define PS2PROTOCOL_TRAIL  0XBB
#define KEY_NUM 16
#define KEY_DATA_LEN 4
#define ROCK_DATA_LEN 7

class VKPS2 {
 public:
   VKPS2();
   virtual ~VKPS2();

   void begin(int num);
   int available(void);
   uint16_t readData(void);

 private:

   int8_t number;
   typedef struct
   {
     uint8_t  state;
     uint8_t  index;
     bool     received;
     uint16_t data;
   } VKPS2_t;
     VKPS2_t VKPS2_rx;

  bool VKPS2Update(uint8_t data);
  bool VKPS2Receive(unsigned char *pPacket, int numPacket);
};
#endif /* VKPS2_H_ */
