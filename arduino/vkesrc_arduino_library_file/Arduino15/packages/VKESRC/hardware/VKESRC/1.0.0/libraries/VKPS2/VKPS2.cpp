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

#include "VKPS2.h"

VKPS2::VKPS2()
{
}

VKPS2::~VKPS2()
{
}

void VKPS2::begin(int num)
{
  if(num == 1)
  {
    Serial2.begin(9600);
    number = num;
  }
  else if(num == 2)
  {
    Serial4.begin(9600);
    number = num;
  }
  else
  {
    Serial2.begin(9600);
    number = 1;
  }
  VKPS2_rx.state = 0;
  VKPS2_rx.index = 0;
  VKPS2_rx.received = false;
}

int VKPS2::available(void)
{
  if (number == 1)
  {
    if(Serial2.available())
    {
      return VKPS2Update(Serial2.read());
    }
  }
  else if (number == 2)
  {
    if(Serial4.available())
    {
      return VKPS2Update(Serial4.read());
    }
  }

  return false;
}

uint16_t VKPS2::readData(void)
{
  return VKPS2_rx.data;
}

bool VKPS2::VKPS2Update(uint8_t data)
{
  bool ret = false;
  static uint8_t save_data;
  static uint32_t time_t;

  if (millis()-time_t > 100)
  {
    VKPS2_rx.state = 0;
  }

  switch(VKPS2_rx.state)
  {
    case 0:
      if (data == PS2PROTOCOL_HEADER)
      {
        VKPS2_rx.state = 1;
		VKPS2_rx.received = false;
		VKPS2_rx.data     = 0;
        time_t = millis();
      }
	  else
	  {
		 VKPS2_rx.state = 0; 
	  }
      break;

    case 1:
      save_data      = data;
      VKPS2_rx.state = 2;
      break;

    case 2:
	  if (data == 0x00)
      {
        VKPS2_rx.state    = 3;
        VKPS2_rx.received = false;
      }
      else
      {
        VKPS2_rx.state = 0;
      }
      break;
    case 3:
	  if (data == PS2PROTOCOL_TRAIL)
      {
		VKPS2_rx.data  = save_data;
        VKPS2_rx.received = true;
		ret = true;
		save_data = 0;
      }
      VKPS2_rx.state = 0;
      break;
    default:
      VKPS2_rx.state = 0;
      break;
  }

  return ret;
}
