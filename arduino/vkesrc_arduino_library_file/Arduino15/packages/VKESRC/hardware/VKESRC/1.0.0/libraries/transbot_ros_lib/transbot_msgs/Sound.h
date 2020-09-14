#ifndef _ROS_transbot_msgs_Sound_h
#define _ROS_transbot_msgs_Sound_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace transbot_msgs
{

  class Sound : public ros::Msg
  {
    public:
      typedef uint8_t _value_type;
      _value_type value;
      enum { OFF =  0 };
      enum { ON =  1 };
      enum { LOW_BATTERY =  2 };
      enum { ERROR =  3 };
      enum { BUTTON1 =  4 };
      enum { BUTTON2 =  5 };

    Sound():
      value(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->value >> (8 * 0)) & 0xFF;
      offset += sizeof(this->value);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      this->value =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->value);
     return offset;
    }
//change by jiapengfeng 20200722 begin
    const char * getType(){ return "transbot_msgs/Sound"; };
//change by jiapengfeng 20200722 end
    const char * getMD5(){ return "e1f8c7f8a9a61383b5734fbdeca2f99a"; };

  };

}
#endif