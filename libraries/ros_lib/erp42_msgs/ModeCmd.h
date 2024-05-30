#ifndef _ROS_erp42_msgs_ModeCmd_h
#define _ROS_erp42_msgs_ModeCmd_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace erp42_msgs
{

  class ModeCmd : public ros::Msg
  {
    public:
      typedef uint8_t _MorA_type;
      _MorA_type MorA;
      typedef uint8_t _EStop_type;
      _EStop_type EStop;
      typedef uint8_t _Gear_type;
      _Gear_type Gear;
      typedef uint8_t _alive_type;
      _alive_type alive;

    ModeCmd():
      MorA(0),
      EStop(0),
      Gear(0),
      alive(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->MorA >> (8 * 0)) & 0xFF;
      offset += sizeof(this->MorA);
      *(outbuffer + offset + 0) = (this->EStop >> (8 * 0)) & 0xFF;
      offset += sizeof(this->EStop);
      *(outbuffer + offset + 0) = (this->Gear >> (8 * 0)) & 0xFF;
      offset += sizeof(this->Gear);
      *(outbuffer + offset + 0) = (this->alive >> (8 * 0)) & 0xFF;
      offset += sizeof(this->alive);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      this->MorA =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->MorA);
      this->EStop =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->EStop);
      this->Gear =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->Gear);
      this->alive =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->alive);
     return offset;
    }

    virtual const char * getType() override { return "erp42_msgs/ModeCmd"; };
    virtual const char * getMD5() override { return "93db1f5d15225b4e964829a0cf6a3bdb"; };

  };

}
#endif
