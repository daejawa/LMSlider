#ifndef _ROS_erp42_msgs_CmdControl_h
#define _ROS_erp42_msgs_CmdControl_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace erp42_msgs
{

  class CmdControl : public ros::Msg
  {
    public:
      typedef uint8_t _MorA_type;
      _MorA_type MorA;
      typedef uint8_t _EStop_type;
      _EStop_type EStop;
      typedef uint8_t _Gear_type;
      _Gear_type Gear;
      typedef uint16_t _KPH_type;
      _KPH_type KPH;
      typedef int16_t _Deg_type;
      _Deg_type Deg;
      typedef uint8_t _brake_type;
      _brake_type brake;
      typedef uint8_t _alive_type;
      _alive_type alive;

    CmdControl():
      MorA(0),
      EStop(0),
      Gear(0),
      KPH(0),
      Deg(0),
      brake(0),
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
      *(outbuffer + offset + 0) = (this->KPH >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->KPH >> (8 * 1)) & 0xFF;
      offset += sizeof(this->KPH);
      union {
        int16_t real;
        uint16_t base;
      } u_Deg;
      u_Deg.real = this->Deg;
      *(outbuffer + offset + 0) = (u_Deg.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_Deg.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->Deg);
      *(outbuffer + offset + 0) = (this->brake >> (8 * 0)) & 0xFF;
      offset += sizeof(this->brake);
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
      this->KPH =  ((uint16_t) (*(inbuffer + offset)));
      this->KPH |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->KPH);
      union {
        int16_t real;
        uint16_t base;
      } u_Deg;
      u_Deg.base = 0;
      u_Deg.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_Deg.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->Deg = u_Deg.real;
      offset += sizeof(this->Deg);
      this->brake =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->brake);
      this->alive =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->alive);
     return offset;
    }

    virtual const char * getType() override { return "erp42_msgs/CmdControl"; };
    virtual const char * getMD5() override { return "17043bad9780a7db1065d8002634203b"; };

  };

}
#endif
