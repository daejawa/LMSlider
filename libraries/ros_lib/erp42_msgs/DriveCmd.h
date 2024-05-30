#ifndef _ROS_erp42_msgs_DriveCmd_h
#define _ROS_erp42_msgs_DriveCmd_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace erp42_msgs
{

  class DriveCmd : public ros::Msg
  {
    public:
      typedef uint16_t _KPH_type;
      _KPH_type KPH;
      typedef int16_t _Deg_type;
      _Deg_type Deg;
      typedef uint8_t _brake_type;
      _brake_type brake;

    DriveCmd():
      KPH(0),
      Deg(0),
      brake(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
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
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
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
     return offset;
    }

    virtual const char * getType() override { return "erp42_msgs/DriveCmd"; };
    virtual const char * getMD5() override { return "69a82c1e3ba067908d8fabb7e7d8716a"; };

  };

}
#endif
