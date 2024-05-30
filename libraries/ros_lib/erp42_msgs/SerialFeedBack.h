#ifndef _ROS_erp42_msgs_SerialFeedBack_h
#define _ROS_erp42_msgs_SerialFeedBack_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace erp42_msgs
{

  class SerialFeedBack : public ros::Msg
  {
    public:
      typedef uint8_t _MorA_type;
      _MorA_type MorA;
      typedef uint8_t _EStop_type;
      _EStop_type EStop;
      typedef uint8_t _Gear_type;
      _Gear_type Gear;
      typedef float _speed_type;
      _speed_type speed;
      typedef float _steer_type;
      _steer_type steer;
      typedef int16_t _brake_type;
      _brake_type brake;
      typedef int32_t _encoder_type;
      _encoder_type encoder;
      typedef uint8_t _alive_type;
      _alive_type alive;

    SerialFeedBack():
      MorA(0),
      EStop(0),
      Gear(0),
      speed(0),
      steer(0),
      brake(0),
      encoder(0),
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
      offset += serializeAvrFloat64(outbuffer + offset, this->speed);
      offset += serializeAvrFloat64(outbuffer + offset, this->steer);
      union {
        int16_t real;
        uint16_t base;
      } u_brake;
      u_brake.real = this->brake;
      *(outbuffer + offset + 0) = (u_brake.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_brake.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->brake);
      union {
        int32_t real;
        uint32_t base;
      } u_encoder;
      u_encoder.real = this->encoder;
      *(outbuffer + offset + 0) = (u_encoder.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_encoder.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_encoder.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_encoder.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->encoder);
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
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->speed));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->steer));
      union {
        int16_t real;
        uint16_t base;
      } u_brake;
      u_brake.base = 0;
      u_brake.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_brake.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->brake = u_brake.real;
      offset += sizeof(this->brake);
      union {
        int32_t real;
        uint32_t base;
      } u_encoder;
      u_encoder.base = 0;
      u_encoder.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_encoder.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_encoder.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_encoder.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->encoder = u_encoder.real;
      offset += sizeof(this->encoder);
      this->alive =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->alive);
     return offset;
    }

    virtual const char * getType() override { return "erp42_msgs/SerialFeedBack"; };
    virtual const char * getMD5() override { return "977be1b73fb4d99913310d68e9241255"; };

  };

}
#endif
