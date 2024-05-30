#ifndef _ROS_erp42_msgs_CANFeedBack_h
#define _ROS_erp42_msgs_CANFeedBack_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace erp42_msgs
{

  class CANFeedBack : public ros::Msg
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
      typedef uint8_t _brake_type;
      _brake_type brake;
      typedef uint8_t _alive_type;
      _alive_type alive;
      typedef int32_t _encoder_type;
      _encoder_type encoder;
      typedef uint8_t _brake_cmd_raw_type;
      _brake_cmd_raw_type brake_cmd_raw;
      typedef uint8_t _brake_raw_type;
      _brake_raw_type brake_raw;
      typedef uint8_t _brake_echo_type;
      _brake_echo_type brake_echo;
      typedef uint8_t _brake_init_max_type;
      _brake_init_max_type brake_init_max;

    CANFeedBack():
      MorA(0),
      EStop(0),
      Gear(0),
      speed(0),
      steer(0),
      brake(0),
      alive(0),
      encoder(0),
      brake_cmd_raw(0),
      brake_raw(0),
      brake_echo(0),
      brake_init_max(0)
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
      *(outbuffer + offset + 0) = (this->brake >> (8 * 0)) & 0xFF;
      offset += sizeof(this->brake);
      *(outbuffer + offset + 0) = (this->alive >> (8 * 0)) & 0xFF;
      offset += sizeof(this->alive);
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
      *(outbuffer + offset + 0) = (this->brake_cmd_raw >> (8 * 0)) & 0xFF;
      offset += sizeof(this->brake_cmd_raw);
      *(outbuffer + offset + 0) = (this->brake_raw >> (8 * 0)) & 0xFF;
      offset += sizeof(this->brake_raw);
      *(outbuffer + offset + 0) = (this->brake_echo >> (8 * 0)) & 0xFF;
      offset += sizeof(this->brake_echo);
      *(outbuffer + offset + 0) = (this->brake_init_max >> (8 * 0)) & 0xFF;
      offset += sizeof(this->brake_init_max);
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
      this->brake =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->brake);
      this->alive =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->alive);
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
      this->brake_cmd_raw =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->brake_cmd_raw);
      this->brake_raw =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->brake_raw);
      this->brake_echo =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->brake_echo);
      this->brake_init_max =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->brake_init_max);
     return offset;
    }

    virtual const char * getType() override { return "erp42_msgs/CANFeedBack"; };
    virtual const char * getMD5() override { return "8d74d0371fb1f42def9619b2296ec65d"; };

  };

}
#endif
