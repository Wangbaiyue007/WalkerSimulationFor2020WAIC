// Generated by gencpp from file servo_ctrl/EcatLimitSrv.msg
// DO NOT EDIT!


#ifndef SERVO_CTRL_MESSAGE_ECATLIMITSRV_H
#define SERVO_CTRL_MESSAGE_ECATLIMITSRV_H

#include <ros/service_traits.h>


#include <servo_ctrl/EcatLimitSrvRequest.h>
#include <servo_ctrl/EcatLimitSrvResponse.h>


namespace servo_ctrl
{

struct EcatLimitSrv
{

typedef EcatLimitSrvRequest Request;
typedef EcatLimitSrvResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct EcatLimitSrv
} // namespace servo_ctrl


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::servo_ctrl::EcatLimitSrv > {
  static const char* value()
  {
    return "e3d452c6cafa43f5461e27d1edc8c915";
  }

  static const char* value(const ::servo_ctrl::EcatLimitSrv&) { return value(); }
};

template<>
struct DataType< ::servo_ctrl::EcatLimitSrv > {
  static const char* value()
  {
    return "servo_ctrl/EcatLimitSrv";
  }

  static const char* value(const ::servo_ctrl::EcatLimitSrv&) { return value(); }
};


// service_traits::MD5Sum< ::servo_ctrl::EcatLimitSrvRequest> should match
// service_traits::MD5Sum< ::servo_ctrl::EcatLimitSrv >
template<>
struct MD5Sum< ::servo_ctrl::EcatLimitSrvRequest>
{
  static const char* value()
  {
    return MD5Sum< ::servo_ctrl::EcatLimitSrv >::value();
  }
  static const char* value(const ::servo_ctrl::EcatLimitSrvRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::servo_ctrl::EcatLimitSrvRequest> should match
// service_traits::DataType< ::servo_ctrl::EcatLimitSrv >
template<>
struct DataType< ::servo_ctrl::EcatLimitSrvRequest>
{
  static const char* value()
  {
    return DataType< ::servo_ctrl::EcatLimitSrv >::value();
  }
  static const char* value(const ::servo_ctrl::EcatLimitSrvRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::servo_ctrl::EcatLimitSrvResponse> should match
// service_traits::MD5Sum< ::servo_ctrl::EcatLimitSrv >
template<>
struct MD5Sum< ::servo_ctrl::EcatLimitSrvResponse>
{
  static const char* value()
  {
    return MD5Sum< ::servo_ctrl::EcatLimitSrv >::value();
  }
  static const char* value(const ::servo_ctrl::EcatLimitSrvResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::servo_ctrl::EcatLimitSrvResponse> should match
// service_traits::DataType< ::servo_ctrl::EcatLimitSrv >
template<>
struct DataType< ::servo_ctrl::EcatLimitSrvResponse>
{
  static const char* value()
  {
    return DataType< ::servo_ctrl::EcatLimitSrv >::value();
  }
  static const char* value(const ::servo_ctrl::EcatLimitSrvResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // SERVO_CTRL_MESSAGE_ECATLIMITSRV_H
