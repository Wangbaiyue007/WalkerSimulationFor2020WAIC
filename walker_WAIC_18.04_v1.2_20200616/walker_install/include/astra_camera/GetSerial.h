// Generated by gencpp from file astra_camera/GetSerial.msg
// DO NOT EDIT!


#ifndef ASTRA_CAMERA_MESSAGE_GETSERIAL_H
#define ASTRA_CAMERA_MESSAGE_GETSERIAL_H

#include <ros/service_traits.h>


#include <astra_camera/GetSerialRequest.h>
#include <astra_camera/GetSerialResponse.h>


namespace astra_camera
{

struct GetSerial
{

typedef GetSerialRequest Request;
typedef GetSerialResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct GetSerial
} // namespace astra_camera


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::astra_camera::GetSerial > {
  static const char* value()
  {
    return "fca40cf463282a80db4e2037c8a61741";
  }

  static const char* value(const ::astra_camera::GetSerial&) { return value(); }
};

template<>
struct DataType< ::astra_camera::GetSerial > {
  static const char* value()
  {
    return "astra_camera/GetSerial";
  }

  static const char* value(const ::astra_camera::GetSerial&) { return value(); }
};


// service_traits::MD5Sum< ::astra_camera::GetSerialRequest> should match
// service_traits::MD5Sum< ::astra_camera::GetSerial >
template<>
struct MD5Sum< ::astra_camera::GetSerialRequest>
{
  static const char* value()
  {
    return MD5Sum< ::astra_camera::GetSerial >::value();
  }
  static const char* value(const ::astra_camera::GetSerialRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::astra_camera::GetSerialRequest> should match
// service_traits::DataType< ::astra_camera::GetSerial >
template<>
struct DataType< ::astra_camera::GetSerialRequest>
{
  static const char* value()
  {
    return DataType< ::astra_camera::GetSerial >::value();
  }
  static const char* value(const ::astra_camera::GetSerialRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::astra_camera::GetSerialResponse> should match
// service_traits::MD5Sum< ::astra_camera::GetSerial >
template<>
struct MD5Sum< ::astra_camera::GetSerialResponse>
{
  static const char* value()
  {
    return MD5Sum< ::astra_camera::GetSerial >::value();
  }
  static const char* value(const ::astra_camera::GetSerialResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::astra_camera::GetSerialResponse> should match
// service_traits::DataType< ::astra_camera::GetSerial >
template<>
struct DataType< ::astra_camera::GetSerialResponse>
{
  static const char* value()
  {
    return DataType< ::astra_camera::GetSerial >::value();
  }
  static const char* value(const ::astra_camera::GetSerialResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // ASTRA_CAMERA_MESSAGE_GETSERIAL_H
