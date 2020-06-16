// Generated by gencpp from file cruiser_msgs/uwbCoordinateMapRequest.msg
// DO NOT EDIT!


#ifndef CRUISER_MSGS_MESSAGE_UWBCOORDINATEMAPREQUEST_H
#define CRUISER_MSGS_MESSAGE_UWBCOORDINATEMAPREQUEST_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace cruiser_msgs
{
template <class ContainerAllocator>
struct uwbCoordinateMapRequest_
{
  typedef uwbCoordinateMapRequest_<ContainerAllocator> Type;

  uwbCoordinateMapRequest_()
    : intput_x(0.0)
    , intput_y(0.0)
    , intput_theta(0.0)
    , theta_valid_flag(false)  {
    }
  uwbCoordinateMapRequest_(const ContainerAllocator& _alloc)
    : intput_x(0.0)
    , intput_y(0.0)
    , intput_theta(0.0)
    , theta_valid_flag(false)  {
  (void)_alloc;
    }



   typedef float _intput_x_type;
  _intput_x_type intput_x;

   typedef float _intput_y_type;
  _intput_y_type intput_y;

   typedef float _intput_theta_type;
  _intput_theta_type intput_theta;

   typedef uint8_t _theta_valid_flag_type;
  _theta_valid_flag_type theta_valid_flag;





  typedef boost::shared_ptr< ::cruiser_msgs::uwbCoordinateMapRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::cruiser_msgs::uwbCoordinateMapRequest_<ContainerAllocator> const> ConstPtr;

}; // struct uwbCoordinateMapRequest_

typedef ::cruiser_msgs::uwbCoordinateMapRequest_<std::allocator<void> > uwbCoordinateMapRequest;

typedef boost::shared_ptr< ::cruiser_msgs::uwbCoordinateMapRequest > uwbCoordinateMapRequestPtr;
typedef boost::shared_ptr< ::cruiser_msgs::uwbCoordinateMapRequest const> uwbCoordinateMapRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::cruiser_msgs::uwbCoordinateMapRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::cruiser_msgs::uwbCoordinateMapRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace cruiser_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'nav_msgs': ['/opt/ros/kinetic/share/nav_msgs/cmake/../msg'], 'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg'], 'actionlib_msgs': ['/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg'], 'sensor_msgs': ['/opt/ros/kinetic/share/sensor_msgs/cmake/../msg'], 'cruiser_msgs': ['/home/cjl/core_ws/walker_ws/walker2_motion_output/src/ros_common/cruiser_msgs/msg'], 'geometry_msgs': ['/opt/ros/kinetic/share/geometry_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::cruiser_msgs::uwbCoordinateMapRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::cruiser_msgs::uwbCoordinateMapRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::cruiser_msgs::uwbCoordinateMapRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::cruiser_msgs::uwbCoordinateMapRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::cruiser_msgs::uwbCoordinateMapRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::cruiser_msgs::uwbCoordinateMapRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::cruiser_msgs::uwbCoordinateMapRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "7f6018a742f32b139af5bdf538b4633a";
  }

  static const char* value(const ::cruiser_msgs::uwbCoordinateMapRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x7f6018a742f32b13ULL;
  static const uint64_t static_value2 = 0x9af5bdf538b4633aULL;
};

template<class ContainerAllocator>
struct DataType< ::cruiser_msgs::uwbCoordinateMapRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "cruiser_msgs/uwbCoordinateMapRequest";
  }

  static const char* value(const ::cruiser_msgs::uwbCoordinateMapRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::cruiser_msgs::uwbCoordinateMapRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "float32 intput_x\n\
float32 intput_y\n\
float32 intput_theta\n\
bool theta_valid_flag\n\
";
  }

  static const char* value(const ::cruiser_msgs::uwbCoordinateMapRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::cruiser_msgs::uwbCoordinateMapRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.intput_x);
      stream.next(m.intput_y);
      stream.next(m.intput_theta);
      stream.next(m.theta_valid_flag);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct uwbCoordinateMapRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::cruiser_msgs::uwbCoordinateMapRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::cruiser_msgs::uwbCoordinateMapRequest_<ContainerAllocator>& v)
  {
    s << indent << "intput_x: ";
    Printer<float>::stream(s, indent + "  ", v.intput_x);
    s << indent << "intput_y: ";
    Printer<float>::stream(s, indent + "  ", v.intput_y);
    s << indent << "intput_theta: ";
    Printer<float>::stream(s, indent + "  ", v.intput_theta);
    s << indent << "theta_valid_flag: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.theta_valid_flag);
  }
};

} // namespace message_operations
} // namespace ros

#endif // CRUISER_MSGS_MESSAGE_UWBCOORDINATEMAPREQUEST_H