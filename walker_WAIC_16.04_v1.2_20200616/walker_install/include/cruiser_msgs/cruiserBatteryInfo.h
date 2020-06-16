// Generated by gencpp from file cruiser_msgs/cruiserBatteryInfo.msg
// DO NOT EDIT!


#ifndef CRUISER_MSGS_MESSAGE_CRUISERBATTERYINFO_H
#define CRUISER_MSGS_MESSAGE_CRUISERBATTERYINFO_H


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
struct cruiserBatteryInfo_
{
  typedef cruiserBatteryInfo_<ContainerAllocator> Type;

  cruiserBatteryInfo_()
    : battery_level(0)
    , voltage(0)
    , charge_status(0)
    , temperature(0)  {
    }
  cruiserBatteryInfo_(const ContainerAllocator& _alloc)
    : battery_level(0)
    , voltage(0)
    , charge_status(0)
    , temperature(0)  {
  (void)_alloc;
    }



   typedef uint32_t _battery_level_type;
  _battery_level_type battery_level;

   typedef uint32_t _voltage_type;
  _voltage_type voltage;

   typedef uint32_t _charge_status_type;
  _charge_status_type charge_status;

   typedef uint32_t _temperature_type;
  _temperature_type temperature;





  typedef boost::shared_ptr< ::cruiser_msgs::cruiserBatteryInfo_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::cruiser_msgs::cruiserBatteryInfo_<ContainerAllocator> const> ConstPtr;

}; // struct cruiserBatteryInfo_

typedef ::cruiser_msgs::cruiserBatteryInfo_<std::allocator<void> > cruiserBatteryInfo;

typedef boost::shared_ptr< ::cruiser_msgs::cruiserBatteryInfo > cruiserBatteryInfoPtr;
typedef boost::shared_ptr< ::cruiser_msgs::cruiserBatteryInfo const> cruiserBatteryInfoConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::cruiser_msgs::cruiserBatteryInfo_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::cruiser_msgs::cruiserBatteryInfo_<ContainerAllocator> >::stream(s, "", v);
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
struct IsFixedSize< ::cruiser_msgs::cruiserBatteryInfo_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::cruiser_msgs::cruiserBatteryInfo_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::cruiser_msgs::cruiserBatteryInfo_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::cruiser_msgs::cruiserBatteryInfo_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::cruiser_msgs::cruiserBatteryInfo_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::cruiser_msgs::cruiserBatteryInfo_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::cruiser_msgs::cruiserBatteryInfo_<ContainerAllocator> >
{
  static const char* value()
  {
    return "7c1736befdc88aac8e2cd081c711d9ba";
  }

  static const char* value(const ::cruiser_msgs::cruiserBatteryInfo_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x7c1736befdc88aacULL;
  static const uint64_t static_value2 = 0x8e2cd081c711d9baULL;
};

template<class ContainerAllocator>
struct DataType< ::cruiser_msgs::cruiserBatteryInfo_<ContainerAllocator> >
{
  static const char* value()
  {
    return "cruiser_msgs/cruiserBatteryInfo";
  }

  static const char* value(const ::cruiser_msgs::cruiserBatteryInfo_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::cruiser_msgs::cruiserBatteryInfo_<ContainerAllocator> >
{
  static const char* value()
  {
    return "uint32 battery_level\n\
uint32 voltage\n\
uint32 charge_status\n\
uint32 temperature\n\
\n\
";
  }

  static const char* value(const ::cruiser_msgs::cruiserBatteryInfo_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::cruiser_msgs::cruiserBatteryInfo_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.battery_level);
      stream.next(m.voltage);
      stream.next(m.charge_status);
      stream.next(m.temperature);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct cruiserBatteryInfo_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::cruiser_msgs::cruiserBatteryInfo_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::cruiser_msgs::cruiserBatteryInfo_<ContainerAllocator>& v)
  {
    s << indent << "battery_level: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.battery_level);
    s << indent << "voltage: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.voltage);
    s << indent << "charge_status: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.charge_status);
    s << indent << "temperature: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.temperature);
  }
};

} // namespace message_operations
} // namespace ros

#endif // CRUISER_MSGS_MESSAGE_CRUISERBATTERYINFO_H