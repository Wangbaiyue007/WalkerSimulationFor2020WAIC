// Generated by gencpp from file cruiser_msgs/diagnosticCtrlCmd.msg
// DO NOT EDIT!


#ifndef CRUISER_MSGS_MESSAGE_DIAGNOSTICCTRLCMD_H
#define CRUISER_MSGS_MESSAGE_DIAGNOSTICCTRLCMD_H


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
struct diagnosticCtrlCmd_
{
  typedef diagnosticCtrlCmd_<ContainerAllocator> Type;

  diagnosticCtrlCmd_()
    : key(0)
    , ctrlflag(0)  {
    }
  diagnosticCtrlCmd_(const ContainerAllocator& _alloc)
    : key(0)
    , ctrlflag(0)  {
  (void)_alloc;
    }



   typedef int16_t _key_type;
  _key_type key;

   typedef int16_t _ctrlflag_type;
  _ctrlflag_type ctrlflag;





  typedef boost::shared_ptr< ::cruiser_msgs::diagnosticCtrlCmd_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::cruiser_msgs::diagnosticCtrlCmd_<ContainerAllocator> const> ConstPtr;

}; // struct diagnosticCtrlCmd_

typedef ::cruiser_msgs::diagnosticCtrlCmd_<std::allocator<void> > diagnosticCtrlCmd;

typedef boost::shared_ptr< ::cruiser_msgs::diagnosticCtrlCmd > diagnosticCtrlCmdPtr;
typedef boost::shared_ptr< ::cruiser_msgs::diagnosticCtrlCmd const> diagnosticCtrlCmdConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::cruiser_msgs::diagnosticCtrlCmd_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::cruiser_msgs::diagnosticCtrlCmd_<ContainerAllocator> >::stream(s, "", v);
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
struct IsFixedSize< ::cruiser_msgs::diagnosticCtrlCmd_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::cruiser_msgs::diagnosticCtrlCmd_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::cruiser_msgs::diagnosticCtrlCmd_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::cruiser_msgs::diagnosticCtrlCmd_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::cruiser_msgs::diagnosticCtrlCmd_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::cruiser_msgs::diagnosticCtrlCmd_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::cruiser_msgs::diagnosticCtrlCmd_<ContainerAllocator> >
{
  static const char* value()
  {
    return "31951abf4e94f1645603ca9e8079c519";
  }

  static const char* value(const ::cruiser_msgs::diagnosticCtrlCmd_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x31951abf4e94f164ULL;
  static const uint64_t static_value2 = 0x5603ca9e8079c519ULL;
};

template<class ContainerAllocator>
struct DataType< ::cruiser_msgs::diagnosticCtrlCmd_<ContainerAllocator> >
{
  static const char* value()
  {
    return "cruiser_msgs/diagnosticCtrlCmd";
  }

  static const char* value(const ::cruiser_msgs::diagnosticCtrlCmd_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::cruiser_msgs::diagnosticCtrlCmd_<ContainerAllocator> >
{
  static const char* value()
  {
    return "#topic \"/cruiser/diag_ctrl_cmd\"\n\
int16 key\n\
int16 ctrlflag\n\
";
  }

  static const char* value(const ::cruiser_msgs::diagnosticCtrlCmd_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::cruiser_msgs::diagnosticCtrlCmd_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.key);
      stream.next(m.ctrlflag);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct diagnosticCtrlCmd_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::cruiser_msgs::diagnosticCtrlCmd_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::cruiser_msgs::diagnosticCtrlCmd_<ContainerAllocator>& v)
  {
    s << indent << "key: ";
    Printer<int16_t>::stream(s, indent + "  ", v.key);
    s << indent << "ctrlflag: ";
    Printer<int16_t>::stream(s, indent + "  ", v.ctrlflag);
  }
};

} // namespace message_operations
} // namespace ros

#endif // CRUISER_MSGS_MESSAGE_DIAGNOSTICCTRLCMD_H