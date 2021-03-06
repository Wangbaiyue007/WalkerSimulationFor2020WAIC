// Generated by gencpp from file cruiser_msgs/cruiser_diagnostics.msg
// DO NOT EDIT!


#ifndef CRUISER_MSGS_MESSAGE_CRUISER_DIAGNOSTICS_H
#define CRUISER_MSGS_MESSAGE_CRUISER_DIAGNOSTICS_H


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
struct cruiser_diagnostics_
{
  typedef cruiser_diagnostics_<ContainerAllocator> Type;

  cruiser_diagnostics_()
    : level(0)
    , name()
    , message()
    , key(0)
    , value(0)
    , data(0)  {
    }
  cruiser_diagnostics_(const ContainerAllocator& _alloc)
    : level(0)
    , name(_alloc)
    , message(_alloc)
    , key(0)
    , value(0)
    , data(0)  {
  (void)_alloc;
    }



   typedef int8_t _level_type;
  _level_type level;

   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _name_type;
  _name_type name;

   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _message_type;
  _message_type message;

   typedef int32_t _key_type;
  _key_type key;

   typedef int32_t _value_type;
  _value_type value;

   typedef int32_t _data_type;
  _data_type data;





  typedef boost::shared_ptr< ::cruiser_msgs::cruiser_diagnostics_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::cruiser_msgs::cruiser_diagnostics_<ContainerAllocator> const> ConstPtr;

}; // struct cruiser_diagnostics_

typedef ::cruiser_msgs::cruiser_diagnostics_<std::allocator<void> > cruiser_diagnostics;

typedef boost::shared_ptr< ::cruiser_msgs::cruiser_diagnostics > cruiser_diagnosticsPtr;
typedef boost::shared_ptr< ::cruiser_msgs::cruiser_diagnostics const> cruiser_diagnosticsConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::cruiser_msgs::cruiser_diagnostics_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::cruiser_msgs::cruiser_diagnostics_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::cruiser_msgs::cruiser_diagnostics_<ContainerAllocator1> & lhs, const ::cruiser_msgs::cruiser_diagnostics_<ContainerAllocator2> & rhs)
{
  return lhs.level == rhs.level &&
    lhs.name == rhs.name &&
    lhs.message == rhs.message &&
    lhs.key == rhs.key &&
    lhs.value == rhs.value &&
    lhs.data == rhs.data;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::cruiser_msgs::cruiser_diagnostics_<ContainerAllocator1> & lhs, const ::cruiser_msgs::cruiser_diagnostics_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace cruiser_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::cruiser_msgs::cruiser_diagnostics_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::cruiser_msgs::cruiser_diagnostics_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::cruiser_msgs::cruiser_diagnostics_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::cruiser_msgs::cruiser_diagnostics_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::cruiser_msgs::cruiser_diagnostics_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::cruiser_msgs::cruiser_diagnostics_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::cruiser_msgs::cruiser_diagnostics_<ContainerAllocator> >
{
  static const char* value()
  {
    return "2acd95d3e541d4e596dab4dec2608e90";
  }

  static const char* value(const ::cruiser_msgs::cruiser_diagnostics_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x2acd95d3e541d4e5ULL;
  static const uint64_t static_value2 = 0x96dab4dec2608e90ULL;
};

template<class ContainerAllocator>
struct DataType< ::cruiser_msgs::cruiser_diagnostics_<ContainerAllocator> >
{
  static const char* value()
  {
    return "cruiser_msgs/cruiser_diagnostics";
  }

  static const char* value(const ::cruiser_msgs::cruiser_diagnostics_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::cruiser_msgs::cruiser_diagnostics_<ContainerAllocator> >
{
  static const char* value()
  {
    return "#topic /cruiser/diagnostics\n"
"# level -- ok:0, warn:1, error:2\n"
"byte level\n"
"string name\n"
"string message\n"
"int32 key\n"
"int32 value\n"
"int32 data\n"
;
  }

  static const char* value(const ::cruiser_msgs::cruiser_diagnostics_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::cruiser_msgs::cruiser_diagnostics_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.level);
      stream.next(m.name);
      stream.next(m.message);
      stream.next(m.key);
      stream.next(m.value);
      stream.next(m.data);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct cruiser_diagnostics_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::cruiser_msgs::cruiser_diagnostics_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::cruiser_msgs::cruiser_diagnostics_<ContainerAllocator>& v)
  {
    s << indent << "level: ";
    Printer<int8_t>::stream(s, indent + "  ", v.level);
    s << indent << "name: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.name);
    s << indent << "message: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.message);
    s << indent << "key: ";
    Printer<int32_t>::stream(s, indent + "  ", v.key);
    s << indent << "value: ";
    Printer<int32_t>::stream(s, indent + "  ", v.value);
    s << indent << "data: ";
    Printer<int32_t>::stream(s, indent + "  ", v.data);
  }
};

} // namespace message_operations
} // namespace ros

#endif // CRUISER_MSGS_MESSAGE_CRUISER_DIAGNOSTICS_H
