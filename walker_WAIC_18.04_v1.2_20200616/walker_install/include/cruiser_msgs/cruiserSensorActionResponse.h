// Generated by gencpp from file cruiser_msgs/cruiserSensorActionResponse.msg
// DO NOT EDIT!


#ifndef CRUISER_MSGS_MESSAGE_CRUISERSENSORACTIONRESPONSE_H
#define CRUISER_MSGS_MESSAGE_CRUISERSENSORACTIONRESPONSE_H


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
struct cruiserSensorActionResponse_
{
  typedef cruiserSensorActionResponse_<ContainerAllocator> Type;

  cruiserSensorActionResponse_()
    : result(0)  {
    }
  cruiserSensorActionResponse_(const ContainerAllocator& _alloc)
    : result(0)  {
  (void)_alloc;
    }



   typedef uint32_t _result_type;
  _result_type result;





  typedef boost::shared_ptr< ::cruiser_msgs::cruiserSensorActionResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::cruiser_msgs::cruiserSensorActionResponse_<ContainerAllocator> const> ConstPtr;

}; // struct cruiserSensorActionResponse_

typedef ::cruiser_msgs::cruiserSensorActionResponse_<std::allocator<void> > cruiserSensorActionResponse;

typedef boost::shared_ptr< ::cruiser_msgs::cruiserSensorActionResponse > cruiserSensorActionResponsePtr;
typedef boost::shared_ptr< ::cruiser_msgs::cruiserSensorActionResponse const> cruiserSensorActionResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::cruiser_msgs::cruiserSensorActionResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::cruiser_msgs::cruiserSensorActionResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::cruiser_msgs::cruiserSensorActionResponse_<ContainerAllocator1> & lhs, const ::cruiser_msgs::cruiserSensorActionResponse_<ContainerAllocator2> & rhs)
{
  return lhs.result == rhs.result;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::cruiser_msgs::cruiserSensorActionResponse_<ContainerAllocator1> & lhs, const ::cruiser_msgs::cruiserSensorActionResponse_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace cruiser_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::cruiser_msgs::cruiserSensorActionResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::cruiser_msgs::cruiserSensorActionResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::cruiser_msgs::cruiserSensorActionResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::cruiser_msgs::cruiserSensorActionResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::cruiser_msgs::cruiserSensorActionResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::cruiser_msgs::cruiserSensorActionResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::cruiser_msgs::cruiserSensorActionResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "13d5d28ceaaadbc975dd072a2e10b88a";
  }

  static const char* value(const ::cruiser_msgs::cruiserSensorActionResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x13d5d28ceaaadbc9ULL;
  static const uint64_t static_value2 = 0x75dd072a2e10b88aULL;
};

template<class ContainerAllocator>
struct DataType< ::cruiser_msgs::cruiserSensorActionResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "cruiser_msgs/cruiserSensorActionResponse";
  }

  static const char* value(const ::cruiser_msgs::cruiserSensorActionResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::cruiser_msgs::cruiserSensorActionResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "uint32 result\n"
"\n"
"\n"
;
  }

  static const char* value(const ::cruiser_msgs::cruiserSensorActionResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::cruiser_msgs::cruiserSensorActionResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.result);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct cruiserSensorActionResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::cruiser_msgs::cruiserSensorActionResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::cruiser_msgs::cruiserSensorActionResponse_<ContainerAllocator>& v)
  {
    s << indent << "result: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.result);
  }
};

} // namespace message_operations
} // namespace ros

#endif // CRUISER_MSGS_MESSAGE_CRUISERSENSORACTIONRESPONSE_H
