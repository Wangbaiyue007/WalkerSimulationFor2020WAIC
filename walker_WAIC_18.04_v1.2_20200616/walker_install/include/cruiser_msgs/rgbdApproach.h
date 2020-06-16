// Generated by gencpp from file cruiser_msgs/rgbdApproach.msg
// DO NOT EDIT!


#ifndef CRUISER_MSGS_MESSAGE_RGBDAPPROACH_H
#define CRUISER_MSGS_MESSAGE_RGBDAPPROACH_H


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
struct rgbdApproach_
{
  typedef rgbdApproach_<ContainerAllocator> Type;

  rgbdApproach_()
    : approachStatus(0)
    , distance(0)  {
    }
  rgbdApproach_(const ContainerAllocator& _alloc)
    : approachStatus(0)
    , distance(0)  {
  (void)_alloc;
    }



   typedef int32_t _approachStatus_type;
  _approachStatus_type approachStatus;

   typedef int32_t _distance_type;
  _distance_type distance;





  typedef boost::shared_ptr< ::cruiser_msgs::rgbdApproach_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::cruiser_msgs::rgbdApproach_<ContainerAllocator> const> ConstPtr;

}; // struct rgbdApproach_

typedef ::cruiser_msgs::rgbdApproach_<std::allocator<void> > rgbdApproach;

typedef boost::shared_ptr< ::cruiser_msgs::rgbdApproach > rgbdApproachPtr;
typedef boost::shared_ptr< ::cruiser_msgs::rgbdApproach const> rgbdApproachConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::cruiser_msgs::rgbdApproach_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::cruiser_msgs::rgbdApproach_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::cruiser_msgs::rgbdApproach_<ContainerAllocator1> & lhs, const ::cruiser_msgs::rgbdApproach_<ContainerAllocator2> & rhs)
{
  return lhs.approachStatus == rhs.approachStatus &&
    lhs.distance == rhs.distance;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::cruiser_msgs::rgbdApproach_<ContainerAllocator1> & lhs, const ::cruiser_msgs::rgbdApproach_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace cruiser_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::cruiser_msgs::rgbdApproach_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::cruiser_msgs::rgbdApproach_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::cruiser_msgs::rgbdApproach_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::cruiser_msgs::rgbdApproach_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::cruiser_msgs::rgbdApproach_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::cruiser_msgs::rgbdApproach_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::cruiser_msgs::rgbdApproach_<ContainerAllocator> >
{
  static const char* value()
  {
    return "69a51e326617d1f179799e297efe015b";
  }

  static const char* value(const ::cruiser_msgs::rgbdApproach_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x69a51e326617d1f1ULL;
  static const uint64_t static_value2 = 0x79799e297efe015bULL;
};

template<class ContainerAllocator>
struct DataType< ::cruiser_msgs::rgbdApproach_<ContainerAllocator> >
{
  static const char* value()
  {
    return "cruiser_msgs/rgbdApproach";
  }

  static const char* value(const ::cruiser_msgs::rgbdApproach_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::cruiser_msgs::rgbdApproach_<ContainerAllocator> >
{
  static const char* value()
  {
    return "#topic approachDetect\n"
"# approach status,1=approach 2=leave\n"
"int32 approachStatus\n"
"\n"
"# approach distance cm\n"
"int32 distance\n"
"\n"
;
  }

  static const char* value(const ::cruiser_msgs::rgbdApproach_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::cruiser_msgs::rgbdApproach_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.approachStatus);
      stream.next(m.distance);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct rgbdApproach_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::cruiser_msgs::rgbdApproach_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::cruiser_msgs::rgbdApproach_<ContainerAllocator>& v)
  {
    s << indent << "approachStatus: ";
    Printer<int32_t>::stream(s, indent + "  ", v.approachStatus);
    s << indent << "distance: ";
    Printer<int32_t>::stream(s, indent + "  ", v.distance);
  }
};

} // namespace message_operations
} // namespace ros

#endif // CRUISER_MSGS_MESSAGE_RGBDAPPROACH_H